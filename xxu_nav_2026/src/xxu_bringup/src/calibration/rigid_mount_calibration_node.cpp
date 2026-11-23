#include "rigid_mount_calibration_io.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <mutex>

namespace xxu_bringup {
using namespace calibration_detail;
using namespace calibration_io;
using namespace std::chrono_literals;

class RigidMountCalibrationNode : public rclcpp::Node
{
public:
  explicit RigidMountCalibrationNode(const rclcpp::NodeOptions & options)
  : Node("rigid_mount_calibration", options)
  {
    read_config();
    const auto input_type = fixed_parameter<std::string>("lidar_input_type", "pointcloud2");
    const auto qos = rclcpp::SensorDataQoS();
    if (input_type == "pointcloud2") {
      lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.lidar_topic, qos, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          on_cloud(*msg);
        });
    } else if (input_type == "livox") {
#ifdef XXU_HAVE_LIVOX_CUSTOM_MSG
      lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        config_.lidar_topic, qos, [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
          on_cloud(*msg);
        });
#else
      throw std::invalid_argument("lidar_input_type=livox requires livox_ros_driver2 at build time");
#endif
    } else {
      throw std::invalid_argument("lidar_input_type must be pointcloud2 or livox");
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(config_.imu_topic, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {on_imu(*message);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(config_.odom_topic, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odom(*message);});

    for (const auto stage : {Stage::stationary, Stage::straight, Stage::spin}) {
      add_service(std::string("start_") + name(stage), [this, stage](Response & r) {start(stage, r);});
      add_service(std::string("stop_") + name(stage), [this, stage](Response & r) {stop(stage, r);});
    }
    add_service("capture_stationary", [this](Response & r) {timed_capture(r);});
    add_service("solve_and_export", [this](Response & r) {export_results(r);});
    add_service("reset", [this](Response & r) {
        cancel_capture();
        captures_ = {};
        clear_confirmation();
        r.success = true;
        r.message = "all captures cleared";
      });
    last_status_ = std::chrono::steady_clock::now();
    status_timer_ = create_wall_timer(1s, [this] {status();});
  }

private:
  using Trigger = std_srvs::srv::Trigger;
  using Response = Trigger::Response;
  enum class Stage {stationary, straight, spin, idle};

  static const char * name(Stage stage)
  {
    return std::array{"stationary", "straight", "spin", "idle"}[static_cast<int>(stage)];
  }

  static const char * prompt(Stage stage)
  {
    return std::array{
      "keep the vehicle still with floor in view",
      "drive or push along base X",
      "rotate about the fixed base center",
      "call start_stationary, start_straight, start_spin or solve_and_export"
    }[static_cast<int>(stage)];
  }

  template<typename T>
  T fixed_parameter(const std::string & name, const T & initial)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, initial, descriptor);
  }

  Eigen::Vector3d parameter_vector(
    const std::string & name, const std::vector<double> & initial)
  {
    const auto values = fixed_parameter(name, initial);
    if (values.size() != 3) {throw std::invalid_argument(name + " must have three elements");}
    return {values[0], values[1], values[2]};
  }

  std::optional<double> manual(const std::string & name)
  {
    const auto value = fixed_parameter<std::string>(name, "");
    if (value.empty()) {return {};}
    return parse_number(value, name);
  }

  void read_config()
  {
    auto & c = config_;
    c.imu_topic = fixed_parameter("imu_topic", c.imu_topic);
    c.lidar_topic = fixed_parameter("lidar_topic", c.lidar_topic);
    c.odom_topic = fixed_parameter("odom_topic", c.odom_topic);
    c.allow_unisolated_odom_topic = fixed_parameter("allow_unisolated_odom_topic", c.allow_unisolated_odom_topic);
    c.odom_frame = fixed_parameter("odom_frame", c.odom_frame);
    c.odom_child_frame = fixed_parameter("odom_child_frame", c.odom_child_frame);
    c.imu_acceleration_scale = fixed_parameter("imu_acceleration_scale", c.imu_acceleration_scale);
    c.base_height_above_ground = fixed_parameter("base_height_above_ground", c.base_height_above_ground);
    c.imu_from_lidar.translation() =
      parameter_vector("imu_from_lidar_xyz", {-0.011, -0.02329, 0.04412});
    c.imu_from_lidar.linear() = rpy_rotation(parameter_vector("imu_from_lidar_rpy", {0., 0., 0.}));
    c.straight_direction = fixed_parameter("straight_direction", c.straight_direction);
    c.sensor_frame_mode = fixed_parameter("sensor_frame_mode", c.sensor_frame_mode);
    c.base_frame = fixed_parameter("base_frame", c.base_frame);
    c.sensor_frame = fixed_parameter("sensor_frame", c.sensor_frame);
    c.lidar_frame = fixed_parameter("lidar_frame", c.lidar_frame);
    c.imu_source_frame = fixed_parameter("imu_source_frame", c.imu_source_frame);
    c.manual_psi_deg = manual("manual_yaw_deg");
    c.manual_tx = manual("manual_tx");
    c.manual_ty = manual("manual_ty");
    c.manual_tz = manual("manual_tz");
    declare_parameter("confirm", false);
    for (const auto & [parameter, field] : real_thresholds) {
      c.*field = fixed_parameter(parameter, c.*field);
    }
    for (const auto & [parameter, field] : count_thresholds) {
      c.*field = fixed_parameter(parameter, c.*field);
    }
    outputs_.mount = fixed_parameter<std::string>("output_mount_file", "");
    outputs_.backend = fixed_parameter<std::string>("output_backend_body_file", "");
    outputs_.report = fixed_parameter<std::string>("output_report_file", "");
    stationary_capture_sec_ = fixed_parameter("stationary_capture_sec", 3.0);
    // A wall timer must have a finite, representable period.
    if (!std::isfinite(stationary_capture_sec_) ||
      stationary_capture_sec_ < c.min_stationary_duration || stationary_capture_sec_ > 3600.0)
    {
      throw std::invalid_argument("stationary_capture_sec must be between min_stationary_duration and 3600");
    }
    validate_calibration_config(c);
    validate_output_paths(outputs_);
  }

  CalibrationSamples & buffer(Stage stage) {return captures_[static_cast<int>(stage)];}

  template<typename Callback>
  void add_service(const std::string & suffix, Callback callback)
  {
    services_.push_back(create_service<Trigger>("~/" + suffix,
      [this, callback](Trigger::Request::ConstSharedPtr, Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        try {
          callback(*response);
        } catch (const std::exception & e) {
          response->success = false;
          response->message = e.what();
        }
      }));
  }

  void on_imu(const sensor_msgs::msg::Imu & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_[0];
    if (active_ != Stage::stationary) {return;}
    auto & data = buffer(active_);
    observe(data.imu_frames, message.header.frame_id);
    const auto sample = imu_sample(message);
    if (!valid_imu(sample) || !append_sample(data, data.imu, sample, max_capture_samples, "IMU")) {
      ++data.rejected_imu;
    }
  }

  template<typename Cloud>
  void on_cloud(const Cloud & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_[1];
    if (active_ != Stage::stationary) {return;}
    auto & data = buffer(active_);
    observe(data.lidar_frames, message.header.frame_id);
    const double time = stamp_time(message.header.stamp);
    try {
      if (valid_time(time) && (data.clouds.empty() || data.clouds.back().time < time)) {
        if (data.clouds.size() >= max_clouds) {
          input_error(data, "LiDAR capture limit exceeded; recapture a shorter segment");
        } else {
          auto points = extract_calibration_points(message, max_points_per_cloud);
          if (!points.empty()) {
            data.clouds.push_back({time, std::move(points)});
            return;
          }
        }
      }
    } catch (const std::invalid_argument &) {
      // A malformed cloud is counted and discarded.
    }
    ++data.rejected_clouds;
  }

  void on_odom(const nav_msgs::msg::Odometry & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++received_[2];
    if (active_ != Stage::straight && active_ != Stage::spin) {return;}
    auto & data = buffer(active_);
    observe(data.odom_frames, message.header.frame_id);
    observe(data.odom_child_frames, message.child_frame_id);
    auto & odom = active_ == Stage::straight ? data.straight : data.spin;
    const auto sample = odom_sample(message);
    if (!valid_odom(sample) || !append_sample(data, odom, sample, max_capture_samples, name(active_))) {
      ++data.rejected_odom;
    }
  }

  void clear_confirmation()
  {
    if (get_parameter("confirm").as_bool()) {
      set_parameter(rclcpp::Parameter("confirm", false));
      RCLCPP_INFO(get_logger(), "reset confirm=false after capture change");
    }
  }

  void start(Stage stage, Response & response)
  {
    if (active_ != Stage::idle) {
      response.message = std::string("stop active ") + name(active_) + " capture first";
      return;
    }
    clear_confirmation();
    auto & data = buffer(stage);
    data = CalibrationSamples{};
    data.straight_requested = stage == Stage::straight;
    data.spin_requested = stage == Stage::spin;
    active_ = stage;
    response.success = true;
    response.message = prompt(stage);
  }

  void cancel_capture()
  {
    ++timer_generation_;
    if (capture_timer_) {
      capture_timer_->cancel();
      capture_timer_.reset();
    }
    active_ = Stage::idle;
  }

  void stop(Stage stage, Response & response)
  {
    if (active_ != stage) {
      response.message = std::string("no active ") + name(stage) + " capture";
      return;
    }
    clear_confirmation();
    cancel_capture();
    auto & data = buffer(stage);
    if (stage == Stage::stationary) {
      const auto stationary = solve_stationary(data.imu, data.clouds, config_);
      auto reasons = stationary.reasons;
      reasons.insert(reasons.end(), data.input_errors.begin(), data.input_errors.end());
      if (!data.imu_frames.empty() && data.imu_frames != std::set{config_.imu_source_frame}) {
        reasons.emplace_back("IMU frame does not match imu_source_frame");
      }
      if (!data.lidar_frames.empty() && data.lidar_frames != std::set{config_.lidar_frame}) {
        reasons.emplace_back("LiDAR frame does not match lidar_frame");
      }
      response.success = reasons.empty();
      std::ostringstream message;
      message << "stationary IMU=" << stationary.imu_count << " clouds=" << stationary.cloud_count;
      if (stationary.scale) {message << " scale=" << *stationary.scale;}
      if (stationary.tz) {message << " tz=" << *stationary.tz << " m";}
      if (!reasons.empty()) {message << " | " << summarize(reasons);}
      response.message = message.str();
    } else {
      const auto count = stage == Stage::straight ? data.straight.size() : data.spin.size();
      response.success = count > 0 && data.input_errors.empty();
      response.message = std::string(name(stage)) + " captured odometry=" + std::to_string(count);
      if (!data.input_errors.empty()) {response.message += " | " + summarize(data.input_errors);}
    }
  }

  void timed_capture(Response & response)
  {
    start(Stage::stationary, response);
    if (!response.success) {return;}
    const auto generation = ++timer_generation_;
    capture_timer_ = create_wall_timer(std::chrono::duration<double>(stationary_capture_sec_),
      [this, generation] {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation != timer_generation_) {return;}
        Response result;
        stop(Stage::stationary, result);
        if (result.success) {RCLCPP_INFO(get_logger(), "%s", result.message.c_str());}
        else {RCLCPP_WARN(get_logger(), "%s", result.message.c_str());}
      });
    response.message += " for " + std::to_string(stationary_capture_sec_) + " s";
  }

  void export_results(Response & response)
  {
    if (active_ != Stage::idle) {
      response.message = "stop capture before solving";
      return;
    }
    if (outputs_.mount.empty() && outputs_.backend.empty() && outputs_.report.empty()) {
      response.message = "configure at least one output file";
      return;
    }
    auto data = buffer(Stage::stationary);
    data.straight = buffer(Stage::straight).straight;
    data.spin = buffer(Stage::spin).spin;
    data.straight_requested = buffer(Stage::straight).straight_requested;
    data.spin_requested = buffer(Stage::spin).spin_requested;
    for (const auto stage : {Stage::straight, Stage::spin}) {
      const auto & motion = buffer(stage);
      data.odom_frames.insert(motion.odom_frames.begin(), motion.odom_frames.end());
      data.odom_child_frames.insert(motion.odom_child_frames.begin(), motion.odom_child_frames.end());
      data.rejected_odom += motion.rejected_odom;
      data.input_errors.insert(data.input_errors.end(), motion.input_errors.begin(), motion.input_errors.end());
    }
    auto settings = config_;
    settings.confirm = get_parameter("confirm").as_bool();
    const auto result = solve_all(data, settings);
    const auto errors = write_outputs(outputs_, data, settings, result);
    const bool final_confirmed = result.confirmed && errors.empty();
    response.success = errors.empty() && data.input_errors.empty() &&
      (!settings.confirm || final_confirmed);
    response.message = final_confirmed ? "calibration confirmed" : "calibration unconfirmed";
    if (!result.reasons.empty()) {response.message += " | " + summarize(result.reasons);}
    if (!errors.empty()) {response.message += " | " + summarize(errors);}
  }

  void status()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_status_).count();
    RCLCPP_INFO(get_logger(),
      "capture=%s IMU=%zu (%.1f Hz) LiDAR=%zu (%.1f Hz) odometry=%zu (%.1f Hz). %s",
      name(active_), received_[0], (received_[0] - previous_[0]) / dt,
      received_[1], (received_[1] - previous_[1]) / dt,
      received_[2], (received_[2] - previous_[2]) / dt, prompt(active_));
    previous_ = received_;
    last_status_ = now;
  }

  CalibrationConfig config_;
  OutputFiles outputs_;
  double stationary_capture_sec_;
  Stage active_ = Stage::idle;
  std::mutex mutex_;
  std::array<CalibrationSamples, 3> captures_{};
  std::array<std::size_t, 3> received_{}, previous_{};
  std::chrono::steady_clock::time_point last_status_;
  std::size_t timer_generation_ = 0;
  rclcpp::TimerBase::SharedPtr status_timer_, capture_timer_;
  rclcpp::SubscriptionBase::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> services_;
};

}  // namespace xxu_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_bringup::RigidMountCalibrationNode)
