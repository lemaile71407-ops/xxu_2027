#include "xxu_hardware/chassis_protocol.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/string.hpp>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace xxu_hardware
{
namespace
{

double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

double declare_positive_number(rclcpp::Node & node, const char * name, double initial,
                               bool allow_zero = false)
{
  const double value = node.declare_parameter(name, initial);
  if (!std::isfinite(value) || value < 0.0 || (!allow_zero && value == 0.0)) {
    throw std::invalid_argument(name);
  }
  return value;
}

double declare_seconds(rclcpp::Node & node, const char * name, double initial,
                       bool allow_zero = false)
{
  return declare_positive_number(node, name, initial, allow_zero);
}

double declare_rate_hz(rclcpp::Node & node, const char * name, double initial)
{
  return declare_positive_number(node, name, initial, false);
}

speed_t resolve_baud_constant(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::invalid_argument("unsupported serial baudrate");
  }
}

bool is_valid_command_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000u;
}

bool is_finite_twist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

struct ChassisBridgeConfig {
  std::string serial_port;
  int baudrate = 115200;
  speed_t baud_constant = B115200;

  std::string expected_frame_id;
  bool accept_empty_frame_id = true;
  bool require_fresh_stamp = true;
  std::string base_frame;
  std::string wheel_odom_frame;

  double send_rate_hz = 50.0;
  double command_timeout_sec = 0.3;
  double reconnect_interval_sec = 0.5;

  bool publish_wheel_vel = true;
  bool publish_wheel_odom = true;
  std::string wheel_vel_topic;
  std::string wheel_odom_topic;
  std::string status_topic;
  std::string cmd_vel_topic;
};

}  // namespace

class ChassisLowerBridge final : public rclcpp::Node
{
public:
  explicit ChassisLowerBridge(const rclcpp::NodeOptions & options)
      : Node("chassis_lower_bridge", options)
  {
    load_config();
    init_interfaces();
  }

  ~ChassisLowerBridge() override
  {
    if (serial_fd_ >= 0) {
      write_frame(chassis_state_.shutdown_stop());
      ::close(serial_fd_);
    }
  }

private:
  struct PublishedStatus {
    double timestamp_sec = 0.0;
    bool connected = false;
    bool running = false;
    bool tx_run = false;
    std::string reason;
    std::optional<std::uint16_t> chassis_flags;
  };

  void load_config()
  {
    config_.serial_port = declare_parameter("port", "/dev/ttyACM0");
    config_.baudrate = declare_parameter("baudrate", 115200);
    config_.expected_frame_id = declare_parameter("expected_frame_id", "base_link");
    config_.accept_empty_frame_id = declare_parameter("accept_empty_frame_id", true);
    config_.require_fresh_stamp = declare_parameter("require_fresh_stamp", true);
    config_.base_frame = declare_parameter("base_frame", "base_link");
    config_.wheel_odom_frame = declare_parameter("wheel_odom_frame", "wheel_odom");

    config_.send_rate_hz = declare_rate_hz(*this, "send_rate", 50.0);
    config_.command_timeout_sec = declare_seconds(*this, "cmd_timeout", 0.3, true);
    chassis_state_.status_timeout = declare_seconds(*this, "status_timeout", 0.15);
    chassis_state_.ack_timeout = declare_seconds(*this, "ack_timeout", 0.25, true);
    chassis_state_.require_enable = declare_parameter("require_run_enable", false);
    config_.reconnect_interval_sec = declare_seconds(*this, "reconnect_interval", 0.5);

    if (config_.serial_port.empty() || config_.base_frame.empty() ||
        config_.wheel_odom_frame.empty()) {
      throw std::invalid_argument("empty hardware port/frame");
    }
    // Validate before any timer can open a device.
    config_.baud_constant = resolve_baud_constant(config_.baudrate);

    config_.wheel_vel_topic = declare_parameter("wheel_vel_topic", "/chassis/wheel_vel");
    config_.wheel_odom_topic = declare_parameter("wheel_odom_topic", "/wheel_odom");
    config_.publish_wheel_vel = declare_parameter("publish_wheel_vel", true);
    config_.publish_wheel_odom = declare_parameter("publish_wheel_odom", true);
    config_.status_topic = declare_parameter("status_topic", "/chassis/status");
    config_.cmd_vel_topic = declare_parameter("cmd_vel_topic", "/cmd_vel");
  }

  void init_interfaces()
  {
    if (config_.publish_wheel_vel) {
      velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(config_.wheel_vel_topic,
                                                                         rclcpp::SensorDataQoS());
    }
    if (config_.publish_wheel_odom) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(config_.wheel_odom_topic,
                                                            rclcpp::SensorDataQoS());
    }
    status_pub_ = create_publisher<std_msgs::msg::String>(config_.status_topic,
                                                          rclcpp::QoS(1).transient_local());
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      config_.cmd_vel_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) { on_command(*message); });

    read_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { read_serial(); });
    send_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / config_.send_rate_hz),
                                    [this] { send_command(); });
  }

  void on_command(const geometry_msgs::msg::TwistStamped & message)
  {
    const bool rejected_empty = !config_.accept_empty_frame_id && message.header.frame_id.empty();
    const bool mismatched_frame = !message.header.frame_id.empty() &&
                                  !config_.expected_frame_id.empty() &&
                                  message.header.frame_id != config_.expected_frame_id;
    if (rejected_empty || mismatched_frame) {
      return;
    }

    const auto & twist = message.twist;
    if (!is_valid_command_stamp(message.header.stamp) || !is_finite_twist(twist)) {
      command_received_sec_.reset();
      return;
    }

    try {
      quantize(twist.linear.x);
      quantize(twist.linear.y);
      quantize(twist.angular.z);
    } catch (const std::invalid_argument &) {
      command_received_sec_.reset();
      return;
    }

    latest_command_ = message;
    command_received_sec_ = steady_seconds();
  }

  void reset_odometry_state()
  {
    parser_.clear();
    last_sample_ms_.reset();
    odom_x_m_ = 0.0;
    odom_y_m_ = 0.0;
    odom_yaw_rad_ = 0.0;
  }

  void reset_link(const std::string & reason)
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
    const double now_sec = steady_seconds();
    reconnect_at_sec_ = now_sec + config_.reconnect_interval_sec;
    reset_odometry_state();
    chassis_state_.link_reset(now_sec, reason);
  }

  void open_serial()
  {
    if (serial_fd_ >= 0 || steady_seconds() < reconnect_at_sec_) {
      return;
    }

    serial_fd_ = ::open(config_.serial_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    termios serial_config{};
    if (serial_fd_ < 0 || ::tcgetattr(serial_fd_, &serial_config) != 0) {
      reset_link("serial_open_error");
      return;
    }

    ::cfmakeraw(&serial_config);
    serial_config.c_cflag |= CLOCAL | CREAD;
    serial_config.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
    serial_config.c_cflag = (serial_config.c_cflag & ~CSIZE) | CS8;
    serial_config.c_cc[VMIN] = 0;
    serial_config.c_cc[VTIME] = 0;
    ::cfsetispeed(&serial_config, config_.baud_constant);
    ::cfsetospeed(&serial_config, config_.baud_constant);

    if (::tcsetattr(serial_fd_, TCSANOW, &serial_config) != 0) {
      reset_link("serial_config_error");
      return;
    }

    reset_odometry_state();
    chassis_state_.link_reset(steady_seconds(), "serial_connected");
    RCLCPP_INFO(get_logger(), "Connected to STM32 chassis port %s", config_.serial_port.c_str());
  }

  bool write_frame(const std::vector<std::uint8_t> & bytes)
  {
    const double deadline_sec = steady_seconds() + 0.05;
    std::size_t sent_bytes = 0;
    while (sent_bytes < bytes.size() && steady_seconds() < deadline_sec) {
      const auto written =
        ::write(serial_fd_, bytes.data() + sent_bytes, bytes.size() - sent_bytes);
      if (written > 0) {
        sent_bytes += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        return false;
      }
      pollfd poller{serial_fd_, POLLOUT, 0};
      ::poll(&poller, 1, 1);
    }
    return sent_bytes == bytes.size();
  }

  void read_serial()
  {
    if (serial_fd_ < 0) {
      return;
    }

    std::array<std::uint8_t, 256> buffer{};
    // Bound each callback even if a faulty peer floods the serial device.
    for (int batch = 0; batch < 16; ++batch) {
      const auto bytes_read = ::read(serial_fd_, buffer.data(), buffer.size());
      if (bytes_read <= 0) {
        if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          reset_link("serial_read_error");
        }
        return;
      }
      for (const auto & status :
           parser_.feed(buffer.data(), static_cast<std::size_t>(bytes_read))) {
        chassis_state_.observe(status, steady_seconds());
        publish_feedback(status);
      }
    }
  }

  void publish_feedback(const ChassisStatus & status)
  {
    if (!status.wheel_valid() || (last_sample_ms_ && *last_sample_ms_ == status.sample_ms)) {
      return;
    }

    const double vx_m_s = status.vx * 0.001;
    const double vy_m_s = status.vy * 0.001;
    const double wz_rad_s = status.wz * 0.001;
    if (last_sample_ms_) {
      const auto delta_ms = static_cast<std::uint32_t>(status.sample_ms - *last_sample_ms_);
      const double dt_sec = delta_ms * 0.001;
      if (delta_ms > 0 && delta_ms <= 0x80000000u && dt_sec <= 0.5) {
        const double mid_yaw_rad = odom_yaw_rad_ + 0.5 * wz_rad_s * dt_sec;
        odom_x_m_ += (vx_m_s * std::cos(mid_yaw_rad) - vy_m_s * std::sin(mid_yaw_rad)) * dt_sec;
        odom_y_m_ += (vx_m_s * std::sin(mid_yaw_rad) + vy_m_s * std::cos(mid_yaw_rad)) * dt_sec;
        const double next_yaw_rad = odom_yaw_rad_ + wz_rad_s * dt_sec;
        odom_yaw_rad_ = std::atan2(std::sin(next_yaw_rad), std::cos(next_yaw_rad));
      }
    }

    last_sample_ms_ = status.sample_ms;
    last_pub_ns_ = std::max(now().nanoseconds(), last_pub_ns_ + 1);

    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = rclcpp::Time(last_pub_ns_);
    velocity.header.frame_id = config_.base_frame;
    velocity.twist.linear.x = vx_m_s;
    velocity.twist.linear.y = vy_m_s;
    velocity.twist.angular.z = wz_rad_s;
    if (velocity_pub_) {
      velocity_pub_->publish(velocity);
    }

    if (odom_pub_) {
      nav_msgs::msg::Odometry odom;
      odom.header = velocity.header;
      odom.header.frame_id = config_.wheel_odom_frame;
      odom.child_frame_id = config_.base_frame;
      odom.pose.pose.position.x = odom_x_m_;
      odom.pose.pose.position.y = odom_y_m_;
      odom.pose.pose.orientation.z = std::sin(odom_yaw_rad_ / 2.0);
      odom.pose.pose.orientation.w = std::cos(odom_yaw_rad_ / 2.0);
      odom.twist.twist = velocity.twist;
      odom_pub_->publish(odom);
    }
  }

  void send_command()
  {
    open_serial();
    const double now_sec = steady_seconds();
    if (serial_fd_ >= 0) {
      const bool wall_fresh = command_received_sec_ && (now_sec - *command_received_sec_ >= 0.0) &&
                              (now_sec - *command_received_sec_ <= config_.command_timeout_sec);
      bool stamp_fresh = !config_.require_fresh_stamp;
      if (wall_fresh && config_.require_fresh_stamp) {
        const double age_sec = (now() - rclcpp::Time(latest_command_.header.stamp)).seconds();
        stamp_fresh = age_sec >= 0.0 && age_sec <= config_.command_timeout_sec;
      }

      const bool valid = wall_fresh && stamp_fresh;
      const auto & twist = latest_command_.twist;
      const auto bytes =
        chassis_state_.next(now_sec, twist.linear.x, twist.linear.y, twist.angular.z, valid);
      if (!write_frame(bytes)) {
        reset_link("serial_write_error");
      }
    }
    publish_status();
  }

  void publish_status()
  {
    const double now_sec = steady_seconds();
    const bool connected = serial_fd_ >= 0;
    const bool tx_run = connected && chassis_state_.last_tx_run;
    const auto chassis_flags = chassis_state_.status
                                 ? std::optional<std::uint16_t>{chassis_state_.status->status}
                                 : std::nullopt;

    // Sequence numbers, ages and parser counters update on the heartbeat.
    const bool unchanged_heartbeat = published_status_ &&
                                     (now_sec - published_status_->timestamp_sec < 0.2) &&
                                     (published_status_->connected == connected) &&
                                     (published_status_->running == chassis_state_.running) &&
                                     (published_status_->tx_run == tx_run) &&
                                     (published_status_->reason == chassis_state_.reason) &&
                                     (published_status_->chassis_flags == chassis_flags);
    if (unchanged_heartbeat) {
      return;
    }

    const char * state_name = chassis_state_.running ? "RUNNING" : "WAIT_STOP_ACK";
    const auto & counters = parser_.counters;
    nlohmann::json status{{"connected", connected},
                          {"port", config_.serial_port},
                          {"state", state_name},
                          {"stop_reason", chassis_state_.reason},
                          {"tx", nullptr},
                          {"rx_age_s", chassis_state_.status_time
                                         ? nlohmann::json(now_sec - *chassis_state_.status_time)
                                         : nlohmann::json(nullptr)},
                          {"chassis", nullptr},
                          {"parser",
                           {{"frames_ok", counters.frames_ok},
                            {"bad_header", counters.bad_header},
                            {"bad_version", counters.bad_version},
                            {"bad_type", counters.bad_type},
                            {"bad_length", counters.bad_length},
                            {"bad_crc", counters.bad_crc}}}};

    if (connected) {
      status["tx"] = {{"seq", chassis_state_.last_tx_seq},
                      {"mode", tx_run ? 1 : 0},
                      {"state", state_name},
                      {"stop_reason", chassis_state_.reason},
                      {"invalid_reason", ""}};
    }
    if (chassis_state_.status) {
      const auto & rx_status = *chassis_state_.status;
      status["chassis"] = {{"seq", rx_status.seq},
                           {"last_seq", rx_status.last_seq},
                           {"status_raw", rx_status.status},
                           {"ready", static_cast<bool>(rx_status.status & 1)},
                           {"enabled", static_cast<bool>(rx_status.status & 2)},
                           {"estop", static_cast<bool>(rx_status.status & 4)},
                           {"fault", static_cast<bool>(rx_status.status & 8)},
                           {"comm_timeout", static_cast<bool>(rx_status.status & 16)},
                           {"wheel_valid", rx_status.wheel_valid()},
                           {"sample_time_ms", rx_status.sample_ms}};
    }

    std_msgs::msg::String message;
    message.data = status.dump();
    status_pub_->publish(message);
    published_status_ = PublishedStatus{
      now_sec, connected, chassis_state_.running, tx_run, chassis_state_.reason, chassis_flags};
  }

  // Configuration
  ChassisBridgeConfig config_;

  // Serial and odometry state
  int serial_fd_ = -1;
  double reconnect_at_sec_ = 0.0;
  double odom_x_m_ = 0.0;
  double odom_y_m_ = 0.0;
  double odom_yaw_rad_ = 0.0;
  std::int64_t last_pub_ns_ = 0;
  std::optional<std::uint32_t> last_sample_ms_;
  std::optional<double> command_received_sec_;
  std::optional<PublishedStatus> published_status_;
  ChassisStateMachine chassis_state_;
  FrameStreamParser parser_;
  geometry_msgs::msg::TwistStamped latest_command_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

}  // namespace xxu_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(xxu_hardware::ChassisLowerBridge)
