#include "xxu_hardware/chassis_protocol.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace xxu_hardware {
double steady_seconds()
{return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}

class ChassisLowerBridge final : public rclcpp::Node
{
public:
  explicit ChassisLowerBridge(const rclcpp::NodeOptions & options) : Node("chassis_lower_bridge", options)
  {
    port_ = declare_parameter("port", "/dev/ttyACM0");
    baudrate_ = declare_parameter("baudrate", 115200);
    expected_ = declare_parameter("expected_frame_id", "base_link");
    accept_empty_ = declare_parameter("accept_empty_frame_id", true);
    fresh_stamp_ = declare_parameter("require_fresh_stamp", true);
    base_ = declare_parameter("base_frame", "base_link");
    wheel_frame_ = declare_parameter("wheel_odom_frame", "wheel_odom");
    const double rate = parameter("send_rate", 50.);
    command_timeout_ = parameter("cmd_timeout", .3, true);
    state_.status_timeout = parameter("status_timeout", .15);
    state_.ack_timeout = parameter("ack_timeout", .25, true);
    state_.require_enable = declare_parameter("require_run_enable", false);
    reconnect_ = parameter("reconnect_interval", .5);
    if (port_.empty() || base_.empty() || wheel_frame_.empty()) {throw std::invalid_argument("empty hardware port/frame");}
    baud_constant();  // Validate before any timer can open a device.
    const auto vel_topic = declare_parameter("wheel_vel_topic", "/chassis/wheel_vel");
    const auto odom_topic = declare_parameter("wheel_odom_topic", "/wheel_odom");
    if (declare_parameter("publish_wheel_vel", true)) {
      velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(vel_topic, rclcpp::SensorDataQoS());
    }
    if (declare_parameter("publish_wheel_odom", true)) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS());
    }
    status_pub_ = create_publisher<std_msgs::msg::String>(declare_parameter("status_topic", "/chassis/status"),
      rclcpp::QoS(1).transient_local());
    command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(declare_parameter("cmd_vel_topic", "/cmd_vel"),
      rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {command(*m);});
    reader_ = create_wall_timer(std::chrono::milliseconds(5), [this] {read_serial();});
    sender_ = create_wall_timer(std::chrono::duration<double>(1. / rate), [this] {send();});
  }
  ~ChassisLowerBridge() override
  {
    if (fd_ >= 0) {write_frame(state_.shutdown_stop()); ::close(fd_);}
  }
private:
  double parameter(const char * name, double initial, bool allow_zero = false)
  {
    const double value = declare_parameter(name, initial);
    if (!std::isfinite(value) || value < 0 || (!allow_zero && value == 0)) {throw std::invalid_argument(name);}
    return value;
  }
  speed_t baud_constant() const
  {
    switch (baudrate_) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: throw std::invalid_argument("unsupported serial baudrate");
    }
  }
  void command(const geometry_msgs::msg::TwistStamped & message)
  {
    if ((!accept_empty_ && message.header.frame_id.empty()) || (!message.header.frame_id.empty() &&
      !expected_.empty() && message.header.frame_id != expected_)) {return;}
    const auto & t = message.twist;
    if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000u ||
      !std::isfinite(t.linear.x) || !std::isfinite(t.linear.y) || !std::isfinite(t.linear.z) ||
      !std::isfinite(t.angular.x) || !std::isfinite(t.angular.y) || !std::isfinite(t.angular.z))
    {received_.reset(); return;}
    try {quantize(t.linear.x); quantize(t.linear.y); quantize(t.angular.z);}
    catch (const std::invalid_argument &) {received_.reset(); return;}
    command_ = message; received_ = steady_seconds();
  }
  void reset_link(const std::string & reason)
  {
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
    const double time = steady_seconds();
    reconnect_at_ = time + reconnect_; parser_.clear();
    sample_ms_.reset(); x_ = y_ = yaw_ = 0.; state_.link_reset(time, reason);
    tx_.reset();
  }
  void open_serial()
  {
    if (fd_ >= 0 || steady_seconds() < reconnect_at_) {return;}
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    termios config{};
    if (fd_ < 0 || ::tcgetattr(fd_, &config) != 0) {reset_link("serial_open_error"); return;}
    ::cfmakeraw(&config);
    config.c_cflag |= CLOCAL | CREAD;
    config.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
    config.c_cflag = (config.c_cflag & ~CSIZE) | CS8;
    config.c_cc[VMIN] = 0; config.c_cc[VTIME] = 0;
    ::cfsetispeed(&config, baud_constant()); ::cfsetospeed(&config, baud_constant());
    if (::tcsetattr(fd_, TCSANOW, &config) != 0) {reset_link("serial_config_error"); return;}
    parser_.clear(); sample_ms_.reset(); x_ = y_ = yaw_ = 0.;
    state_.link_reset(steady_seconds(), "serial_connected");
    RCLCPP_INFO(get_logger(), "Connected to STM32 chassis port %s", port_.c_str());
  }
  bool write_frame(const std::vector<std::uint8_t> & bytes)
  {
    const double deadline = steady_seconds() + .05;
    std::size_t sent = 0;
    while (sent < bytes.size() && steady_seconds() < deadline) {
      const auto count = ::write(fd_, bytes.data() + sent, bytes.size() - sent);
      if (count > 0) {sent += static_cast<std::size_t>(count); continue;}
      if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {return false;}
      pollfd poller{fd_, POLLOUT, 0};
      ::poll(&poller, 1, 1);
    }
    return sent == bytes.size();
  }
  void read_serial()
  {
    if (fd_ < 0) {return;}
    std::array<std::uint8_t, 256> bytes;
    // Bound each callback even if a faulty peer floods the serial device.
    for (int batch = 0; batch < 16; ++batch) {
      const auto size = ::read(fd_, bytes.data(), bytes.size());
      if (size <= 0) {
        if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {reset_link("serial_read_error");}
        return;
      }
      for (const auto & s : parser_.feed(bytes.data(), static_cast<std::size_t>(size))) {
        state_.observe(s, steady_seconds()); publish_feedback(s);
      }
    }
  }
  void publish_feedback(const ChassisStatus & s)
  {
    if (!s.wheel_valid() || (sample_ms_ && *sample_ms_ == s.sample_ms)) {return;}
    const double vx = s.vx * .001, vy = s.vy * .001, wz = s.wz * .001;
    if (sample_ms_) {
      const auto delta = static_cast<std::uint32_t>(s.sample_ms - *sample_ms_);
      const double dt = delta * .001;
      if (delta > 0 && delta <= 0x80000000u && dt <= .5) {
        const double mid = yaw_ + .5 * wz * dt;
        x_ += (vx * std::cos(mid) - vy * std::sin(mid)) * dt;
        y_ += (vx * std::sin(mid) + vy * std::cos(mid)) * dt;
        yaw_ = std::atan2(std::sin(yaw_ + wz * dt), std::cos(yaw_ + wz * dt));
      }
    }
    sample_ms_ = s.sample_ms;
    last_pub_ns_ = std::max(now().nanoseconds(), last_pub_ns_ + 1);
    geometry_msgs::msg::TwistStamped velocity;
    velocity.header.stamp = rclcpp::Time(last_pub_ns_); velocity.header.frame_id = base_;
    velocity.twist.linear.x = vx; velocity.twist.linear.y = vy; velocity.twist.angular.z = wz;
    if (velocity_pub_) {velocity_pub_->publish(velocity);}
    if (odom_pub_) {
      nav_msgs::msg::Odometry odom; odom.header = velocity.header; odom.header.frame_id = wheel_frame_;
      odom.child_frame_id = base_; odom.pose.pose.position.x = x_; odom.pose.pose.position.y = y_;
      odom.pose.pose.orientation.z = std::sin(yaw_ / 2.); odom.pose.pose.orientation.w = std::cos(yaw_ / 2.);
      odom.twist.twist = velocity.twist; odom_pub_->publish(odom);
    }
  }
  void send()
  {
    open_serial();
    const double time = steady_seconds();
    if (fd_ >= 0) {
      const double age = (now() - rclcpp::Time(command_.header.stamp)).seconds();
      const bool valid = received_ && time - *received_ >= 0 && time - *received_ <= command_timeout_ &&
        (!fresh_stamp_ || (age >= 0 && age <= command_timeout_));
      const auto & t = command_.twist;
      const auto bytes = state_.next(time, t.linear.x, t.linear.y, t.angular.z, valid);
      tx_ = nlohmann::json{{"seq", state_.last_tx_seq}, {"mode", state_.last_tx_run ? 1 : 0},
        {"state", state_.running ? "RUNNING" : "WAIT_STOP_ACK"}, {"stop_reason", state_.reason}, {"invalid_reason", ""}};
      if (!write_frame(bytes)) {reset_link("serial_write_error");}
    }
    const auto & c = parser_.counters;
    nlohmann::json status{{"connected", fd_ >= 0}, {"port", port_},
      {"state", state_.running ? "RUNNING" : "WAIT_STOP_ACK"}, {"stop_reason", state_.reason},
      {"tx", tx_ ? *tx_ : nlohmann::json(nullptr)},
      {"rx_age_s", state_.status_time ? nlohmann::json(time - *state_.status_time) : nlohmann::json(nullptr)},
      {"chassis", nullptr}, {"parser", {{"frames_ok", c.frames_ok}, {"bad_header", c.bad_header},
        {"bad_version", c.bad_version}, {"bad_type", c.bad_type}, {"bad_length", c.bad_length}, {"bad_crc", c.bad_crc}}}};
    if (state_.status) {
      const auto & s = *state_.status;
      status["chassis"] = {{"seq", s.seq}, {"last_seq", s.last_seq}, {"status_raw", s.status},
        {"ready", bool(s.status & 1)}, {"enabled", bool(s.status & 2)}, {"estop", bool(s.status & 4)},
        {"fault", bool(s.status & 8)}, {"comm_timeout", bool(s.status & 16)},
        {"wheel_valid", s.wheel_valid()}, {"sample_time_ms", s.sample_ms}};
    }
    std_msgs::msg::String message; message.data = status.dump(); status_pub_->publish(message);
  }
  int fd_ = -1, baudrate_;
  std::string port_, expected_, base_, wheel_frame_;
  bool accept_empty_, fresh_stamp_;
  double command_timeout_, reconnect_, reconnect_at_ = 0., x_ = 0., y_ = 0., yaw_ = 0.;
  std::int64_t last_pub_ns_ = 0;
  std::optional<std::uint32_t> sample_ms_;
  std::optional<double> received_;
  std::optional<nlohmann::json> tx_;
  ChassisStateMachine state_;
  FrameStreamParser parser_;
  geometry_msgs::msg::TwistStamped command_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr reader_, sender_;
};
}  // namespace xxu_hardware
RCLCPP_COMPONENTS_REGISTER_NODE(xxu_hardware::ChassisLowerBridge)
