#include "rigid_mount_calibration_detail.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <iomanip>
#include <locale>
#include <regex>
#include <sstream>

namespace xxu_bringup {
using namespace calibration_detail;
namespace {

using OptionalVector = std::array<std::optional<double>, 3>;
constexpr auto yaml_tag = "tag:yaml.org,2002:";

OptionalVector components(const Eigen::Vector3d & vector)
{
  return {vector.x(), vector.y(), vector.z()};
}

YAML::Node number(double value)
{
  if (!std::isfinite(value)) {return YAML::Node(YAML::NodeType::Null);}
  std::ostringstream scalar;
  scalar.imbue(std::locale::classic());
  // Keep a decimal point even in scientific notation for Python's YAML loader.
  scalar << std::showpoint << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return YAML::Load(scalar.str());
}

YAML::Node number(const std::optional<double> & value)
{
  return number(value.value_or(unmeasured));
}

template<typename Vector>
YAML::Node vector_node(const Vector & values)
{
  YAML::Node node(YAML::NodeType::Sequence);
  node.SetStyle(YAML::EmitterStyle::Flow);
  for (const auto & value : values) {node.push_back(number(value));}
  return node;
}

YAML::Node pose_node(const OptionalVector & xyz, const OptionalVector & rpy)
{
  YAML::Node node;
  node["xyz"] = vector_node(xyz);
  node["rpy"] = vector_node(rpy);
  return node;
}

const std::regex integer_pattern(R"(^[-+]?(?:0b[01_]+|0[0-7_]+|0|[1-9][0-9_]*|0x[0-9a-fA-F_]+|[1-9][0-9_]*(?::[0-5]?[0-9])+)$)");
const std::regex float_pattern(R"(^(?:[-+]?[0-9][0-9_]*\.[0-9_]*(?:[eE][-+][0-9]+)?|\.[0-9][0-9_]*(?:[eE][-+][0-9]+)?|[-+]?[0-9][0-9_]*(?::[0-5]?[0-9])+\.[0-9_]*|[-+]?\.(?:inf|Inf|INF)|\.(?:nan|NaN|NAN))$)");
const std::regex bool_pattern(R"(^(?:yes|Yes|YES|no|No|NO|true|True|TRUE|false|False|FALSE|on|On|ON|off|Off|OFF)$)");
const std::regex date_pattern(
  R"(^[0-9]{4}-(?:[0-9]{2}-[0-9]{2}|[0-9]{1,2}-[0-9]{1,2}[Tt][0-9]{1,2}:[0-9]{2}:[0-9]{2}(?:\.[0-9]*)?(?:Z|[-+][0-9]{1,2}(?::[0-9]{2})?)?)$)");

YAML::Node text_node(const std::string & value)
{
  YAML::Node node(value);
  // Protect TF frame names that YAML would otherwise resolve as non-strings.
  if (std::regex_match(value, integer_pattern) || std::regex_match(value, float_pattern) ||
    std::regex_match(value, bool_pattern) || std::regex_match(value, date_pattern) ||
    value == "null" || value == "Null" || value == "NULL" || value == "~")
  {
    node.SetTag(std::string(yaml_tag) + "str");
  }
  return node;
}

std::string emit(const YAML::Node & node)
{
  YAML::Emitter emitter;
  emitter.SetNullFormat(YAML::LowerNull);
  emitter << node;
  if (!emitter.good()) {throw std::runtime_error(emitter.GetLastError());}
  return std::string(emitter.c_str()) + "\n";
}

std::array<std::string, 4> topic_names(const CalibrationConfig & c)
{
  return {c.lidar_topic, c.lidar_topic + "_compensated", c.imu_topic,
    c.imu_topic + "_lio_compensated"};
}

YAML::Node mount_node(
  const CalibrationConfig & c, const OptionalVector & xyz, const OptionalVector & base_rpy,
  const OptionalVector & lidar_xyz, const OptionalVector & lidar_rpy,
  const OptionalVector & imu_rpy, const std::optional<double> & scale, bool confirmed)
{
  YAML::Node node;
  node["version"] = 1;
  node["mounting"] = "rigid";
  node["calibration_confirmed"] = confirmed;
  node["frames"]["base"] = text_node(c.base_frame);
  node["frames"]["sensor"] = text_node(c.sensor_frame);
  node["frames"]["lidar"] = text_node(c.lidar_frame);
  node["frames"]["imu_source"] = text_node(c.imu_source_frame);
  node["base_to_sensor"] = pose_node(xyz, base_rpy);
  node["sensor_to_lidar"] = pose_node(lidar_xyz, lidar_rpy);
  node["sensor_from_imu_rpy"] = vector_node(imu_rpy);
  node["imu_acceleration_scale"] = number(scale);
  const auto topics = topic_names(c);
  const std::array<const char *, 4> keys{"points_in", "points_out", "imu_in", "imu_out"};
  for (std::size_t i = 0; i < keys.size(); ++i) {node["topics"][keys[i]] = topics[i];}
  return node;
}

bool tagged(const YAML::Node & node, const char * tag)
{
  return node.Tag() == std::string(yaml_tag) + tag;
}

std::string read_text(const YAML::Node & node)
{
  if (node.IsScalar()) {
    const auto value = node.Scalar();
    if (node.Tag() == "!" || tagged(node, "str")) {return value;}
    if (node.Tag() == "?" && !std::regex_match(value, integer_pattern) &&
      !std::regex_match(value, float_pattern) && !std::regex_match(value, bool_pattern) &&
      !std::regex_match(value, date_pattern))
    {
      return value;
    }
  }
  throw std::invalid_argument("mount frame/topic must be a YAML string");
}

double read_number(const YAML::Node & node)
{
  if (!node.IsScalar()) {throw std::invalid_argument("mount value must be a finite YAML number");}
  auto value = node.Scalar();
  const bool integer = tagged(node, "int") ||
    (node.Tag() == "?" && std::regex_match(value, integer_pattern));
  if (!integer && !(tagged(node, "float") ||
    (node.Tag() == "?" && std::regex_match(value, float_pattern))))
  {
    throw std::invalid_argument("mount value must be a finite YAML number");
  }
  value.erase(std::remove(value.begin(), value.end(), '_'), value.end());
  const double sign = !value.empty() && value.front() == '-' ? -1.0 : 1.0;
  if (!value.empty() && (value.front() == '-' || value.front() == '+')) {value.erase(0, 1);}
  double result = 0.0;
  const auto parse = [](const std::string & part) {
      std::istringstream stream(part);
      stream.imbue(std::locale::classic());
      double scalar;
      if (!(stream >> scalar) || stream.peek() != std::char_traits<char>::eof()) {
        throw std::invalid_argument("mount value must be a finite YAML number");
      }
      return scalar;
    };
  if (value.find(':') != std::string::npos) {
    std::istringstream parts(value);
    for (std::string part; std::getline(parts, part, ':');) {result = result * 60 + parse(part);}
  } else if (integer && value.size() > 1 && value.front() == '0') {
    const char prefix = value[1];
    const int base = prefix == 'x' ? 16 : prefix == 'b' ? 2 : 8;
    const auto digits = value.substr(base == 8 ? 1 : 2);
    if (digits.empty()) {throw std::invalid_argument("missing digits in mount value");}
    for (const char digit : digits) {
      const auto place = std::string("0123456789abcdef").find(
        static_cast<char>(std::tolower(static_cast<unsigned char>(digit))));
      if (place >= static_cast<std::size_t>(base)) {
        throw std::invalid_argument("invalid digits in mount value");
      }
      result = result * base + place;
    }
  } else {
    result = parse(value);
  }
  if (!std::isfinite(result)) {throw std::invalid_argument("nonfinite mount value");}
  return sign * result;
}

Eigen::Vector3d read_vector(const YAML::Node & node)
{
  if (!node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument("mount vector must contain three measured finite numbers");
  }
  return {read_number(node[0]), read_number(node[1]), read_number(node[2])};
}

Eigen::Isometry3d read_pose(const YAML::Node & node)
{
  if (!node.IsMap()) {throw std::invalid_argument("missing measured transform");}
  auto pose = Eigen::Isometry3d::Identity();
  pose.translation() = read_vector(node["xyz"]);
  pose.linear() = rpy_rotation(read_vector(node["rpy"]));
  return pose;
}

YAML::Node stage_node(const StageQuality & quality)
{
  YAML::Node node;
  node["attempted"] = quality.attempted;
  node["passed"] = quality.passed();
  node["reasons"] = quality.reasons;
  return node;
}

YAML::Node stationary_node(const StationaryResult & result)
{
  auto node = stage_node(result);
  node["imu_samples"] = result.imu_count;
  node["clouds"] = result.cloud_count;
  node["duration_sec"] = number(result.duration);
  node["max_imu_gap_sec"] = number(result.max_imu_gap);
  node["accel_mean_raw"] = vector_node(components(result.accel_mean));
  node["accel_mean_norm_raw"] = number(result.accel_norm);
  node["imu_acceleration_scale"] = number(result.scale);
  node["gyro_bias_rad_s"] = vector_node(components(result.gyro_bias));
  node["gyro_variance_rad2_s2"] = vector_node(components(result.gyro_variance));
  node["gyro_std_rad_s"] = vector_node(components(result.gyro_variance.cwiseSqrt()));
  node["accel_variance_raw"] = vector_node(components(result.accel_variance));
  node["accel_std_raw"] = vector_node(components(result.accel_variance.cwiseSqrt()));
  const double scale = result.scale.value_or(unmeasured);
  node["accel_variance_m2_s4"] = vector_node(components(result.accel_variance * scale * scale));
  node["accel_std_m_s2"] = vector_node(components(result.accel_variance.cwiseSqrt() * scale));
  node["accel_norm_std_m_s2"] = number(result.accel_norm_std);
  node["max_accel_deviation_m_s2"] = number(result.max_accel_deviation);
  node["gyro_norm_std_rad_s"] = number(result.gyro_norm_std);
  node["max_gyro_deviation_rad_s"] = number(result.max_gyro_deviation);
  node["level_from_imu_rpy_rad"] =
    vector_node(result.leveling ? components(rotation_to_rpy(*result.leveling)) : OptionalVector{});
  auto floor = node["floor"];
  floor["attempted"] = result.floor_attempted;
  floor["roi_points"] = result.roi_count;
  floor["inliers"] = result.inlier_count;
  floor["normal"] = vector_node(components(result.floor_normal));
  floor["offset_m"] = number(result.floor_offset);
  floor["rms_m"] = number(result.floor_rms);
  floor["gravity_ground_angle_deg"] = number(result.gravity_ground_angle_deg);
  floor["solved_tz_m"] = number(result.tz);
  return node;
}

YAML::Node straight_node(const StraightResult & result)
{
  auto node = stage_node(result);
  node["samples"] = result.sample_count;
  node["max_sample_gap_sec"] = number(result.max_sample_gap);
  node["psi_deg"] = number(result.psi ? degrees(*result.psi) : unmeasured);
  node["distance_m"] = number(result.distance);
  node["yaw_span_deg"] = number(result.yaw_span_deg);
  node["lateral_rms_m"] = number(result.lateral_rms);
  node["vertical_ratio"] = number(result.vertical_ratio);
  node["pca_axis_vs_endpoints_deg"] = number(result.pca_angle_deg);
  node["yaw_error_deg"] = number(result.yaw_error_deg);
  return node;
}

YAML::Node spin_node(const SpinResult & result)
{
  auto node = stage_node(result);
  node["samples"] = result.sample_count;
  node["max_sample_gap_sec"] = number(result.max_sample_gap);
  node["fitted_tx_ty_m"] = vector_node(std::array{result.tx, result.ty});
  node["center_xy_m"] = vector_node(std::array{result.center.x(), result.center.y()});
  node["yaw_span_deg"] = number(result.yaw_span_deg);
  node["roll_span_deg"] = number(result.roll_span_deg);
  node["pitch_span_deg"] = number(result.pitch_span_deg);
  node["rank"] = result.rank;
  node["condition"] = number(result.condition);
  auto covariance = node["covariance_cx_cy_tx_ty_m2"];
  for (Eigen::Index row = 0; row < result.covariance.rows(); ++row) {
    const std::array<double, 4> values{result.covariance(row, 0), result.covariance(row, 1),
      result.covariance(row, 2), result.covariance(row, 3)};
    covariance.push_back(vector_node(values));
  }
  node["std_tx_ty_m"] = vector_node(std::array{result.std_tx, result.std_ty});
  node["raw_radius_rms_m"] = number(result.raw_radius_rms);
  node["residual_rms_m"] = number(result.residual_rms);
  return node;
}

YAML::Node window_node(const std::optional<TimeWindow> & window)
{
  return window ? vector_node(std::array{window->start, window->end}) :
         YAML::Node(YAML::NodeType::Null);
}

template<typename Sample>
YAML::Node capture_interval(const std::vector<Sample> & samples)
{
  if (samples.empty()) {return YAML::Node(YAML::NodeType::Null);}
  return window_node(TimeWindow{samples.front().time, samples.back().time});
}

template<typename Result>
void report_stages(YAML::Node & node, const Result & result)
{
  node["stationary"] = stationary_node(result.stationary);
  node["straight"] = straight_node(result.straight);
  node["spin"] = spin_node(result.spin);
  node["reasons"] = result.reasons;
}

std::string render_body(
  const OptionalVector & angles, const OptionalVector & xyz, const std::string & frame,
  bool confirmed)
{
  YAML::Node node;
  auto body = node["common"]["body"];
  body["rotation"]["roll_deg"] = number(angles[0]);
  body["rotation"]["pitch_deg"] = number(angles[1]);
  body["rotation"]["yaw_deg"] = number(angles[2]);
  body["offset"]["x"] = number(xyz[0] ? -*xyz[0] : unmeasured);
  body["offset"]["y"] = number(xyz[1] ? -*xyz[1] : unmeasured);
  body["offset"]["z"] = number(xyz[2] ? -*xyz[2] : unmeasured);
  body["frame"] = text_node(frame);
  return std::string("# Merge only common.body into the deployment-specific backend YAML.\n") +
         "# offset = -p_base_imu in base axes. Use with a confirmed mount.\n" +
         (confirmed ? "" : "# Calibration unconfirmed.\n") + emit(node);
}

}  // namespace

RigidMount load_rigid_mount_cpp(const std::string & path)
{
  const auto node = YAML::LoadFile(path);
  if (!node.IsMap() || read_number(node["version"]) != 1.0 || read_text(node["mounting"]) != "rigid") {
    throw std::invalid_argument("mount requires version 1 and mounting: rigid");
  }
  const auto confirmed = node["calibration_confirmed"];
  if (!confirmed.IsScalar() || (confirmed.Tag() != "?" && !tagged(confirmed, "bool")) ||
    !std::regex_match(confirmed.Scalar(), std::regex("true|True|TRUE|yes|Yes|YES|on|On|ON")))
  {
    throw std::invalid_argument("mount must explicitly have calibration_confirmed: true");
  }
  RigidMount result;
  const auto frames = node["frames"];
  if (!frames.IsMap()) {throw std::invalid_argument("mount requires a frames mapping");}
  result.base_frame = read_text(frames["base"]);
  result.sensor_frame = read_text(frames["sensor"]);
  result.lidar_frame = read_text(frames["lidar"]);
  result.imu_source_frame = read_text(frames["imu_source"]);
  for (const auto & name : {result.base_frame, result.sensor_frame, result.lidar_frame,
    result.imu_source_frame})
  {
    validate_mount_frame(name);
  }
  if (std::set<std::string>{result.base_frame, result.sensor_frame, result.lidar_frame}.size() != 3) {
    throw std::invalid_argument("mount base, sensor and lidar frames must be distinct");
  }
  result.base_from_sensor = read_pose(node["base_to_sensor"]);
  result.sensor_from_lidar = read_pose(node["sensor_to_lidar"]);
  result.sensor_from_imu = rpy_rotation(read_vector(node["sensor_from_imu_rpy"]));
  result.imu_acceleration_scale = read_number(node["imu_acceleration_scale"]);
  if (result.imu_acceleration_scale <= 0) {throw std::invalid_argument("mount IMU scale must be positive");}
  const auto topics = node["topics"];
  if (!topics.IsMap()) {throw std::invalid_argument("mount requires a topics mapping");}
  const std::array<const char *, 4> keys{"points_in", "points_out", "imu_in", "imu_out"};
  for (std::size_t i = 0; i < keys.size(); ++i) {
    result.topics[i] = read_text(topics[keys[i]]);
    if (!valid_topic(result.topics[i])) {throw std::invalid_argument("invalid absolute mount topic");}
  }
  if (std::set<std::string>(result.topics.begin(), result.topics.end()).size() != keys.size()) {
    throw std::invalid_argument("mount topics must be distinct");
  }
  return result;
}

std::string render_rigid_mount_yaml(const CalibrationResult & result, const CalibrationConfig & c)
{
  const bool base_aligned = c.sensor_frame_mode == "base_aligned";
  const OptionalVector zero{0.0, 0.0, 0.0};
  const auto rpy = result.base_from_imu ?
    components(rotation_to_rpy(*result.base_from_imu)) : OptionalVector{};
  OptionalVector lidar_xyz, lidar_rpy;
  if (base_aligned && result.base_from_imu) {
    lidar_xyz = components(*result.base_from_imu * c.imu_from_lidar.translation());
    lidar_rpy = components(rotation_to_rpy(*result.base_from_imu * c.imu_from_lidar.linear()));
  } else if (!base_aligned) {
    lidar_xyz = components(c.imu_from_lidar.translation());
    lidar_rpy = components(rotation_to_rpy(c.imu_from_lidar.linear()));
  }
  return "# Transforms: parent <- child. XYZ in metres, fixed-axis RPY in radians.\n"
         "# Sensor origin coincides with the physical IMU origin. null means unmeasured.\n"
         "# sensor_from_imu_rpy rotates physical IMU measurements into the virtual sensor.\n" +
         emit(mount_node(c, {result.tx, result.ty, result.tz}, base_aligned ? zero : rpy,
           lidar_xyz, lidar_rpy, base_aligned ? rpy : zero, result.scale, result.confirmed));
}

std::string render_backend_body_yaml(const CalibrationResult & result, const CalibrationConfig & c)
{
  OptionalVector angles;
  if (c.sensor_frame_mode == "base_aligned") {
    angles = {0.0, 0.0, 0.0};
  } else if (result.base_from_imu) {
    angles = components(rotation_to_rpy(*result.base_from_imu) * (180.0 / calibration_pi));
  }
  return render_body(angles, {result.tx, result.ty, result.tz}, c.base_frame, result.confirmed);
}

namespace calibration_io {

std::string render_verified_mount(const RigidMount & mount, bool confirmed)
{
  CalibrationConfig c;
  c.base_frame = mount.base_frame;
  c.sensor_frame = mount.sensor_frame;
  c.lidar_frame = mount.lidar_frame;
  c.imu_source_frame = mount.imu_source_frame;
  auto node = mount_node(c, components(mount.base_from_sensor.translation()),
    components(rotation_to_rpy(mount.base_from_sensor.linear())),
    components(mount.sensor_from_lidar.translation()),
    components(rotation_to_rpy(mount.sensor_from_lidar.linear())),
    components(rotation_to_rpy(mount.sensor_from_imu)), mount.imu_acceleration_scale, confirmed);
  const std::array<const char *, 4> keys{"points_in", "points_out", "imu_in", "imu_out"};
  for (std::size_t i = 0; i < keys.size(); ++i) {node["topics"][keys[i]] = mount.topics[i];}
  return "# Transforms: parent <- child. XYZ in metres, fixed-axis RPY in radians.\n" + emit(node);
}

std::string render_verified_body(const RigidMount & mount, bool confirmed)
{
  return render_body(components(rotation_to_rpy(mount.base_from_sensor.linear()) *
      (180.0 / calibration_pi)), components(mount.base_from_sensor.translation()),
    mount.base_frame, confirmed);
}

}  // namespace calibration_io

std::string render_report_yaml(
  const CalibrationSamples & samples, const CalibrationConfig & c, const CalibrationResult & result,
  const std::optional<Segmentation> & segments, const std::optional<VerificationResult> & verification)
{
  YAML::Node node;
  node["operation"] = verification ? "verification" : "calibration";
  node["calibration_confirmed"] = result.confirmed;
  node["confirmation_requested"] = c.confirm;
  node["odom_topic"] = text_node(c.odom_topic);
  node["allow_unisolated_odom_topic"] = c.allow_unisolated_odom_topic;
  node["sensor_frame_mode"] = c.sensor_frame_mode;
  node["base_height_above_ground_m"] = number(c.base_height_above_ground);
  node["straight_direction"] = c.straight_direction;
  node["thresholds"]["imu_scale_requested"] = number(c.imu_acceleration_scale);
  for (const auto & [name, field] : real_thresholds) {node["thresholds"][name] = number(c.*field);}
  for (const auto & [name, field] : count_thresholds) {node["thresholds"][name] = c.*field;}
  node["imu_from_lidar"] = pose_node(components(c.imu_from_lidar.translation()),
    components(rotation_to_rpy(c.imu_from_lidar.linear())));
  node["manual"]["psi_deg"] = number(c.manual_psi_deg);
  node["manual"]["tx_ty_tz_m"] = vector_node(std::array{c.manual_tx, c.manual_ty, c.manual_tz});
  node["observed_frames"]["imu"] = std::vector<std::string>(samples.imu_frames.begin(), samples.imu_frames.end());
  node["observed_frames"]["lidar"] = std::vector<std::string>(samples.lidar_frames.begin(), samples.lidar_frames.end());
  node["observed_frames"]["odometry_parent"] = std::vector<std::string>(samples.odom_frames.begin(), samples.odom_frames.end());
  node["observed_frames"]["odometry_child"] = std::vector<std::string>(samples.odom_child_frames.begin(), samples.odom_child_frames.end());
  node["captured_messages"]["imu"] = samples.imu.size();
  node["captured_messages"]["lidar"] = samples.clouds.size();
  node["captured_messages"]["straight_odometry"] = samples.straight.size();
  node["captured_messages"]["spin_odometry"] = samples.spin.size();
  node["capture_intervals_header_sec"]["imu"] = capture_interval(samples.imu);
  node["capture_intervals_header_sec"]["lidar"] = capture_interval(samples.clouds);
  node["capture_intervals_header_sec"]["straight_odometry"] = capture_interval(samples.straight);
  node["capture_intervals_header_sec"]["spin_odometry"] = capture_interval(samples.spin);
  node["rejected_messages"]["imu"] = samples.rejected_imu;
  node["rejected_messages"]["lidar"] = samples.rejected_clouds;
  node["rejected_messages"]["odometry"] = samples.rejected_odom;
  node["input_errors"] = samples.input_errors;
  if (segments) {
    auto windows = node["windows_sec"];
    windows["time_origin"] = "first bag reception timestamp";
    windows["stationary"] = window_node(segments->stationary);
    windows["straight"] = window_node(segments->straight);
    windows["spin"] = window_node(segments->spin);
    windows["reasons"] = segments->reasons;
  }
  if (verification) {
    report_stages(node, *verification);
    node["verification_passed"] = verification->passed();
    node["imu_acceleration_scale"] = number(verification->stationary.scale);
    node["gravity_base_angle_deg"] = number(verification->gravity_angle_deg);
    node["gravity_vector_error_m_s2"] = number(verification->gravity_error);
    node["floor_height_error_m"] = number(verification->floor_height_error);
    return emit(node);
  }
  report_stages(node, result);
  node["imu_acceleration_scale"] = number(result.scale);
  node["base_from_imu"]["xyz_m"] = vector_node(std::array{result.tx, result.ty, result.tz});
  node["base_from_imu"]["rpy_rad"] = vector_node(result.base_from_imu ?
      components(rotation_to_rpy(*result.base_from_imu)) : OptionalVector{});
  node["psi_deg"] = number(result.psi ? degrees(*result.psi) : unmeasured);
  auto refinement = stage_node(result.refinement_quality);
  refinement["iterations"] = YAML::Node(YAML::NodeType::Sequence);
  for (const auto & iteration : result.refinement) {
    YAML::Node metrics;
    metrics["psi_change_rad"] = number(iteration[0]);
    metrics["lever_change_m"] = number(iteration[1]);
    metrics["straight_rms_m"] = number(iteration[2]);
    metrics["spin_rms_m"] = number(iteration[3]);
    refinement["iterations"].push_back(metrics);
  }
  node["refinement"] = refinement;
  return emit(node);
}

}  // namespace xxu_bringup
