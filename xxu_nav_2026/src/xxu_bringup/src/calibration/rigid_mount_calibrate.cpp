#include "rigid_mount_calibration_io.hpp"
#include "xxu_bringup/sensor_math.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>

#include <iostream>
#include <map>

namespace {
using namespace xxu_bringup;
using namespace xxu_bringup::calibration_detail;
using namespace xxu_bringup::calibration_io;

struct Options
{
  CalibrationConfig config;
  OutputFiles outputs;
  std::string bag, stationary_bag, straight_bag, spin_bag, verify_path;
  bool automatic = false, imu_topic_set = false, lidar_topic_set = false;
  Segmentation windows;
};

void help()
{
  std::cout << R"(Usage:
  rigid_mount_calibrate --bag DIR --auto-segment [OPTIONS]
  rigid_mount_calibrate --bag DIR --stationary-window S:E
      [--straight-window S:E] [--spin-window S:E] [OPTIONS]
  rigid_mount_calibrate --stationary-bag DIR [--straight-bag DIR] [--spin-bag DIR] [OPTIONS]

Input: MCAP or SQLite3 rosbag. Windows are inclusive seconds from the first
bag reception timestamp, for example 0:5. Motion odometry must be T_odom_imu
from raw-input ZLIO, with continuous poses during each maneuver.
  --verify-mount PATH             check a confirmed mount using these samples
  --lidar-topic TOPIC             /livox/lidar (PointCloud2 or Livox CustomMsg)
  --imu-topic TOPIC               /livox/imu
  --odom-topic TOPIC              /calibration/lidar_odometry
  --allow-unisolated-odom-topic   allow a shared production topic only after
                                  confirming it carries raw-input ZLIO poses
  --odom-frame NAME               odom (odometry header.frame_id)
  --odom-child-frame NAME         imu (physical IMU child_frame_id)
  --imu-scale SCALE               0=auto, or a positive multiplier to m/s^2
  --base-height-above-ground M    0 by default; use the measured base reference
  --imu-from-lidar-xyz X Y Z      metres, default -0.011 -0.02329 0.04412
  --imu-from-lidar-rpy R P Y      fixed-axis radians, default 0 0 0
  --straight-direction 1|-1      forward (default) or backward
  --sensor-frame-mode MODE       base_aligned (default) or imu_aligned
  --base-frame NAME              base_link
  --sensor-frame NAME            lio_base_sensor
  --lidar-frame NAME             livox_frame
  --imu-source-frame NAME        livox_frame
  --yaw-deg DEG                  manual psi: R_base_imu = Rz(psi) * R_level_imu
  --base-to-sensor-x M           manual translation along base X
  --base-to-sensor-y M           manual translation along base Y
  --base-to-sensor-z M           manual translation along base Z
  --output-mount PATH            rigid mount YAML, including unconfirmed results
  --output-backend-body PATH     common.body fragment, written only when confirmed
  --output-report PATH           metrics and quality gate results
  --confirm                      require a complete passing solution to confirm

Incomplete/failed --confirm or --verify-mount, invalid input and export errors
exit nonzero. Partial capture without --confirm can exit 0 and exports null
for unmeasured values. Verification reads its mount without modifying it.
Quality thresholds:
)";
  const CalibrationConfig defaults;
  const auto print = [](std::string name, const auto value) {
      std::replace(name.begin(), name.end(), '_', '-');
      std::cout << "  --" << name << " VALUE (default " << value << ")\n";
    };
  std::cout << "  --min-spin-yaw-deg VALUE (alias for --min-spin-yaw-span-deg)\n"
            << "  --max-spin-rms VALUE (alias for --max-spin-residual-rms)\n";
  for (const auto & [name, field] : real_thresholds) {print(name, defaults.*field);}
  for (const auto & [name, field] : count_thresholds) {print(name, defaults.*field);}
}

TimeWindow window(const std::string & argument)
{
  const auto split = argument.find(':');
  if (split == std::string::npos) {throw std::invalid_argument("window must be START:END seconds");}
  const double begin = parse_number(argument.substr(0, split), "window start");
  const double end = parse_number(argument.substr(split + 1), "window end");
  if (begin < 0 || begin >= end) {throw std::invalid_argument("window requires 0 <= START < END");}
  return {begin, end};
}

void parse(Options & o, int argc, char ** argv)
{
  auto & c = o.config;
  const std::map<std::string, std::string *> strings{
    {"--bag", &o.bag}, {"--stationary-bag", &o.stationary_bag}, {"--straight-bag", &o.straight_bag},
    {"--spin-bag", &o.spin_bag}, {"--verify-mount", &o.verify_path},
    {"--imu-topic", &c.imu_topic}, {"--lidar-topic", &c.lidar_topic}, {"--odom-topic", &c.odom_topic},
    {"--odom-frame", &c.odom_frame}, {"--odom-child-frame", &c.odom_child_frame},
    {"--sensor-frame-mode", &c.sensor_frame_mode}, {"--base-frame", &c.base_frame},
    {"--sensor-frame", &c.sensor_frame}, {"--lidar-frame", &c.lidar_frame},
    {"--imu-source-frame", &c.imu_source_frame}, {"--output-mount", &o.outputs.mount},
    {"--output-backend-body", &o.outputs.backend}, {"--output-report", &o.outputs.report}};
  const std::map<std::string, std::optional<double> *> manual{
    {"--yaw-deg", &c.manual_psi_deg}, {"--base-to-sensor-x", &c.manual_tx},
    {"--base-to-sensor-y", &c.manual_ty}, {"--base-to-sensor-z", &c.manual_tz}};
  const std::map<std::string, std::optional<TimeWindow> *> windows{
    {"--stationary-window", &o.windows.stationary}, {"--straight-window", &o.windows.straight},
    {"--spin-window", &o.windows.spin}};
  std::map<std::string, double *> reals{
    {"--imu-scale", &c.imu_acceleration_scale}, {"--base-height-above-ground", &c.base_height_above_ground},
    {"--min-spin-yaw-deg", &c.min_spin_yaw_span_deg}, {"--max-spin-rms", &c.max_spin_residual_rms}};
  std::map<std::string, int *> counts{{"--straight-direction", &c.straight_direction}};
  const auto flag = [](std::string name) {
      std::replace(name.begin(), name.end(), '_', '-');
      return "--" + name;
    };
  for (const auto & [name, field] : real_thresholds) {reals.emplace(flag(name), &(c.*field));}
  for (const auto & [name, field] : count_thresholds) {counts.emplace(flag(name), &(c.*field));}
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    const auto next = [&]() -> std::string {
        if (++i == argc) {throw std::invalid_argument("missing value for " + option);}
        return argv[i];
      };
    if (const auto entry = strings.find(option); entry != strings.end()) {
      *entry->second = next();
      if (option == "--imu-topic") {o.imu_topic_set = true;}
      if (option == "--lidar-topic") {o.lidar_topic_set = true;}
    } else if (const auto entry = manual.find(option); entry != manual.end()) {
      *entry->second = parse_number(next(), option);
    } else if (const auto entry = windows.find(option); entry != windows.end()) {
      *entry->second = window(next());
    } else if (const auto entry = reals.find(option); entry != reals.end()) {
      *entry->second = parse_number(next(), option);
    } else if (const auto entry = counts.find(option); entry != counts.end()) {
      *entry->second = parse_count(next(), option);
    } else if (option == "--imu-from-lidar-xyz" || option == "--imu-from-lidar-rpy") {
      Eigen::Vector3d xyz;
      for (double & value : xyz) {value = parse_number(next(), option);}
      if (option == "--imu-from-lidar-xyz") {c.imu_from_lidar.translation() = xyz;}
      else {c.imu_from_lidar.linear() = rpy_rotation(xyz);}
    } else if (option == "--confirm") {
      c.confirm = true;
    } else if (option == "--allow-unisolated-odom-topic") {
      c.allow_unisolated_odom_topic = true;
    } else if (option == "--auto-segment") {
      o.automatic = true;
    } else {
      throw std::invalid_argument("unknown option: " + option + " (use --help)");
    }
  }
}

bool inside(const std::filesystem::path & path, const std::filesystem::path & root)
{
  const auto relative = path.lexically_relative(root);
  return !relative.empty() && *relative.begin() != "..";
}

void validate_inputs(const Options & o)
{
  const bool explicit_windows = o.windows.stationary || o.windows.straight || o.windows.spin;
  if (!o.bag.empty()) {
    if (!o.stationary_bag.empty() || !o.straight_bag.empty() || !o.spin_bag.empty() ||
      (o.automatic && explicit_windows) || (!o.automatic && !o.windows.stationary))
    {
      throw std::invalid_argument("use --bag with --auto-segment or explicit windows including stationary");
    }
  } else if (o.stationary_bag.empty() || explicit_windows || o.automatic) {
    throw std::invalid_argument("provide --bag or --stationary-bag (use --help)");
  }
}

void protect_inputs(const Options & o)
{
  for (const auto & name : output_paths(o.outputs)) {
    if (name.empty()) {continue;}
    const auto path = std::filesystem::weakly_canonical(std::filesystem::absolute(name));
    for (const auto & source : {o.bag, o.stationary_bag, o.straight_bag, o.spin_bag, o.verify_path}) {
      if (!source.empty() && inside(path,
        std::filesystem::weakly_canonical(std::filesystem::absolute(source))))
      {
        throw std::invalid_argument("output path must be outside input bags and distinct from input mount");
      }
    }
  }
}

enum class Clouds {pointcloud2, livox};
constexpr char pointcloud_type[] = "sensor_msgs/msg/PointCloud2";
constexpr char livox_type[] = "livox_ros_driver2/msg/CustomMsg";

class Bag
{
public:
  Bag(const std::string & path, const CalibrationConfig & config, bool sensors, bool odometry)
  {
    reader.open(path);
    // Fastwrite MCAP without an index only supports file order. Scan its
    // remaining records instead of assuming reception timestamps are sorted.
    const bool per_file_ordered = reader.set_read_order(rosbag2_storage::ReadOrder{});
    const auto & metadata = reader.get_metadata();
    // set_read_order sorts within each storage file, not across split files.
    ordered = per_file_ordered && metadata.files.size() <= 1 &&
      metadata.relative_file_paths.size() <= 1;
    auto starting_time = metadata.starting_time;
    for (const auto & file : metadata.files) {
      if (file.message_count > 0) {
        starting_time = std::min(starting_time, file.starting_time);
      }
    }
    start = std::max<std::int64_t>(0, std::chrono::duration_cast<std::chrono::nanoseconds>(
      starting_time.time_since_epoch()).count());
    std::map<std::string, std::string> topics;
    for (const auto & info : reader.get_all_topics_and_types()) {
      if (topics.count(info.name) && topics.at(info.name) != info.type) {
        throw std::invalid_argument("multiple types recorded for topic " + info.name);
      }
      topics[info.name] = info.type;
    }
    const auto type = [&](const std::string & topic) {
        const auto it = topics.find(topic);
        if (it == topics.end()) {throw std::invalid_argument("bag missing topic " + topic);}
        return it->second;
      };
    rosbag2_storage::StorageFilter filter;
    if (sensors) {
      const auto lidar = type(config.lidar_topic);
      if (lidar == livox_type) {
#ifndef XXU_HAVE_LIVOX_CUSTOM_MSG
        throw std::invalid_argument("bag contains Livox CustomMsg; rebuild with livox_ros_driver2 available");
#else
        clouds = Clouds::livox;
#endif
      } else if (lidar != pointcloud_type) {
        throw std::invalid_argument("unsupported LiDAR topic type: " + lidar);
      }
      filter.topics.push_back(config.lidar_topic);
      if (type(config.imu_topic) != "sensor_msgs/msg/Imu") {
        throw std::invalid_argument("wrong type for IMU topic " + config.imu_topic);
      }
      filter.topics.push_back(config.imu_topic);
    }
    if (odometry) {
      if (type(config.odom_topic) != "nav_msgs/msg/Odometry") {
        throw std::invalid_argument("wrong type for odometry topic " + config.odom_topic);
      }
      filter.topics.push_back(config.odom_topic);
    }
    reader.set_filter(filter);
  }

  double relative_time(const rosbag2_storage::SerializedBagMessage & message) const
  {
    if (message.recv_timestamp < 0) {return unmeasured;}
    if (message.recv_timestamp < start) {
      // Keep early file-order records at the start of the capture timeline.
      return ordered ? unmeasured : 0.0;
    }
    return (message.recv_timestamp - start) * 1.e-9;
  }

  rosbag2_cpp::Reader reader;
  Clouds clouds = Clouds::pointcloud2;
  bool ordered = false;
private:
  std::int64_t start;
};

template<typename Message>
void deserialize(const rosbag2_storage::SerializedBagMessage & bag_message, Message & message)
{
  rclcpp::SerializedMessage bytes(*bag_message.serialized_data);
  rclcpp::Serialization<Message> serialization;
  serialization.deserialize_message(&bytes, &message);
}

template<typename Message>
Message deserialize(const rosbag2_storage::SerializedBagMessage & bag_message)
{
  Message message;
  deserialize(bag_message, message);
  return message;
}

template<typename Cloud>
void record_cloud(
  const rosbag2_storage::SerializedBagMessage & input, const TimeWindow * acquisition_window,
  CalibrationSamples & data)
{
  const auto cloud = deserialize<Cloud>(input);
  const double stamp = stamp_time(cloud.header.stamp);
  if (acquisition_window && valid_time(stamp) && !acquisition_window->contains(stamp)) {return;}
  observe(data.lidar_frames, cloud.header.frame_id);
  if (valid_time(stamp)) {
    if (data.clouds.size() == max_clouds) {
      input_error(data, "LiDAR capture limit exceeded; select a shorter stationary window");
    } else {
      auto points = extract_calibration_points(cloud, max_points_per_cloud);
      if (!points.empty()) {
        data.clouds.push_back({stamp, std::move(points)});
        return;
      }
    }
  }
  ++data.rejected_clouds;
}

void record_cloud(
  const Bag & bag, const rosbag2_storage::SerializedBagMessage & message,
  const TimeWindow * acquisition_window, CalibrationSamples & data)
{
  try {
#ifdef XXU_HAVE_LIVOX_CUSTOM_MSG
    if (bag.clouds == Clouds::livox) {
      record_cloud<livox_ros_driver2::msg::CustomMsg>(message, acquisition_window, data);
      return;
    }
#endif
    record_cloud<sensor_msgs::msg::PointCloud2>(message, acquisition_window, data);
  } catch (const rclcpp::exceptions::RCLErrorBase &) {
    ++data.rejected_clouds;
  } catch (const std::invalid_argument &) {
    ++data.rejected_clouds;
  }
}

void require_messages(bool available, const std::string & label, CalibrationSamples & data)
{
  if (!available) {input_error(data, "zero valid messages for " + label);}
}

template<typename Sample>
struct ReceivedSample
{
  double reception;
  std::int64_t header_ns;
  Sample sample;
};

struct AcquisitionWindows
{
  std::optional<TimeWindow> imu, straight, spin, clouds;
};

struct CloudStamp
{
  double time;
};

std::int64_t header_nanoseconds(const builtin_interfaces::msg::Time & time)
{
  return std::int64_t{time.sec} * 1000000000 + time.nanosec;
}

template<typename Cloud>
std::optional<builtin_interfaces::msg::Time> read_cloud_stamp(
  const rosbag2_storage::SerializedBagMessage & message, Cloud & scratch)
{
  try {
    deserialize(message, scratch);
  } catch (const rclcpp::exceptions::RCLErrorBase &) {
    return std::nullopt;
  }
  const auto stamp = scratch.header.stamp;
  if (!valid_time(stamp_time(stamp))) {return std::nullopt;}
  return stamp;
}

template<typename Sample>
void sort_headers(std::vector<ReceivedSample<Sample>> & records)
{
  std::stable_sort(records.begin(), records.end(), [](const auto & a, const auto & b) {
      return a.header_ns == b.header_ns ? a.reception < b.reception : a.header_ns < b.header_ns;
    });
  records.erase(std::unique(records.begin(), records.end(),
    [](const auto & a, const auto & b) {return a.header_ns == b.header_ns;}), records.end());
}

template<typename Sample>
std::vector<Sample> segmentation_samples(
  const std::vector<ReceivedSample<Sample>> & records, std::int64_t header_origin, double shift)
{
  std::vector<Sample> samples;
  samples.reserve(records.size());
  for (const auto & record : records) {
    samples.push_back(record.sample);
    samples.back().time = (record.header_ns - header_origin) * 1.e-9 + shift;
  }
  return samples;
}

void include_time(std::optional<TimeWindow> & window, double time)
{
  if (window) {
    window->start = std::min(window->start, time);
    window->end = std::max(window->end, time);
  } else {
    window = TimeWindow{time, time};
  }
}

template<typename Sample>
void map_window(
  const std::optional<TimeWindow> & window, const std::vector<Sample> & aligned,
  const std::vector<ReceivedSample<Sample>> & records,
  std::optional<TimeWindow> & headers, std::optional<TimeWindow> & reception)
{
  if (!window) {return;}
  for (std::size_t i = 0; i < records.size(); ++i) {
    if (window->contains(aligned[i].time)) {
      include_time(headers, records[i].sample.time);
      include_time(reception, records[i].reception);
    }
  }
}

Segmentation find_windows(
  const std::string & path, const CalibrationConfig & c, CalibrationSamples & data,
  AcquisitionWindows & acquisition)
{
  Bag bag(path, c, true, true);
  std::vector<ReceivedSample<ImuSample>> imu_records;
  std::vector<ReceivedSample<OdomSample>> odom_records;
  std::vector<ReceivedSample<CloudStamp>> cloud_records;
  sensor_msgs::msg::PointCloud2 pointcloud;
#ifdef XXU_HAVE_LIVOX_CUSTOM_MSG
  livox_ros_driver2::msg::CustomMsg livox_cloud;
#endif
  while (bag.reader.has_next()) {
    const auto message = bag.reader.read_next();
    const double time = bag.relative_time(*message);
    if (!valid_time(time)) {continue;}
    if (message->topic_name == c.imu_topic) {
      const auto imu = deserialize<sensor_msgs::msg::Imu>(*message);
      auto sample = imu_sample(imu);
      if (valid_imu(sample)) {
        imu_records.push_back({time, header_nanoseconds(imu.header.stamp), std::move(sample)});
      }
    } else if (message->topic_name == c.odom_topic) {
      const auto odom = deserialize<nav_msgs::msg::Odometry>(*message);
      auto sample = odom_sample(odom);
      if (valid_odom(sample)) {
        odom_records.push_back({time, header_nanoseconds(odom.header.stamp), std::move(sample)});
      }
    } else {
      // Only the selected stationary window needs LiDAR points extracted.
      const auto stamp =
#ifdef XXU_HAVE_LIVOX_CUSTOM_MSG
        bag.clouds == Clouds::livox ? read_cloud_stamp(*message, livox_cloud) :
#endif
        read_cloud_stamp(*message, pointcloud);
      if (stamp) {
        cloud_records.push_back({time, header_nanoseconds(*stamp), CloudStamp{stamp_time(*stamp)}});
      }
    }
  }
  require_messages(!imu_records.empty(), c.imu_topic, data);
  require_messages(!odom_records.empty(), c.odom_topic, data);
  require_messages(!cloud_records.empty(), c.lidar_topic, data);
  sort_headers(imu_records);
  sort_headers(odom_records);
  sort_headers(cloud_records);
  auto header_origin = imu_records.empty() ? std::numeric_limits<std::int64_t>::max() :
    imu_records.front().header_ns;
  if (!odom_records.empty()) {header_origin = std::min(header_origin, odom_records.front().header_ns);}
  if (!cloud_records.empty()) {header_origin = std::min(header_origin, cloud_records.front().header_ns);}
  const auto imu = segmentation_samples(imu_records, header_origin, 0.0);
  const auto odom = segmentation_samples(odom_records, header_origin, 0.0);
  const auto aligned_cloud_samples = segmentation_samples(cloud_records, header_origin, 0.0);
  std::vector<double> aligned_clouds;
  aligned_clouds.reserve(aligned_cloud_samples.size());
  for (const auto & sample : aligned_cloud_samples) {aligned_clouds.push_back(sample.time);}

  auto found = auto_segment_recording(imu, odom, aligned_clouds, c);
  Segmentation reception;
  reception.reasons = std::move(found.reasons);
  map_window(found.stationary, imu, imu_records, acquisition.imu, reception.stationary);
  if (found.stationary && acquisition.imu) {
    for (std::size_t i = 0; i < cloud_records.size(); ++i) {
      const auto & record = cloud_records[i];
      if (found.stationary->contains(aligned_cloud_samples[i].time) &&
        acquisition.imu->contains(record.sample.time))
      {
        include_time(acquisition.clouds, record.sample.time);
        include_time(reception.stationary, record.reception);
      }
    }
  }
  map_window(found.straight, odom, odom_records, acquisition.straight, reception.straight);
  map_window(found.spin, odom, odom_records, acquisition.spin, reception.spin);
  return reception;
}

template<typename Sample>
std::size_t sort_captured_samples(std::vector<Sample> & samples, std::size_t previous_count)
{
  const auto first = samples.begin() + previous_count;
  std::stable_sort(first, samples.end(),
    [](const Sample & a, const Sample & b) {return a.time < b.time;});
  const auto new_first = previous_count ?
    std::upper_bound(first, samples.end(), samples[previous_count - 1].time,
      [](double time, const Sample & sample) {return time < sample.time;}) : first;
  const auto end = std::unique(new_first, samples.end(),
    [](const Sample & a, const Sample & b) {return a.time == b.time;});
  const std::size_t duplicates = (new_first - first) + (samples.end() - end);
  samples.erase(end, samples.end());
  samples.erase(first, new_first);
  return duplicates;
}

void collect(
  const std::string & path, const CalibrationConfig & c, const Segmentation & selected,
  CalibrationSamples & data, const AcquisitionWindows * acquisition = nullptr)
{
  const bool stationary = selected.stationary.has_value();
  const bool motion = selected.straight || selected.spin;
  if (!stationary && !motion) {return;}
  const auto counts = std::array{data.imu.size(), data.clouds.size(), data.straight.size(), data.spin.size()};
  Bag bag(path, c, stationary, motion);
  double end = 0.0;
  for (const auto & span : {selected.stationary, selected.straight, selected.spin}) {
    if (span) {end = std::max(end, span->end);}
  }
  while (bag.reader.has_next()) {
    const auto message = bag.reader.read_next();
    const auto & name = message->topic_name;
    const double time = bag.relative_time(*message);
    if (!valid_time(time)) {continue;}
    if (time > end) {
      if (bag.ordered) {break;}
      continue;
    }
    if (selected.stationary && selected.stationary->contains(time)) {
      if (name == c.imu_topic) {
        const auto imu = deserialize<sensor_msgs::msg::Imu>(*message);
        const auto sample = imu_sample(imu);
        if (acquisition && valid_time(sample.time) &&
          !(acquisition->imu && acquisition->imu->contains(sample.time)))
        {
          continue;
        }
        observe(data.imu_frames, imu.header.frame_id);
        if (!valid_imu(sample) ||
          !append_sample(data, data.imu, sample, max_capture_samples, "IMU", false))
        {
          ++data.rejected_imu;
        }
      } else if (name == c.lidar_topic && (!acquisition || acquisition->clouds)) {
        const TimeWindow * acquisition_window =
          acquisition && acquisition->clouds ? &*acquisition->clouds : nullptr;
        record_cloud(bag, *message, acquisition_window, data);
      }
    }
    if (name != c.odom_topic) {continue;}
    bool straight = selected.straight && selected.straight->contains(time);
    bool spin = selected.spin && selected.spin->contains(time);
    if (!straight && !spin) {continue;}
    const auto odom = deserialize<nav_msgs::msg::Odometry>(*message);
    const auto sample = odom_sample(odom);
    if (acquisition && valid_time(sample.time)) {
      straight = straight && acquisition->straight && acquisition->straight->contains(sample.time);
      spin = spin && acquisition->spin && acquisition->spin->contains(sample.time);
      if (!straight && !spin) {continue;}
    }
    observe(data.odom_frames, odom.header.frame_id);
    observe(data.odom_child_frames, odom.child_frame_id);
    if (!valid_odom(sample)) {
      ++data.rejected_odom;
      continue;
    }
    if (straight &&
      !append_sample(data, data.straight, sample, max_capture_samples, "straight", false))
    {
      ++data.rejected_odom;
    }
    if (spin && !append_sample(data, data.spin, sample, max_capture_samples, "spin", false)) {
      ++data.rejected_odom;
    }
  }
  // Even indexed bags may deliver header stamps out of order in reception order.
  data.rejected_imu += sort_captured_samples(data.imu, counts[0]);
  data.rejected_clouds += sort_captured_samples(data.clouds, counts[1]);
  data.rejected_odom += sort_captured_samples(data.straight, counts[2]);
  data.rejected_odom += sort_captured_samples(data.spin, counts[3]);
  if (stationary) {
    require_messages(data.imu.size() > counts[0], c.imu_topic + " in stationary window", data);
    require_messages(data.clouds.size() > counts[1], c.lidar_topic + " in stationary window", data);
  }
  if (selected.straight) {
    require_messages(data.straight.size() > counts[2], c.odom_topic + " in straight window", data);
  }
  if (selected.spin) {
    require_messages(data.spin.size() > counts[3], c.odom_topic + " in spin window", data);
  }
}

void load_samples(const Options & options, CalibrationSamples & data, std::optional<Segmentation> & segments)
{
  if (!options.bag.empty()) {
    data.straight_requested = options.automatic || options.windows.straight.has_value();
    data.spin_requested = options.automatic || options.windows.spin.has_value();
    AcquisitionWindows acquisition;
    segments = options.automatic ?
      find_windows(options.bag, options.config, data, acquisition) : options.windows;
    collect(options.bag, options.config, *segments, data, options.automatic ? &acquisition : nullptr);
  } else {
    data.straight_requested = !options.straight_bag.empty();
    data.spin_requested = !options.spin_bag.empty();
    const TimeWindow all{0.0, std::numeric_limits<double>::infinity()};
    const auto capture = [&](const std::string & bag, const Segmentation & chosen) {
        if (bag.empty()) {return;}
        try {
          collect(bag, options.config, chosen, data);
        } catch (const std::exception & e) {
          input_error(data, bag + ": " + e.what());
        }
      };
    capture(options.stationary_bag, {all, {}, {}, {}});
    capture(options.straight_bag, {{}, all, {}, {}});
    capture(options.spin_bag, {{}, {}, all, {}});
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 1 || (argc == 2 && std::string(argv[1]) == "--help")) {
    help();
    return argc == 1 ? 1 : 0;
  }
  Options options;
  CalibrationSamples data;
  CalibrationResult result;
  std::optional<Segmentation> segments;
  std::optional<VerificationResult> verification;
  std::optional<RigidMount> mount;
  try {
    parse(options, argc, argv);
    validate_output_paths(options.outputs);
    protect_inputs(options);
  } catch (const std::exception & e) {
    std::cerr << "rigid_mount_calibrate: " << e.what() << '\n';
    return 1;
  }

  bool failed = false;
  try {
    if (!options.verify_path.empty()) {verification.emplace();}
    validate_inputs(options);
    if (verification) {
      mount = load_rigid_mount_cpp(options.verify_path);
      options.config.imu_acceleration_scale = mount->imu_acceleration_scale;
      if (!options.imu_topic_set) {options.config.imu_topic = mount->topics[2];}
      if (!options.lidar_topic_set) {options.config.lidar_topic = mount->topics[0];}
      options.config.sensor_frame_mode = mount->base_from_sensor.linear().isIdentity(1.e-10) ?
        "base_aligned" : "imu_aligned";
      options.config.imu_from_lidar = mount->sensor_from_lidar;
      options.config.imu_from_lidar.linear() =
        mount->sensor_from_imu.transpose() * mount->sensor_from_lidar.linear();
      options.config.imu_from_lidar.translation() =
        mount->sensor_from_imu.transpose() * mount->sensor_from_lidar.translation();
    }
    validate_calibration_config(options.config);
    try {
      load_samples(options, data, segments);
    } catch (const std::exception & e) {
      input_error(data, e.what());
    }
    if (mount) {
      verification = verify_calibration(*mount, data, options.config);
      result.confirmed = options.config.confirm && verification->passed();
      failed = !verification->passed();
    } else {
      result = solve_all(data, options.config);
      failed = !data.input_errors.empty() || (options.config.confirm && !result.confirmed);
    }
  } catch (const std::exception & e) {
    failed = true;
    result.reasons.push_back(e.what());
    if (verification) {verification->reasons.push_back(e.what());}
  }
  if (segments && !segments->reasons.empty()) {
    std::cerr << summarize(segments->reasons) << '\n';
  }
  const auto & reasons = verification ? verification->reasons : result.reasons;
  if (!reasons.empty()) {std::cerr << summarize(reasons) << '\n';}
  const auto errors = write_outputs(options.outputs, data, options.config, result,
    segments, verification, mount);
  const bool final_confirmed = result.confirmed && errors.empty();
  if (mount) {
    std::cout << (failed || !errors.empty() ? "Verification failed" : "Verification passed") << '\n';
  } else if (!verification) {
    std::cout << (final_confirmed ? "Calibration confirmed" : "Calibration unconfirmed") << '\n';
  }
  if (!errors.empty()) {std::cerr << summarize(errors) << '\n';}
  return failed || !errors.empty() ? 1 : 0;
}
