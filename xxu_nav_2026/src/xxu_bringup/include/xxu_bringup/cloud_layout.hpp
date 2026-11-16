#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace xxu_bringup {
template<typename T> T read_scalar(const std::uint8_t * address, bool bigendian)
{
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), address, bytes.size());
  const std::uint16_t one = 1;
  const bool host_big = *reinterpret_cast<const std::uint8_t *>(&one) == 0;
  if (host_big != bigendian) {std::reverse(bytes.begin(), bytes.end());}
  T result;
  std::memcpy(&result, bytes.data(), sizeof(T));
  return result;
}
template<typename T> void write_scalar(std::uint8_t * address, T value, bool bigendian)
{
  std::array<std::uint8_t, sizeof(T)> bytes;
  std::memcpy(bytes.data(), &value, sizeof(T));
  const std::uint16_t one = 1;
  const bool host_big = *reinterpret_cast<const std::uint8_t *>(&one) == 0;
  if (host_big != bigendian) {std::reverse(bytes.begin(), bytes.end());}
  std::memcpy(address, bytes.data(), sizeof(T));
}
struct CloudLayout
{
  std::array<std::uint32_t, 3> xyz;
  std::uint32_t timestamp = 0;
  explicit CloudLayout(const sensor_msgs::msg::PointCloud2 & cloud, bool need_time = false)
  {
    using Field = sensor_msgs::msg::PointField;
    if (cloud.point_step == 0 || static_cast<std::uint64_t>(cloud.row_step) <
      static_cast<std::uint64_t>(cloud.width) * cloud.point_step ||
      static_cast<std::uint64_t>(cloud.row_step) * cloud.height != cloud.data.size())
    {throw std::invalid_argument("invalid cloud layout");}
    auto field = [&](const std::string & name, int datatype, std::uint32_t size) {
        auto found = cloud.fields.end();
        for (auto it = cloud.fields.begin(); it != cloud.fields.end(); ++it) {
          if (it->name != name) {continue;}
          if (found != cloud.fields.end()) {throw std::invalid_argument("duplicate point field");}
          found = it;
        }
        if (found == cloud.fields.end() || found->datatype != datatype || found->count != 1 ||
          found->offset > cloud.point_step || size > cloud.point_step - found->offset)
        {throw std::invalid_argument("invalid " + name + " point field");}
        return found->offset;
      };
    xyz = {field("x", Field::FLOAT32, 4), field("y", Field::FLOAT32, 4), field("z", Field::FLOAT32, 4)};
    if (need_time) {timestamp = field("timestamp", Field::FLOAT64, 8);}
  }
};
}  // namespace xxu_bringup
