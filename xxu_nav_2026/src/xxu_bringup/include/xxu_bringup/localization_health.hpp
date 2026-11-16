#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace xxu_bringup {

inline std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) {return -1;}
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

inline bool fresh(const std::optional<double> & received, double timeout, double now)
{
  return received && now - *received >= 0 && now - *received <= timeout;
}

// Shared by startup and the final command gate. Steady receipt/progress time
// and ROS source time are deliberately independent (including paused /clock).
class LocalizationHealth
{
public:
  explicit LocalizationHealth(double timeout = .5, double recovery_hold = 1.)
  : timeout_(timeout), recovery_hold_(recovery_hold)
  {
    if (!std::isfinite(timeout) || timeout <= 0 || !std::isfinite(recovery_hold) || recovery_hold < 0) {
      throw std::invalid_argument("invalid localization timeout or recovery hold");
    }
  }

  void close()
  {
    trusted_ = false;
    state_progress_.reset();
    pose_progress_.reset();
    pose_received_.reset();
    stable_since_.reset();
  }

  void observe_state(const std::string & text, double received)
  {
    try {
      std::map<std::string, std::string> fields;
      for (std::size_t start = 0; start <= text.size();) {
        const auto end = text.find(';', start);
        const auto item = text.substr(start, end == std::string::npos ? end : end - start);
        const auto eq = item.find('=');
        if (eq == std::string::npos || !fields.emplace(item.substr(0, eq), item.substr(eq + 1)).second) {
          throw std::invalid_argument("malformed state fields");
        }
        if (end == std::string::npos) {break;}
        start = end + 1;
      }
      auto number = [&fields](const char * key) {
          const auto & value = fields.at(key);
          std::size_t used = 0;
          const auto result = std::stoll(value, &used);
          if (used != value.size() || result < 0) {throw std::invalid_argument("invalid state number");}
          return static_cast<std::int64_t>(result);
        };
      const auto epoch = number("epoch"), generation = number("generation");
      const auto revision = number("pose_revision"), stamp = number("stamp_ns");
      const auto identity = std::make_pair(epoch, generation);
      if (identity_ && (identity < *identity_ || (identity == *identity_ && revision < revision_))) {
        throw std::invalid_argument("regressed state version");
      }
      const bool trusted = (fields.at("status") == "NORMAL" || fields.at("status") == "OBS_DEGRADED") &&
        fields.at("pose_trusted") == "true" && fields.at("tf_frozen") == "false";
      if (!identity_ || identity != *identity_) {
        const bool continuous = identity_ && epoch == identity_->first && trusted_ && trusted;
        if (!continuous) {
          close();
          state_stamp_ = pose_stamp_ = 0;
        }
        identity_ = identity;
      }
      revision_ = revision;
      state_received_ = received;
      if (!trusted) {close(); return;}
      if (stamp != 0 && stamp < state_stamp_) {throw std::invalid_argument("regressed state time");}
      if (stamp > state_stamp_) {state_stamp_ = stamp; state_progress_ = received;}
      trusted_ = true;
    } catch (const std::exception &) {
      close();
    }
  }

  void observe_pose(const nav_msgs::msg::Odometry & message, double received)
  {
    const auto stamp = stamp_ns(message.header.stamp);
    const auto & p = message.pose.pose.position;
    const auto & q = message.pose.pose.orientation;
    if (message.header.frame_id != "map" || message.child_frame_id.empty() || stamp <= 0 ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1.e-12 || stamp < pose_stamp_)
    {
      pose_received_.reset(); pose_progress_.reset(); stable_since_.reset(); return;
    }
    pose_received_ = received;
    if (stamp > pose_stamp_) {pose_stamp_ = stamp; pose_progress_ = received;}
  }

  std::vector<std::string> check(double now, std::int64_t ros_now)
  {
    std::vector<std::string> reasons;
    if (!trusted_) {reasons.emplace_back("localization untrusted or malformed");}
    auto source = [&](const char * name, const auto & received, const auto & progress, auto stamp) {
        if (!fresh(received, timeout_, now) || !fresh(progress, timeout_, now)) {
          reasons.push_back(std::string("localization ") + name + " receipt/source-progress timeout");
        }
        const auto age = ros_now - stamp;
        if (stamp <= 0 || age < -50000000LL || static_cast<double>(age) > timeout_ * 1.e9) {
          reasons.push_back(std::string("localization ") + name + " source timestamp expired or invalid");
        }
      };
    source("state", state_received_, state_progress_, state_stamp_);
    source("pose", pose_received_, pose_progress_, pose_stamp_);
    if (!reasons.empty()) {stable_since_.reset(); return reasons;}
    if (!stable_since_) {stable_since_ = now;}
    if (now - *stable_since_ < recovery_hold_) {reasons.emplace_back("localization recovery hold");}
    return reasons;
  }

private:
  double timeout_, recovery_hold_;
  std::optional<std::pair<std::int64_t, std::int64_t>> identity_;
  std::int64_t revision_ = -1, state_stamp_ = 0, pose_stamp_ = 0;
  std::optional<double> state_received_, state_progress_, pose_received_, pose_progress_, stable_since_;
  bool trusted_ = false;
};
}  // namespace xxu_bringup
