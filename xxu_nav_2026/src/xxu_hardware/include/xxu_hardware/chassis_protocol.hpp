#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace xxu_hardware {
inline std::uint16_t crc16(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);}
  }
  return crc;
}
inline std::int16_t quantize(double value)
{
  const double result = std::round(value * 1000.);
  if (!std::isfinite(result) || result < -32768 || result > 32767) {
    throw std::invalid_argument("velocity exceeds int16 milli-unit range");
  }
  return static_cast<std::int16_t>(result);
}
inline std::vector<std::uint8_t> command_frame(
  std::uint16_t seq, bool run, double vx = 0., double vy = 0., double wz = 0.)
{
  std::vector<std::uint8_t> out{0xaa, 0x55, 2, 0x10,
    static_cast<std::uint8_t>(seq), static_cast<std::uint8_t>(seq >> 8), 7, static_cast<std::uint8_t>(run)};
  for (const auto value : {vx, vy, wz}) {
    const auto q = static_cast<std::uint16_t>(quantize(value));
    if (!run && q != 0) {throw std::invalid_argument("stop frame requires zero velocity");}
    out.push_back(q & 0xff); out.push_back(q >> 8);
  }
  const auto crc = crc16(out.data(), out.size());
  out.push_back(crc & 0xff); out.push_back(crc >> 8);
  return out;
}
struct ChassisStatus
{
  std::uint16_t seq, status, last_seq;
  std::int16_t vx, vy, wz;
  std::uint32_t sample_ms;
  bool wheel_valid() const {return status & 32;}
  bool hardware_fault(bool require_enable) const
  {return !(status & 1) || (status & 12) || (require_enable && !(status & 2));}
  bool healthy(bool require_enable) const {return !hardware_fault(require_enable) && !(status & 16);}
};
struct ParserCounters
{std::uint64_t frames_ok = 0, bad_header = 0, bad_version = 0, bad_type = 0, bad_length = 0, bad_crc = 0;};
class FrameStreamParser
{
public:
  ParserCounters counters;
  void clear() {bytes_.clear();}
  std::vector<ChassisStatus> feed(const std::uint8_t * data, std::size_t size)
  {
    bytes_.insert(bytes_.end(), data, data + size);
    std::vector<ChassisStatus> result;
    while (bytes_.size() >= 9) {
      const std::array<std::uint8_t, 2> sof{0xaa, 0x55};
      const auto found = std::search(bytes_.begin(), bytes_.end(), sof.begin(), sof.end());
      if (found == bytes_.end()) {
        const bool keep = bytes_.back() == 0xaa;
        counters.bad_header += bytes_.size() - (keep ? 1 : 0);
        bytes_.erase(bytes_.begin(), keep ? std::prev(bytes_.end()) : bytes_.end()); break;
      }
      counters.bad_header += static_cast<std::size_t>(found - bytes_.begin());
      bytes_.erase(bytes_.begin(), found);
      if (bytes_.size() < 9) {break;}
      if (bytes_[2] != 2) {++counters.bad_version; bytes_.erase(bytes_.begin()); continue;}
      if (bytes_[3] != 0x20) {++counters.bad_type; bytes_.erase(bytes_.begin()); continue;}
      if (bytes_[6] != 14) {++counters.bad_length; bytes_.erase(bytes_.begin()); continue;}
      constexpr std::size_t length = 23;
      if (bytes_.size() < length) {break;}
      auto u16 = [this](std::size_t offset) {
          return static_cast<std::uint16_t>(bytes_[offset] | (bytes_[offset + 1] << 8));
        };
      if (crc16(bytes_.data(), length - 2) != u16(length - 2)) {
        ++counters.bad_crc; bytes_.erase(bytes_.begin()); continue;
      }
      const auto ms = static_cast<std::uint32_t>(u16(17)) | (static_cast<std::uint32_t>(u16(19)) << 16);
      result.push_back({u16(4), u16(7), u16(9), static_cast<std::int16_t>(u16(11)),
        static_cast<std::int16_t>(u16(13)), static_cast<std::int16_t>(u16(15)), ms});
      ++counters.frames_ok;
      bytes_.erase(bytes_.begin(), bytes_.begin() + length);
    }
    return result;
  }
private:
  std::vector<std::uint8_t> bytes_;
};

class ChassisStateMachine
{
public:
  double status_timeout = .15, ack_timeout = .25;
  bool require_enable = false, running = false;
  std::string reason = "startup";
  std::optional<ChassisStatus> status;
  std::optional<double> status_time;
  std::uint16_t last_tx_seq = 0;
  bool last_tx_run = false;
  void link_reset(double now, const std::string & why)
  {status.reset(); status_time.reset(); saw_nonzero_ = false; sent_.clear(); enter_stop(why, now);}
  void observe(const ChassisStatus & s, double now)
  {
    const bool reboot = running && ((saw_nonzero_ && s.last_seq == 0 &&
      std::find(recent_.begin(), recent_.end(), 0) == recent_.end()) ||
      (status && status->wheel_valid() && s.wheel_valid() &&
      static_cast<std::uint32_t>(s.sample_ms - status->sample_ms) > 0x80000000u));
    status = s; status_time = now;
    if (s.last_seq != 0) {saw_nonzero_ = true;}
    const auto ack = sent_.find(s.last_seq);
    if (ack != sent_.end() && ack->second > acked_order_) {acked_order_ = ack->second; ack_time_ = now;}
    if (running) {
      if (reboot) {enter_stop("stm32_reboot_detected", now);}
      else if (!s.healthy(require_enable)) {enter_stop(unhealthy(s), now);}
      return;
    }
    if (!saw_status_) {saw_status_ = true; baseline_ = s.last_seq;}
    if (s.hardware_fault(require_enable)) {pending_.clear(); baseline_ = s.last_seq; reason = unhealthy(s); return;}
    if (s.status & 16) {reason = "stm32_comm_timeout"; return;}
    const auto stop = pending_.find(s.last_seq);
    if (stop != pending_.end() && (!baseline_ || s.last_seq != *baseline_)) {
      running = true; reason.clear(); acked_order_ = std::max(acked_order_, stop->second);
      pending_.clear(); ack_time_ = now;
    }
  }
  std::vector<std::uint8_t> shutdown_stop() {return command_frame(seq_++, false);}
  std::vector<std::uint8_t> next(double now, double vx, double vy, double wz, bool valid)
  {
    if (running && (!status_time || now - *status_time < 0 || now - *status_time > status_timeout)) {
      enter_stop("status_rx_timeout", now);
    } else if (running && ack_timeout > 0 && ack_time_ && now - *ack_time_ > ack_timeout) {
      enter_stop("cmd_ack_stall", now);
    }
    last_tx_seq = seq_++; ++order_;
    recent_.push_back(last_tx_seq); if (recent_.size() > 128) {recent_.pop_front();}
    last_tx_run = false;
    if (running && valid) {
      try {
        const auto qx = quantize(vx), qy = quantize(vy), qz = quantize(wz);
        last_tx_run = qx != 0 || qy != 0 || qz != 0;
      }
      catch (const std::invalid_argument &) {last_tx_run = false;}
    }
    const auto result = command_frame(last_tx_seq, last_tx_run,
      last_tx_run ? vx : 0., last_tx_run ? vy : 0., last_tx_run ? wz : 0.);
    sent_[last_tx_seq] = order_; trim(sent_);
    if (!running && status && !status->hardware_fault(require_enable)) {
      pending_[last_tx_seq] = order_; trim(pending_);
    }
    return result;
  }
private:
  std::string unhealthy(const ChassisStatus & s) const
  {
    std::string text;
    auto add = [&text](const char * value) {if (!text.empty()) {text += "+";} text += value;};
    if (!(s.status & 1)) {add("not_ready");}
    if (s.status & 4) {add("estop");}
    if (s.status & 8) {add("driver_fault");}
    if (s.status & 16) {add("stm32_comm_timeout");}
    if (require_enable && !(s.status & 2)) {add("run_disabled");}
    return text.empty() ? "unhealthy" : text;
  }
  static void trim(std::map<std::uint16_t, std::uint64_t> & entries)
  {
    if (entries.size() <= 128) {return;}
    const auto oldest = std::min_element(entries.begin(), entries.end(),
      [](const auto & a, const auto & b) {return a.second < b.second;});
    entries.erase(oldest);
  }
  void enter_stop(const std::string & why, double now)
  {
    running = false; reason = why; pending_.clear(); ack_time_.reset();
    const bool fresh = status && status_time && now - *status_time >= 0 && now - *status_time <= status_timeout;
    baseline_ = fresh ? std::optional<std::uint16_t>(status->last_seq) : std::nullopt;
    saw_status_ = fresh;
  }
  std::uint16_t seq_ = 1;
  std::uint64_t order_ = 0, acked_order_ = 0;
  std::map<std::uint16_t, std::uint64_t> sent_, pending_;
  std::deque<std::uint16_t> recent_;
  std::optional<std::uint16_t> baseline_;
  std::optional<double> ack_time_;
  bool saw_status_ = false, saw_nonzero_ = false;
};
}  // namespace xxu_hardware
