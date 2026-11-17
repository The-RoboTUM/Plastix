/**
 * @file scan_stamp.cpp
 * @brief GripperX: /scan timing from per-point stamps, see scan_stamp.h.
 */
#include "scan_stamp.h"

#include <chrono>

namespace scan_stamp
{

uint64_t SystemTimeStampNs()
{
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return static_cast<uint64_t>(tmp.count());
}

bool ParseStampSource(const std::string & name, StampSource * out)
{
  if (name == "publish") {
    *out = StampSource::kPublish;
  } else if (name == "first_ray") {
    *out = StampSource::kFirstRay;
  } else if (name == "mid_sweep") {
    *out = StampSource::kMidSweep;
  } else {
    return false;
  }
  return true;
}

const char * StampSourceName(StampSource source)
{
  switch (source) {
    case StampSource::kPublish: return "publish";
    case StampSource::kFirstRay: return "first_ray";
    case StampSource::kMidSweep: return "mid_sweep";
  }
  return "?";
}

const char * StampFaultName(StampFault fault)
{
  switch (fault) {
    case StampFault::kNone: return "none";
    case StampFault::kNoPoints: return "fewer than two points";
    case StampFault::kZeroStamp: return "zero point stamp";
    case StampFault::kNonMonotonic: return "non-monotonic point stamps";
    case StampFault::kSpanImplausible: return "sweep span implausible for the spin rate";
    case StampFault::kAfterPublish: return "point stamp later than publish time";
    case StampFault::kTooOld: return "point stamp too old";
  }
  return "?";
}

StampFault CheckPointStamps(
  const ldlidar::Points2D & sweep, double spin_freq_hz, int64_t publish_ns)
{
  if (sweep.size() < 2) {
    return StampFault::kNoPoints;
  }
  // Equal neighbours are normal: the last point of a packet and the first of the next
  // both carry the next packet's arrival time (lipkg.cpp).
  for (size_t i = 0; i < sweep.size(); ++i) {
    if (sweep[i].stamp == 0) {
      return StampFault::kZeroStamp;
    }
    if (i > 0 && sweep[i].stamp < sweep[i - 1].stamp) {
      return StampFault::kNonMonotonic;
    }
  }
  const uint64_t first = sweep.front().stamp;
  const uint64_t last = sweep.back().stamp;
  const double span_s = static_cast<double>(last - first) * 1e-9;
  if (!(spin_freq_hz > 0.0)) {
    return StampFault::kSpanImplausible;
  }
  const double period_s = 1.0 / spin_freq_hz;
  if (span_s < kMinSpanRatio * period_s || span_s > kMaxSpanRatio * period_s) {
    return StampFault::kSpanImplausible;
  }
  if (static_cast<int64_t>(last) > publish_ns) {
    return StampFault::kAfterPublish;
  }
  if (static_cast<double>(publish_ns - static_cast<int64_t>(last)) * 1e-9 > kMaxAge_s) {
    return StampFault::kTooOld;
  }
  return StampFault::kNone;
}

ScanTiming ComputeScanTiming(
  StampSource source, const ldlidar::Points2D & sweep, int beam_size,
  bool laser_scan_dir, double spin_freq_hz, int64_t publish_ns, double publish_interval_s)
{
  // Legacy publish timing, unchanged from the LDRobot demo (including the positive
  // time_increment under mirrored beams): the "publish" mode only.
  ScanTiming legacy;
  legacy.stamp_ns = publish_ns;
  legacy.scan_time = static_cast<float>(publish_interval_s);
  legacy.time_increment = (beam_size <= 1) ? 0.0f :
    static_cast<float>(publish_interval_s / static_cast<double>(beam_size - 1));
  legacy.fault = StampFault::kNone;
  if (source == StampSource::kPublish) {
    return legacy;
  }

  StampFault fault = CheckPointStamps(sweep, spin_freq_hz, publish_ns);
  if (fault == StampFault::kNone && beam_size < 2) {
    fault = StampFault::kNoPoints;
  }
  if (fault != StampFault::kNone) {
    ScanTiming bad;
    bad.stamp_ns = 0;
    bad.time_increment = 0.0f;
    bad.scan_time = 0.0f;
    bad.fault = fault;
    return bad;
  }

  const int64_t first = static_cast<int64_t>(sweep.front().stamp);
  const int64_t last = static_cast<int64_t>(sweep.back().stamp);
  const int64_t span = last - first;
  // Beams are evenly spread over the sweep: ranges[0] and ranges[beam_size-1] are its
  // two ends. Mirrored (laser_scan_dir) means ranges[0] is the last acquired point.
  const double step_s = static_cast<double>(span) * 1e-9 / static_cast<double>(beam_size - 1);

  ScanTiming out;
  out.fault = StampFault::kNone;
  out.time_increment = static_cast<float>(laser_scan_dir ? -step_s : step_s);
  out.scan_time = static_cast<float>(step_s * static_cast<double>(beam_size));
  if (source == StampSource::kFirstRay) {
    out.stamp_ns = laser_scan_dir ? last : first;
  } else {
    out.stamp_ns = first + span / 2;
  }
  return out;
}

const char * VerdictName(Verdict verdict)
{
  switch (verdict) {
    case Verdict::kPublish: return "publish";
    case Verdict::kPublishAfterClockStep: return "published after a backward clock step";
    case Verdict::kDropBadStamps: return "dropped: per-point stamps rejected";
    case Verdict::kDropNotIncreasing: return "dropped: stamp not later than the last published";
  }
  return "?";
}

Verdict ScanStamper::Process(
  const ldlidar::Points2D & sweep, int beam_size, bool laser_scan_dir,
  double spin_freq_hz, int64_t publish_ns, double publish_interval_s, ScanTiming * out)
{
  ++scans_;
  *out = ComputeScanTiming(
    source_, sweep, beam_size, laser_scan_dir, spin_freq_hz, publish_ns, publish_interval_s);
  if (source_ == StampSource::kPublish) {
    return Verdict::kPublish;  // legacy, bit for bit: never drops
  }
  if (out->fault != StampFault::kNone) {
    ++dropped_bad_stamps_;
    return Verdict::kDropBadStamps;
  }
  Verdict verdict = Verdict::kPublish;
  if (has_last_ && out->stamp_ns <= last_stamp_ns_) {
    const int64_t back_ns = last_stamp_ns_ - out->stamp_ns;
    if (back_ns <= static_cast<int64_t>(kClockStepThreshold_s * 1e9)) {
      ++dropped_not_increasing_;
      return Verdict::kDropNotIncreasing;
    }
    ++clock_step_resets_;
    last_clock_step_ns_ = back_ns;
    verdict = Verdict::kPublishAfterClockStep;
  }
  has_last_ = true;
  last_stamp_ns_ = out->stamp_ns;
  return verdict;
}

}  // namespace scan_stamp
