/**
 * @file scan_stamp.h
 * @brief GripperX: header stamp, time_increment and scan_time of a /scan sweep,
 *        derived from the driver's per-point stamps (no ROS dependency, unit-tested).
 *
 * Per-point stamps come from LiPkg::Parse (lipkg.cpp): the host clock registered via
 * RegisterGetTimestampFunctional, i.e. GetSystemTimeStamp() in demo.cpp =
 * std::chrono::system_clock (CLOCK_REALTIME), the same clock node->now() reads when the
 * node runs without use_sim_time. They are HOST ARRIVAL times of each 12-point packet,
 * linearly spread over the packet, so they carry the serial transfer and read latency;
 * they are not sensor times. AssemblePacket sorts a sweep by stamp (lipkg.cpp), so the
 * stamps of a delivered sweep are never decreasing and kNonMonotonic cannot fire in
 * production; the check is kept only as a defensive guard against a driver change. The
 * sweep starts at the 0 deg crossing, so the earliest point is raw angle ~0, latest ~360.
 *
 * Drops (first_ray / mid_sweep only): a scan whose per-point stamps fail the checks is
 * NOT published (no publish-time or extrapolated stamp is invented: a publish-time stamp
 * lies later than the next good stamp and makes /scan go backwards, and rf2o divides by
 * the stamp difference). A scan whose stamp is not strictly later than the last published
 * one is dropped too; this also catches LiPkg::GetLaserScanData handing out the same
 * frame twice (its ready flag is reset before the copy). publish mode never drops.
 *
 * Beam order: with laser_scan_dir == true demo.cpp mirrors the beams
 * (index = beam_size - index - 1), so ranges[0] is the LAST acquired point and the
 * per-beam time runs backwards: time_increment is negative.
 *
 * Modes (ROS parameter stamp_source):
 *   publish   - legacy: node->now() at publish, scan_time = interval between publishes,
 *               time_increment = scan_time / (beam_size - 1), always positive. Kept bit
 *               for bit as the "unfixed" baseline.
 *   first_ray - sensor_msgs/LaserScan convention: stamp = acquisition time of ranges[0],
 *               so that stamp + i * time_increment is the time of ranges[i]. Under
 *               laser_scan_dir == true that is the LAST acquired point of the sweep.
 *   mid_sweep - stamp = middle of the sweep (first + span / 2). DEVIATES from the
 *               LaserScan convention on purpose: the /scan consumers on this robot
 *               (slam_toolbox, rf2o, the costmaps; laser scan matchers in general)
 *               do not deskew and treat a scan as a snapshot at its stamp, and the
 *               snapshot's mean acquisition time is mid-sweep. stamp + i * time_increment
 *               is NOT the time of ranges[i] in this mode (it is off by half the span).
 */
#ifndef SCAN_STAMP_H_
#define SCAN_STAMP_H_

#include <cstdint>
#include <string>

#include "ldlidar_datatype.h"

namespace scan_stamp {

  enum class StampSource { kPublish, kFirstRay, kMidSweep };

// Why per-point stamps were rejected for one scan. kNone = accepted (or publish mode).
  enum class StampFault
  {
    kNone,
    kNoPoints,       // fewer than two points
    kZeroStamp,      // a point stamp is 0 (unset)
    kNonMonotonic,   // stamps decrease along the sweep (defensive only, see above)
    kSpanImplausible, // first..last outside [kMinSpanRatio, kMaxSpanRatio] * 1/spin_freq
    kAfterPublish,   // last point stamp later than the publish time (clock step)
    kTooOld,         // last point stamp more than kMaxAge_s before publish (clock step)
  };

// A sweep shorter than half a revolution or longer than 1.5 revolutions is not a sweep
// (AssemblePacket itself discards frames above 1.4 revolutions of points).
  constexpr double kMinSpanRatio = 0.5;
  constexpr double kMaxSpanRatio = 1.5;
// The publish loop picks a frame up within one 10 Hz tick of its assembly; a second is
// far outside that and only reachable by a clock step or a wrong clock.
  constexpr double kMaxAge_s = 1.0;
// Sanity bound, not a measured value: a new stamp more than this BEFORE the last published
// one is taken as a backward system-clock step (the Pi has no RTC; chrony can step it),
// not as a stale or duplicate frame. One scan period is 0.1 s; consecutive good stamps
// differ by about one period and a duplicate frame by exactly 0, so 1 s (10 periods)
// cannot be reached by frame order alone. Without the reset, a step of B seconds would
// drop every scan for B seconds.
  constexpr double kClockStepThreshold_s = 1.0;

  struct ScanTiming
  {
    int64_t stamp_ns;
    float time_increment;
    float scan_time;
    StampFault fault; // != kNone: per-point stamps rejected, the scan must be dropped
  };

// Clock of the per-point stamps (demo.cpp GetSystemTimeStamp): system_clock in ns.
  uint64_t SystemTimeStampNs();

  bool ParseStampSource(const std::string & name, StampSource * out);
  const char * StampSourceName(StampSource source);
  const char * StampFaultName(StampFault fault);

  // kPublishAfterClockStep: published, the stored last stamp was reset (clock stepped back).
  enum class Verdict { kPublish, kPublishAfterClockStep, kDropBadStamps, kDropNotIncreasing };
  const char * VerdictName(Verdict verdict);

// Validates the stamps of a sweep as delivered by the driver (sorted by stamp).
  StampFault CheckPointStamps(
    const ldlidar::Points2D & sweep, double spin_freq_hz, int64_t publish_ns);

// beam_size: number of LaserScan beams (demo.cpp: points in the sweep).
// publish_interval_s: time since the previous publish (legacy scan_time).
// fault != kNone only in first_ray / mid_sweep; the other fields are then meaningless.
  ScanTiming ComputeScanTiming(
    StampSource source, const ldlidar::Points2D & sweep, int beam_size,
    bool laser_scan_dir, double spin_freq_hz, int64_t publish_ns, double publish_interval_s);

// Stateful over consecutive scans: decides publish or drop and counts the drops.
  class ScanStamper
  {
public:
    explicit ScanStamper(StampSource source)
    : source_(source) {
    }

    // On kPublish *out holds the timing to publish and the stamp becomes the last one.
    Verdict Process(
      const ldlidar::Points2D & sweep, int beam_size, bool laser_scan_dir,
      double spin_freq_hz, int64_t publish_ns, double publish_interval_s, ScanTiming * out);

    StampSource source() const {return source_;}
    uint64_t scans() const {return scans_;}
    uint64_t dropped_bad_stamps() const {return dropped_bad_stamps_;}
    uint64_t dropped_not_increasing() const {return dropped_not_increasing_;}
    uint64_t clock_step_resets() const {return clock_step_resets_;}
    // Size of the last backward step that caused a reset (last stamp - new stamp), ns.
    int64_t last_clock_step_ns() const {return last_clock_step_ns_;}

private:
    StampSource source_;
    bool has_last_ = false;
    int64_t last_stamp_ns_ = 0;
    uint64_t scans_ = 0;
    uint64_t dropped_bad_stamps_ = 0;
    uint64_t dropped_not_increasing_ = 0;
    uint64_t clock_step_resets_ = 0;
    int64_t last_clock_step_ns_ = 0;
  };

}  // namespace scan_stamp

#endif  // SCAN_STAMP_H_
