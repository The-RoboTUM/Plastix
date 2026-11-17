// Whole-drive fault: a wheel driven without a confirmed encoder, or a stall
// that does not stay temporary, stops ALL FOUR wheels and stays stopped until
// the process restarts (or, for the STALL_* reasons, an operator clear). User
// decision 2026-10-16; operator clear: SR-18 amendment 2026-11-09.
//
// WHY THIS EXISTS. On 2026-10-13 the BR encoder stayed LIVE_UNCONFIRMED
// (registered, never counted) for a whole day of driving: its position stayed
// at 0.0 while the other three counted about -11..-12.7 rad. HWR-30a's tier-1
// stall detector (stall_detector.hpp) only watches wheels whose provenance is
// LIVE, so BR drove unwatched. The user rejected regulating on the other three:
// a robot with a missing encoder is a danger to itself, so it does not drive.
//
// TWO TRIGGERS, ONE RESPONSE.
//  1. ENCODER MISSING: a wheel is commanded above
//     encoder_confirm_min_command_rad_s for an ACCUMULATED
//     encoder_confirm_window_sec while its provenance is anything but LIVE
//     (LIVE_UNCONFIRMED, NO_ENCODER, INIT_FAILED, UNKNOWN). LIVE_UNCONFIRMED at
//     standstill is the normal boot state and accumulates nothing.
//  2. PERSISTENT STALL: escalation of HWR-30a's tier-1 single-wheel latch.
//     The latch itself is unchanged and still handles a TEMPORARY stall.
//     "Persistent" is, by user decision 2026-10-16, any of THREE:
//       (a) a second wheel satisfies the stall criterion while one is latched
//           (stall_escalate_on_multi_wheel) — HWR-30b tier-2 territory;
//       (b) the same wheel re-trips within stall_escalation_window_sec of its
//           latch clearing;
//       (c) a wheel stays latched for stall_escalation_latched_sec while its
//           requested command is held above the release band — the OP-25
//           deadlock, where the latch can never clear because the command
//           never drops.
//     A fourth criterion, stall_escalation_trips (N trips since activation),
//     exists but is OFF (0) in the active config: the user did not choose it.
// RESPONSE: all four wheel velocity commands become exactly 0.0. Steering is
// not touched (this class never sees it).
//
// CLEARING. configure() changes the thresholds only, and
// reset_activation_state() clears the per-activation counters only. A zero
// command, a new command, a controller deactivate/activate or a re-configure
// therefore cannot clear the latch. SwerveController additionally mirrors the
// record into process-wide storage so that even unloading and re-loading the
// controller in the same controller_manager process restores it (restore()),
// and into a tmpfs marker so that a crash-respawn restores it too.
// TWO ways out:
//  * a stack restart (every reason);
//  * an OPERATOR CLEAR (SR-18 amendment 2026-11-09), clear() below, for the
//    reasons in drive_fault_reason_clearable() ONLY -- a FIXED list in code,
//    not a parameter, so that widening it (step 2: FEEDBACK_LOST) is a
//    deliberate, reviewed change. The clear re-arms detection; it never
//    suppresses, delays or desensitises it.

// NO rclcpp HERE, same as stall_detector.hpp: the whole state machine is
// unit-checkable without a stack (test_drive_fault_monitor.cpp).

#ifndef GRIPPERX_SWERVE_CONTROLLER__DRIVE_FAULT_MONITOR_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__DRIVE_FAULT_MONITOR_HPP_

#include <array>
#include <cstdint>
#include <string>

#include "gripperx_swerve_controller/stall_detector.hpp"
#include "gripperx_swerve_controller/swerve_kinematics.hpp"

namespace gripperx_swerve_controller
{

/// Mirrors gripperx_control_msgs/msg/DriveFault REASON_* — change both or none.
enum DriveFaultReason : int
{
  kDriveFaultNone = 0,
  kDriveFaultEncoderMissing = 1,
  kDriveFaultStallRetrip = 2,
  kDriveFaultStallTripCount = 3,
  kDriveFaultStallMultiWheel = 4,
  kDriveFaultStallLatchedHeld = 5,
  /// Not one wheel: no wheel LIVE while driving (stale/absent /hw/joint_states,
  /// encoder supply), or every position freezing on the same cycle.
  kDriveFaultFeedbackLost = 6,
};

const char * drive_fault_reason_label(int reason);

/// The reasons an operator may clear without a stack restart (SR-18 amendment
/// 2026-11-09, user decisions D1 and OP-48). A FIXED list, deliberately not
/// a parameter. ENCODER_MISSING stays restart-only; FEEDBACK_LOST is step 2
/// (its own go-ahead) -- adding it is a one-line change here plus its tests.
inline constexpr std::array<int, 4> kOperatorClearableDriveFaultReasons{
  kDriveFaultStallRetrip, kDriveFaultStallTripCount, kDriveFaultStallMultiWheel,
  kDriveFaultStallLatchedHeld};

bool drive_fault_reason_clearable(int reason);

/// Why DriveFaultMonitor::clear() refused, or kNone when it would apply.
enum class DriveFaultClearRefusal : int
{
  kNone = 0,
  kNotLatched,
  kNotClearable,
  kExpectedReasonMismatch,
};

struct DriveFaultConfig
{
  /// Master switch. When false NOTHING NEW can trip — but a latch that is
  /// already set (restored from earlier in this process) is still enforced.
  bool enabled{true};

  /// TO-VERIFY. |requested wheel command| above which a non-LIVE wheel counts
  /// as "driven without an encoder": the command above which a HEALTHY wheel at
  /// least twitches and so produces a count. Since 2026-10-22 the same number
  /// arms HWR-30a's zero-count criterion for LIVE wheels
  /// (StallDetectorConfig::silent_min_command_rad_s).
  double encoder_confirm_min_command_rad_s{0.71};

  /// TO-VERIFY. Accumulated (not necessarily continuous) driven-without-LIVE
  /// time, seconds, before the drive fault latches. The accumulator only resets
  /// when that wheel's provenance reaches LIVE.
  double encoder_confirm_window_sec{2.0};

  /// TO-VERIFY. A wheel whose tier-1 stall latch cleared and which trips again
  /// within this many seconds escalates. 0 disables this criterion; negative
  /// or non-finite is refused by validate().
  double stall_escalation_window_sec{10.0};

  /// A wheel reaching this many tier-1 trips since activation escalates.
  /// 0 disables this criterion — the default, because the user's definition of
  /// a persistent stall (2026-10-16) does not include a trip count.
  uint32_t stall_escalation_trips{0};

  /// TO-VERIFY. A wheel latched by tier 1 for this long, continuously, while
  /// its requested command stays above stall_hold_min_command_rad_s escalates
  /// (OP-25 deadlock). 0 disables this criterion; negative or non-finite is
  /// refused by validate().
  double stall_escalation_latched_sec{5.0};

  /// "Held" for the criterion above means |requested| above this. NOT a
  /// parameter of its own: the controller feeds HWR-30a's
  /// stall_release_command_rad_s, because a command at or below it arms the
  /// latch's release, i.e. it is precisely "not held".
  double stall_hold_min_command_rad_s{0.1};

  /// Classification only (FEEDBACK_LOST vs STALL_MULTI_WHEEL), never the
  /// response. Fed from HWR-30a's stall_window_sec / stall_silent_window_sec.
  double stall_window_sec{1.0};
  double stall_silent_window_sec{2.0};

  /// TO-VERIFY (and see the frozen-feedback note in ros2_controllers.yaml). A
  /// second wheel satisfying the stall criterion while one is latched
  /// escalates.
  bool stall_escalate_on_multi_wheel{true};

  /// The twin publishes no /hw/wheel_feedback_valid. With this true, a wheel is
  /// treated as LIVE until the FIRST provenance message arrives; from then on
  /// the message is believed. So the twin neither trips permanently (nothing
  /// publishes) nor is the feature dead there (publish a provenance array into
  /// the twin and it is exercised). Fed from assume_live_provenance: false on
  /// the robot, where "no message" stays UNKNOWN and is not LIVE.
  bool assume_live_until_provenance_received{false};
};

/// The frozen description of the fault at the moment it latched.
struct DriveFaultRecord
{
  bool active{false};
  int reason{kDriveFaultNone};
  /// Joint-order index (FL, FR, BL, BR) of the wheel that tripped it.
  int wheel{-1};
  double trip_time_sec{0.0};
  /// That wheel's REQUESTED command on the trip cycle, rad/s.
  double commanded_rad_s{0.0};
  /// That wheel's provenance code as the monitor used it on the trip cycle.
  int provenance{kStallProvenanceUnknown};
  /// Encoder trigger: accumulated driven-without-LIVE time at the trip.
  double accumulated_sec{0.0};
  /// Stall triggers: that wheel's tier-1 trip count since activation.
  uint32_t stall_trip_count{0};
  /// Stall re-trip: seconds between that wheel's latch clearing and the re-trip
  /// (negative when not applicable).
  double since_release_sec{-1.0};
  /// Multi-wheel: the wheel that was already latched (or -1). FEEDBACK_LOST:
  /// the wheel whose criterion completed first (wheel is then -1).
  int other_wheel{-1};
  /// Latched-held: how long the latch had been held under command.
  double latched_held_sec{0.0};
  /// Stall reasons: the tier-1 criterion involved was HWR-30a's ZERO-COUNT one
  /// (no encoder count change at all), not net displacement.
  bool zero_count{false};
};

struct DriveFaultInput
{
  double now_sec{0.0};
  /// Requested (feedforward, post multiplier) wheel commands, joint order.
  std::array<double, kNumWheels> requested{};
  /// Latest per-wheel provenance, joint order.
  std::array<int, kNumWheels> provenance{
    kStallProvenanceUnknown, kStallProvenanceUnknown, kStallProvenanceUnknown,
    kStallProvenanceUnknown};
  /// Whether any provenance message has been received since the subscription
  /// was created.
  bool provenance_received{false};
  /// A stage before this one already put EXACTLY 0.0 on every wheel this cycle
  /// (the alignment gate). The wheel cannot turn, so nothing accumulates —
  /// same reasoning as arming gate (d) in stall_detector.cpp.
  bool drive_withheld{false};
  /// HWR-30a tier-1 state AFTER this cycle's stall update. Both all-false when
  /// stall detection is disabled.
  std::array<bool, kNumWheels> stall_latched{};
  std::array<StallWheelEvent, kNumWheels> stall_events{};
  /// StallDetector::no_motion_sec() per wheel, after this cycle's update.
  std::array<double, kNumWheels> stall_no_motion_sec{};
  /// StallDetector::silent_sec() per wheel, after this cycle's update.
  std::array<double, kNumWheels> stall_silent_sec{};
  /// StallDetector::latched_by_zero_count() per wheel, after this cycle's update.
  std::array<bool, kNumWheels> stall_latched_zero_count{};
};

struct DriveFaultResult
{
  /// The commands to write: the input commands, or all four exactly 0.0.
  std::array<double, kNumWheels> commands{};
  /// True only on the cycle the latch was set.
  bool tripped{false};
};

/// The crash-surviving marker (user decision 2026-10-16: a crash-respawn of
/// ros2_control_node must NOT clear the fault). One `key=value` per line.
std::string serialize_drive_fault(const DriveFaultRecord & record);
/// Parses what serialize_drive_fault() wrote. Returns false on anything it
/// cannot read -- the caller then still latches (an unreadable marker is a
/// marker), with reason NONE.
bool parse_drive_fault(const std::string & text, DriveFaultRecord & record);

class DriveFaultMonitor
{
public:
  DriveFaultMonitor() = default;
  explicit DriveFaultMonitor(const DriveFaultConfig & config);

  static bool validate(const DriveFaultConfig & config, std::string & error);

  /// Sets the thresholds. Does NOT touch the latch and does NOT touch the
  /// encoder accumulators (they are evidence about the encoder, not about this
  /// controller's lifecycle).
  void configure(const DriveFaultConfig & config);
  const DriveFaultConfig & config() const { return config_; }

  /// Clears what is defined "since activation": per-wheel tier-1 trip counts,
  /// the last-release times and the accumulator's continuity flag. NEVER the
  /// latch, never the accumulated time.
  void reset_activation_state();

  /// Latch an externally kept record (process-wide mirror). A record that is
  /// not active is ignored — this can set, never clear.
  void restore(const DriveFaultRecord & record);

  /// Whether clear(expected_reason) would apply. `expected_reason` 0 means no
  /// expectation; non-zero must equal the latched reason.
  DriveFaultClearRefusal clear_refusal(int expected_reason) const;

  /// OPERATOR CLEAR (SR-18 amendment 2026-11-09). Applies only when
  /// clear_refusal(expected_reason) is kNone; returns whether it applied.
  /// Releases the latch and resets the A4 latched-held timers of all four
  /// wheels. KEEPS, by user decision OP-49 (a): the A1 re-trip reference --
  /// every wheel named in the record (wheel, other_wheel) counts as released
  /// at `now_sec`, so a tier-1 re-trip within stall_escalation_window_sec
  /// escalates at the first trip. Resets the STALL_TRIP_COUNT escalation
  /// counter of those wheels (OP-48 (a)); the diagnostic trip counts
  /// (stall_trips()) are kept. The encoder-missing accumulators are evidence
  /// about the encoders and are kept, as on every other path. The tier-1
  /// latches, the FR-14 regulator and the marker are the caller's part.
  bool clear(double now_sec, int expected_reason);

  /// One cycle. `commands` is what the earlier stages want to write; the result
  /// is either those, or all four 0.0.
  DriveFaultResult update(
    const DriveFaultInput & input, const std::array<double, kNumWheels> & commands);

  bool latched() const { return record_.active; }
  const DriveFaultRecord & record() const { return record_; }
  /// Accumulated driven-without-LIVE time per wheel, for diagnostics.
  double encoder_unconfirmed_sec(std::size_t wheel) const { return unconfirmed_sec_[wheel]; }
  /// Diagnostic tier-1 trip count since activation (an operator clear keeps it).
  uint32_t stall_trips(std::size_t wheel) const { return stall_trips_[wheel]; }
  /// Trips counted towards stall_escalation_trips (an operator clear resets
  /// it for the wheels named in the cleared fault).
  uint32_t escalation_trips(std::size_t wheel) const { return escalation_trips_[wheel]; }
  double latched_held_sec(std::size_t wheel) const { return latched_held_sec_[wheel]; }
  /// The provenance the encoder check would use for this input.
  int effective_provenance(const DriveFaultInput & input, std::size_t wheel) const;

private:
  void latch(const DriveFaultRecord & record);

  DriveFaultConfig config_{};
  DriveFaultRecord record_{};

  std::array<double, kNumWheels> unconfirmed_sec_{};
  std::array<bool, kNumWheels> unconfirmed_prev_{};
  double prev_now_sec_{0.0};
  bool have_prev_now_{false};

  std::array<uint32_t, kNumWheels> stall_trips_{};
  std::array<uint32_t, kNumWheels> escalation_trips_{};
  std::array<bool, kNumWheels> released_once_{};
  std::array<double, kNumWheels> last_release_sec_{};
  std::array<double, kNumWheels> latched_held_sec_{};
  std::array<bool, kNumWheels> latched_held_prev_{};
};

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__DRIVE_FAULT_MONITOR_HPP_
