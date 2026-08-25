// Decision core of the command watchdog — OP-18a option W2, SR-11, §3.1.5.
//
// WHY THIS IS ITS OWN HEADER, AND WHY IT TOUCHES NO ROS TYPE.
// This is the check that exists because of the 2026-07-06 incident and its
// 2026-08-17 recurrence. Inside GripperXInterface it can only be exercised with
// a controller_manager, a URDF, a live hardware feed and a wedged executor —
// i.e. it can only be tested on the robot, which is exactly where a safety
// check must NOT first be tried. Pulled out, its whole state machine is a pure
// function of (timestamps, twist values, echo sequence numbers) and every case
// below — silence, divergence, recovery, and each named false-positive mode —
// is a deterministic unit test (test/test_command_watchdog.cpp).
//
// WHAT IT DECIDES. Under variant B the controller's input is a body twist and
// its output is per-wheel, so the old input-vs-output comparison would need the
// kinematics inside the safety component. It does not: it compares a twist with
// a twist, via the intent echo swerve_controller publishes from its update
// loop (OP-18a item 1 — never from the controller_manager EXECUTOR, or the
// divergence check collapses into the silence check).
//
//   silence    — /cmd_vel itself stale beyond command_timeout_sec.
//   divergence — /cmd_vel fresh, and it has CHANGED since the last /cmd_vel the
//                controller demonstrably consumed, while the echo's `sequence`
//                has stood still for longer than command_timeout_sec.
//
// THE COUNTER SEMANTICS MATTER AND ARE EASY TO GET BACKWARDS. `sequence` counts
// /cmd_vel messages CONSUMED, not update cycles and not distinct values. So:
//   * a legitimately constant command still advances `sequence` (every
//     republished Twist is a new message) — a frozen VALUE with an advancing
//     sequence is normal operation and must not trip anything;
//   * a frozen `sequence` under a fresh, changing /cmd_vel is the fault.
// The check therefore keys on `sequence`, and uses the twist values only to
// answer "has the operator asked for something the controller has not taken
// up?".
//
// WHY THE "HAS CHANGED" CONDITION IS NOT A WEAKENING. While /cmd_vel does not
// change, a wedged controller commands the last consumed twist — which is the
// twist currently being commanded. The robot is doing what it is told; there is
// nothing to stop. The fault becomes real, and is caught within
// command_timeout_sec, at the first change — which includes the operator
// releasing the key, because teleop_mux then publishes zeros at 20 Hz. The
// superseded wheel-command watchdog behaved identically (a frozen output equal
// to a frozen input diverges from nothing), so this is not a regression.
//
// The comparison is deliberately made STICKY by construction rather than by a
// time window: the change counter is snapshotted at each `sequence` advance, so
// "unconsumed change" stays true until the controller consumes again, and
// clears by itself the moment it does.
//
// CLOCK SOURCE — deviation D17, and this component is on the safe side of it.
// Every time here is a std::chrono::steady_clock ARRIVAL time taken in the
// watchdog's own callbacks. No ROS clock, and deliberately NOT the echo's
// header.stamp, which is ROS time. D17 records that a non-advancing /clock
// makes every `now() - stamp` freshness test in the chain read "always fresh"
// and so fails toward "keep commanding". A steady_clock age cannot be stalled
// by a clock source at all, so both checks here keep working with /clock
// stopped, paused, or absent. The exposed tests are the ones that DO use the
// ROS clock — swerve_controller's cmd_vel_timeout_sec and A2's
// direct_timeout_sec — not these. Reusing header.stamp here to "save a
// subtraction" would hand D17 the watchdog as well; do not.

#ifndef GRIPPERX_HARDWARE_INTERFACE__COMMAND_WATCHDOG_HPP_
#define GRIPPERX_HARDWARE_INTERFACE__COMMAND_WATCHDOG_HPP_

#include <chrono>
#include <cmath>
#include <cstdint>

namespace gripperx_hardware_interface
{

/// Which reference input the watchdog polices against (OP-18a).
enum class WatchdogReference
{
  /// Superseded: /wheel_velocity_controller/commands vs. the command interface.
  /// The topic disappears with the NFR-10 rebuild, but the old chain still runs,
  /// so this stays the default until the switch-over is made in configuration.
  kWheelCommands,
  /// OP-18a / W2: /cmd_vel + the swerve_controller intent echo.
  kTwistEcho
};

struct TwistSample
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

/// Tolerances below which two body twists count as the same command.
///
/// SEPARATE FOR LINEAR AND ANGULAR ON PURPOSE: m/s and rad/s are not the same
/// quantity and a single epsilon over both would silently pick whichever
/// happened to be stricter. See the derivation at the parameter defaults in
/// GripperXInterface — the values are TO-VERIFY and are NOT the per-wheel
/// command_divergence_eps.
struct TwistTolerance
{
  double linear{1.0e-4};
  double angular{1.0e-4};

  bool differs(const TwistSample & a, const TwistSample & b) const
  {
    return std::fabs(a.vx - b.vx) > linear || std::fabs(a.vy - b.vy) > linear ||
           std::fabs(a.wz - b.wz) > angular;
  }
};

class TwistEchoWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  struct Verdict
  {
    /// False while no /cmd_vel has been seen at all (startup grace).
    bool enforcing{false};
    bool silence{false};
    bool divergence{false};
    double cmd_vel_age_sec{0.0};
    /// How long the echo's `sequence` has stood still. Also covers the echo
    /// going silent outright, since a sequence that no longer arrives is a
    /// sequence that no longer advances.
    double echo_frozen_sec{0.0};
    /// True when /cmd_vel is flowing but no echo has EVER arrived. Not a stop
    /// condition — see the note on the startup grace in evaluate() — but the
    /// caller should say so out loud.
    bool echo_never_seen{false};
  };

  void configure(double command_timeout_sec, const TwistTolerance & tolerance)
  {
    command_timeout_sec_ = command_timeout_sec;
    tolerance_ = tolerance;
  }

  void reset()
  {
    cmd_vel_received_ = false;
    cmd_vel_change_count_ = 0;
    echo_seen_ = false;
    echo_sequence_ = 0;
    change_count_at_echo_advance_ = 0;
    last_twist_ = TwistSample{};
  }

  void on_cmd_vel(const TwistSample & twist, Clock::time_point stamp)
  {
    if (cmd_vel_received_ && tolerance_.differs(twist, last_twist_)) {
      ++cmd_vel_change_count_;
    }
    last_twist_ = twist;
    last_cmd_vel_time_ = stamp;
    cmd_vel_received_ = true;
  }

  void on_echo(uint64_t sequence, Clock::time_point stamp)
  {
    if (!echo_seen_ || sequence != echo_sequence_) {
      echo_seen_ = true;
      echo_sequence_ = sequence;
      echo_advance_time_ = stamp;
      // The snapshot is what makes "unconsumed change" self-clearing: whatever
      // the operator changed before this consumption is now accounted for.
      change_count_at_echo_advance_ = cmd_vel_change_count_;
    }
  }

  Verdict evaluate(Clock::time_point now) const
  {
    Verdict verdict;
    if (!cmd_vel_received_ || command_timeout_sec_ <= 0.0) {
      // Startup grace, identical in shape to the one the superseded watchdog
      // had ("nothing received yet -> do not enforce"). It is the reason a
      // never-published /cmd_vel cannot latch a stop at boot.
      return verdict;
    }
    verdict.enforcing = true;
    verdict.cmd_vel_age_sec = seconds_between(last_cmd_vel_time_, now);
    verdict.silence = verdict.cmd_vel_age_sec > command_timeout_sec_;

    if (!echo_seen_) {
      // /cmd_vel is flowing and the controller has never echoed. That is
      // reportable, but it is NOT treated as divergence: at bringup the
      // hardware component activates before the controllers spawn, so this
      // state is also the normal startup window, and a stop latched here would
      // block every start. Left as an explicit flag rather than silently
      // dropped.
      verdict.echo_never_seen = !verdict.silence;
      return verdict;
    }

    verdict.echo_frozen_sec = seconds_between(echo_advance_time_, now);
    const bool unconsumed_change = cmd_vel_change_count_ > change_count_at_echo_advance_;
    verdict.divergence = !verdict.silence && unconsumed_change &&
      verdict.echo_frozen_sec > command_timeout_sec_;
    return verdict;
  }

  uint64_t cmd_vel_change_count() const {return cmd_vel_change_count_;}
  uint64_t echo_sequence() const {return echo_sequence_;}

private:
  static double seconds_between(Clock::time_point from, Clock::time_point to)
  {
    return std::chrono::duration<double>(to - from).count();
  }

  double command_timeout_sec_{0.5};
  TwistTolerance tolerance_{};

  bool cmd_vel_received_{false};
  TwistSample last_twist_{};
  Clock::time_point last_cmd_vel_time_{};
  uint64_t cmd_vel_change_count_{0};

  bool echo_seen_{false};
  uint64_t echo_sequence_{0};
  Clock::time_point echo_advance_time_{};
  uint64_t change_count_at_echo_advance_{0};
};

}  // namespace gripperx_hardware_interface

#endif  // GRIPPERX_HARDWARE_INTERFACE__COMMAND_WATCHDOG_HPP_
