#include "gripperx_swerve_controller/stall_detector.hpp"

#include <cmath>
#include <string>

namespace gripperx_swerve_controller
{

StallDetector::StallDetector(const StallDetectorConfig & config) { configure(config); }

bool StallDetector::validate(const StallDetectorConfig & config, std::string & error)
{
  if (config.window_sec <= 0.0) {
    error = "stall_window_sec must be > 0";
    return false;
  }
  if (config.min_command_rad_s <= 0.0) {
    error = "stall_min_command_rad_s must be > 0";
    return false;
  }
  if (config.min_position_delta_rad < 0.0) {
    error = "stall_min_position_delta_rad must be >= 0";
    return false;
  }
  if (config.release_command_rad_s < 0.0) {
    error = "stall_release_command_rad_s must be >= 0";
    return false;
  }
  // THE HYSTERESIS IS THE WHOLE OP-25 ANSWER, so a configuration that removes it
  // is refused rather than accepted and quietly de-fanged. With
  // release >= arm there is a band in which one and the same held command both
  // releases the latch and re-trips it, which is the 30 Hz chatter OP-25 exists
  // to prevent.
  if (config.release_command_rad_s >= config.min_command_rad_s) {
    error = "stall_release_command_rad_s must be < stall_min_command_rad_s (OP-25 hysteresis)";
    return false;
  }
  if (config.max_latched_wheels < 1 || config.max_latched_wheels > kNumWheels) {
    error = "stall_max_latched_wheels must be in [1, 4]";
    return false;
  }
  return true;
}

void StallDetector::configure(const StallDetectorConfig & config)
{
  config_ = config;
  reset();
}

void StallDetector::reset()
{
  latched_.fill(false);
  release_armed_.fill(false);
  armed_.fill(false);
  window_open_.fill(false);
  window_start_sec_.fill(0.0);
  window_start_position_.fill(0.0);
  trip_count_.fill(0);
  // Provenance is deliberately NOT reset: it is latched state published by the
  // hardware component on a TRANSIENT_LOCAL topic and it describes the encoder,
  // not this controller's lifecycle. Dropping it here would silently disarm the
  // detector after every activation until the next (rare) provenance change.
}

void StallDetector::set_provenance(const std::array<int, kNumWheels> & provenance)
{
  provenance_ = provenance;
}

std::size_t StallDetector::latched_count() const
{
  std::size_t count = 0;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    if (latched_[i]) {
      ++count;
    }
  }
  return count;
}

void StallDetector::restart_window(std::size_t wheel, double now_sec, double position)
{
  window_open_[wheel] = true;
  window_start_sec_[wheel] = now_sec;
  window_start_position_[wheel] = position;
}

StallDetectorResult StallDetector::update(
  double now_sec, const std::array<double, kNumWheels> & requested,
  const std::array<double, kNumWheels> & position,
  const std::array<bool, kNumWheels> & position_valid, bool drive_withheld)
{
  StallDetectorResult result;
  result.commands = requested;

  if (!config_.enabled) {
    armed_.fill(false);
    return result;
  }

  std::size_t latched_now = latched_count();

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const double command_magnitude = std::isfinite(requested[i]) ? std::fabs(requested[i]) : 0.0;

    // OP-25 (open decision — option F2's magnitude half): a latched wheel
    // releases only once its own commanded magnitude has FALLEN to or below
    // release_command_rad_s and then RISEN back above min_command_rad_s —
    // two edges, in that order, on the REQUESTED command, never the gated
    // one, which is zero by construction while latched. Not "the next
    // command received" (F1): /cmd_vel repeats a held command at 30 Hz, so
    // F1 would re-energise, re-stall and chatter on/off with inrush current
    // on every cycle — worse than staying off.
    //
    // NOT IMPLEMENTED: release on a DIRECTION change without passing through
    // the release band, and a bounded hold-off (needs a TO-VERIFY value
    // nobody has measured). Both need a user decision; until then the
    // autonomous deadlock SURVIVES here — the other three wheels keep the
    // robot moving, Nav2 sees progress and never reverses, and the latched
    // wheel stays off indefinitely, visibly (stall-state topic, ERROR log,
    // SR-13).
    if (latched_[i]) {
      if (command_magnitude <= config_.release_command_rad_s) {
        release_armed_[i] = true;
      } else if (release_armed_[i] && command_magnitude > config_.min_command_rad_s) {
        latched_[i] = false;
        release_armed_[i] = false;
        window_open_[i] = false;
        result.events[i].released = true;
        result.state_changed = true;
        if (latched_now > 0) {
          --latched_now;
        }
      }
    }

    if (latched_[i]) {
      // TIER 1: the affected wheel's velocity command goes to EXACTLY zero and
      // nothing else is touched — not the other three wheels, not the steering.
      // This is deliberately NOT a hard stop of the machine (HWR-30 tier 1).
      armed_[i] = false;
      window_open_[i] = false;
      result.commands[i] = 0.0;
      continue;
    }

    // Four arming gates: (a) commanded to move, (b) encoder genuinely LIVE,
    // (c) a readable, finite accumulated position to compare against,
    // (d) the command is actually reaching the motor.
    //
    // (b) is BINDING: detection keys off the ENCODER-VALID condition, never
    // off the reported velocity — a dead encoder reporting a plausible 0.0 is
    // bit-identical to a healthy stationary wheel, so velocity cannot
    // separate them. The test is `>= kStallProvenanceLive`, STRICTER than
    // FR-11's `>= LIVE_UNCONFIRMED`: UNCONFIRMED means "begin() succeeded, no
    // count change seen yet", indistinguishable from the fault this detector
    // looks for.
    //
    // (d) is the premise this class rests on: it judges the REQUESTED
    // command (rationale in SwerveController::write_wheel_commands, about
    // the regulator shifting a safety threshold), which presumes the request
    // reaches the motor. The steering alignment gate (stage 2) can replace
    // all four commands with exactly 0.0 until the modules arrive or its
    // timeout expires; under a request above min_command_rad_s the wheel
    // cannot move while such a hold is in effect, so any hold latched a
    // motor off — mirroring the OP-25 rule above, which already refuses to
    // judge the release edge on the gated (zero) command.
    //
    // Disarm, not pause: a fourth arming gate clears window_open_ for its
    // duration, so the first cycle after release RESTARTS the window — the
    // same treatment a command rising from rest already gets, so no new
    // number enters this file (no hold-off constant, no grace period,
    // nothing TO-VERIFY). A resumed pre-hold dwell would judge the wheel
    // against a baseline taken before the drive was removed and could trip
    // within one cycle of its return.
    //
    // It does NOT disarm while the drive is flowing — in particular not
    // after a kAlignTimedOut release with the modules still out of pose,
    // where the motor really is energised against a scrubbing tyre, the load
    // HWR-30a exists to cut.
    //
    // NOT per-wheel: the gate zeroes all four or none, so a per-wheel array
    // here would advertise a withholding no stage performs.
    const bool provenance_live =
      config_.assume_live_provenance || provenance_[i] >= kStallProvenanceLive;
    const bool commanded = command_magnitude > config_.min_command_rad_s;
    const bool feedback_usable = position_valid[i] && std::isfinite(position[i]);

    armed_[i] = commanded && !drive_withheld && provenance_live && feedback_usable;

    if (!armed_[i]) {
      window_open_[i] = false;
      continue;
    }

    if (!window_open_[i]) {
      restart_window(i, now_sec, position[i]);
      continue;
    }

    // A clock that runs backwards (sim time reset, a /clock republished from the
    // start of a bag) must not be read as a long elapsed window.
    if (now_sec < window_start_sec_[i]) {
      restart_window(i, now_sec, position[i]);
      continue;
    }

    // POSITION, NOT VELOCITY. The velocity state is a first difference computed
    // in the firmware and is the quantity HWR-30a forbids keying off. The
    // accumulated position is what the PCNT counter actually holds, and "the
    // count has not moved" is the only statement that separates a dead encoder
    // from a stationary wheel — which is HWR-30a's whole reason to exist.
    if (std::fabs(position[i] - window_start_position_[i]) >= config_.min_position_delta_rad) {
      restart_window(i, now_sec, position[i]);
      continue;
    }

    if ((now_sec - window_start_sec_[i]) < config_.window_sec) {
      continue;
    }

    // -------------------------------------------------------------- would trip
    // TIER 2 IS OUT OF SCOPE AND MUST NOT BE APPROXIMATED BY FOUR TIER-1 LATCHES.
    // HWR-30 defines tier 2 as "sustained, OR affecting more than one motor",
    // and HWR-30b is BLOCKED on the unmeasured GB37-50 stall current. So a trip
    // that would push the number of latched wheels past the cap is REFUSED, and
    // reported as the tier-2 condition it is.
    //
    // This is also the guard against the one false-trip mode that would turn a
    // per-wheel response into a whole-machine stop: if /hw/joint_states stops
    // arriving, every wheel POSITION freezes at once while the commands keep
    // flowing, and all four wheels satisfy the trip condition simultaneously.
    // That is not four stalls, it is one lost feedback path — and the hardware
    // component's own state_timeout_sec handles it a moment later by
    // deactivating (SR-13's evidence block).
    if (latched_now >= config_.max_latched_wheels) {
      result.events[i].multi_wheel_refused = true;
      // Hold the window open at the current sample so the condition is
      // re-evaluated continuously rather than re-armed from scratch, but move
      // the start forward so the refusal is reported once per window, not once
      // per cycle.
      restart_window(i, now_sec, position[i]);
      continue;
    }

    latched_[i] = true;
    release_armed_[i] = false;
    window_open_[i] = false;
    armed_[i] = false;
    ++trip_count_[i];
    ++latched_now;
    result.commands[i] = 0.0;
    result.events[i].tripped = true;
    result.state_changed = true;
  }

  return result;
}

}  // namespace gripperx_swerve_controller
