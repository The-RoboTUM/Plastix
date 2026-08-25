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
  const std::array<bool, kNumWheels> & position_valid)
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

    // ------------------------------------------------------------ OP-25
    // WHAT COUNTS AS A "FRESH COMMAND" — PROPOSAL, PENDING USER CONFIRMATION.
    // OP-25 is recorded `open` in the internal REQUIREMENTS; this is option F2 narrowed to
    // its magnitude half, implemented so the question has a concrete referent.
    //
    // THE RULE: a latched wheel stays off until its own commanded magnitude has
    // FALLEN to or below release_command_rad_s and then RISEN back above
    // min_command_rad_s. Two edges, in that order, on the REQUESTED command —
    // never on the gated one, which is zero by construction while latched and
    // would otherwise release the latch on the very next cycle.
    //
    // WHY IT IS NOT "the next command received" (option F1, rejected): /cmd_vel
    // runs at 30 Hz whether or not anything changed — a held key in teleop, Nav2
    // still pushing the same twist. F1 re-energises ~33 ms after the cut-off,
    // re-stalls, and produces on/off chatter with motor inrush current on EVERY
    // cycle, which is worse for the thin motor lead than either staying off or
    // staying on. THE SAME HELD NON-ZERO COMMAND CAN NEVER RELEASE THIS LATCH.
    //
    // NOT IMPLEMENTED, and named rather than glossed: F2's other half (release
    // on a DIRECTION change without passing through the release band) and the
    // bounded hold-off that OP-25 offers as the cure for its own autonomous
    // deadlock. Both need a user decision; the hold-off additionally needs a
    // TO-VERIFY value that nobody has measured. The deadlock therefore SURVIVES
    // here: in autonomous operation the other three wheels keep the robot
    // moving, Nav2 sees progress, never reverses, and the latched wheel stays
    // off — visibly (the stall-state topic and the ERROR log, SR-13) but
    // indefinitely.
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

    // -------------------------------------------------- the three arming gates
    // (a) commanded to move,
    // (b) the encoder is genuinely LIVE,
    // (c) a readable, finite accumulated position to compare against.
    //
    // (b) IS BINDING AND IT IS THE POINT OF HWR-30a: the detection keys off the
    // ENCODER-VALID condition, never off the reported velocity. A dead encoder
    // reporting a plausible 0.0 is bit-identical to a healthy stationary wheel
    // (FR-11's superseded provenance criterion records exactly this), so the
    // reported velocity cannot separate them and is not consulted here at all.
    //
    // The test is `>= kStallProvenanceLive`, i.e. LIVE only — STRICTER than
    // FR-11's `>= LIVE_UNCONFIRMED` measurement test. LIVE_UNCONFIRMED means
    // "begin() succeeded and no count change has been seen yet", which is
    // indistinguishable from the very fault this detector looks for. Arming on
    // it would make the detector trip on its own uncertainty.
    const bool provenance_live =
      config_.assume_live_provenance || provenance_[i] >= kStallProvenanceLive;
    const bool commanded = command_magnitude > config_.min_command_rad_s;
    const bool feedback_usable = position_valid[i] && std::isfinite(position[i]);

    armed_[i] = commanded && provenance_live && feedback_usable;

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
