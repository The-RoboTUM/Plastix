#include "gripperx_swerve_controller/alignment_gate.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace gripperx_swerve_controller
{

AlignmentGate::AlignmentGate(const AlignmentGateConfig & config) { configure(config); }

bool AlignmentGate::validate(const AlignmentGateConfig & config, std::string & error)
{
  std::ostringstream message;

  if (config.exit_tolerance_rad <= 0.0) {
    message << "exit_tolerance_rad must be > 0, got " << config.exit_tolerance_rad;
    error = message.str();
    return false;
  }
  if (config.timeout_sec <= 0.0) {
    // A gate with no timeout can immobilise the robot for ever on a
    // steering-feedback fault — a worse failure than the one it prevents.
    message << "timeout_sec must be > 0, got " << config.timeout_sec;
    error = message.str();
    return false;
  }
  // THE HYSTERESIS GAP IS THE WHOLE CONTRACT. With entry at or below exit, one
  // steady pose sitting between them makes the gate release, immediately
  // re-engage, release again — the drive chopping on and off at the controller
  // rate, which is worse than not guarding at all.
  if (config.entry_jump_rad <= config.exit_tolerance_rad) {
    message << "entry_jump_rad (" << config.entry_jump_rad
            << ") must be > exit_tolerance_rad (" << config.exit_tolerance_rad
            << "); without that gap the gate oscillates on one steady pose";
    error = message.str();
    return false;
  }
  if (config.entry_error_rad <= config.exit_tolerance_rad) {
    message << "entry_error_rad (" << config.entry_error_rad
            << ") must be > exit_tolerance_rad (" << config.exit_tolerance_rad
            << "); without that gap the gate oscillates on one steady pose";
    error = message.str();
    return false;
  }
  return true;
}

void AlignmentGate::configure(const AlignmentGateConfig & config)
{
  config_ = config;
  reset();
}

void AlignmentGate::reset()
{
  status_ = config_.enabled ? kAlignPassing : kAlignDisabled;
  slewing_since_sec_ = 0.0;
  max_error_rad_ = 0.0;
  previous_target_.fill(0.0);
  previous_target_valid_ = false;
  engage_count_ = 0;
  timeout_count_ = 0;
}

double AlignmentGate::max_error(
  const std::array<double, kNumWheels> & target_angles,
  const std::array<double, kNumWheels> & measured_angles)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    worst = std::max(worst, std::fabs(normalize_angle(target_angles[i] - measured_angles[i])));
  }
  return worst;
}

AlignmentGateResult AlignmentGate::update(
  double now_sec, const std::array<double, kNumWheels> & requested,
  const std::array<double, kNumWheels> & target_angles,
  const std::array<double, kNumWheels> & measured_angles, bool target_written)
{
  AlignmentGateResult result;
  result.commands = requested;

  if (!config_.enabled) {
    result.status = kAlignDisabled;
    return result;
  }

  // No write this cycle means the steering is being HELD (OP-24/S1), so there is
  // no new target to align to. Keep the state, pass the command through: every
  // branch that holds the steering has already zeroed the wheels for its own
  // reason, so there is nothing here for the gate to withhold.
  if (!target_written) {
    result.status = status_;
    result.max_error_rad = max_error_rad_;
    return result;
  }

  const int previous_status = status_;

  // Capture BEFORE the remembered target is overwritten below: on the first
  // cycle after a reset there is no previous target, and "the target did not
  // jump" and "there was nothing to jump from" are not the same statement.
  const bool had_previous_target = previous_target_valid_;
  double jump = 0.0;
  if (had_previous_target) {
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      jump = std::max(jump, std::fabs(normalize_angle(target_angles[i] - previous_target_[i])));
    }
  }
  // A per-wheel +-180 deg module fold — the thing that makes a crab or spin entry
  // violent — needs no separate detector: folding IS a jump of about pi in the
  // commanded angle, so condition 1 already carries it. A dedicated fold flag
  // would be a second signal saying the same thing, and two signals that must
  // agree are two signals that can disagree.
  previous_target_ = target_angles;
  previous_target_valid_ = true;

  max_error_rad_ = max_error(target_angles, measured_angles);
  result.max_error_rad = max_error_rad_;

  if (status_ == kAlignSlewing) {
    if (max_error_rad_ <= config_.exit_tolerance_rad) {
      status_ = kAlignPassing;
    } else if ((now_sec - slewing_since_sec_) >= config_.timeout_sec) {
      status_ = kAlignTimedOut;
      ++timeout_count_;
    }
  } else {
    // Recover from "released on the timeout" as soon as the pose actually IS
    // confirmed. Without this the flag is a one-way door: one steering-feedback
    // dropout would leave the controller reporting an unconfirmed pose for the
    // rest of the session, and a permanently raised warning is a warning nobody
    // reads.
    if (status_ == kAlignTimedOut && max_error_rad_ <= config_.exit_tolerance_rad) {
      status_ = kAlignPassing;
    }
    // Entry. Two independent conditions — see the field comments: condition 1
    // catches a transition as it starts, condition 2 catches a pose error that
    // was already large when we got here (activation, a wheel moved by hand, the
    // first cycle after a hold branch).
    const bool jumped = had_previous_target && jump > config_.entry_jump_rad;
    const bool far = max_error_rad_ > config_.entry_error_rad;
    if (jumped || far) {
      status_ = kAlignSlewing;
      slewing_since_sec_ = now_sec;
      ++engage_count_;
    }
  }

  if (status_ == kAlignSlewing) {
    // ALL FOUR to exactly zero, never a subset. A pose is a property of the four
    // modules together: driving the wheels that happen to have arrived while the
    // others are still travelling is precisely the motion "in a direction nobody
    // asked for" that this class exists to prevent.
    result.commands.fill(0.0);
  }

  result.status = status_;
  result.state_changed = (status_ != previous_status);
  return result;
}

}  // namespace gripperx_swerve_controller
