#include "gripperx_swerve_controller/wheel_regulator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace gripperx_swerve_controller
{

WheelRegulator::WheelRegulator(const WheelRegulatorConfig & config) { configure(config); }

bool WheelRegulator::validate(const WheelRegulatorConfig & config, std::string & error)
{
  if (!std::isfinite(config.kp) || config.kp < 0.0) {
    error = "wheel_regulator_kp must be finite and >= 0";
    return false;
  }
  if (!std::isfinite(config.ki) || config.ki < 0.0) {
    error = "wheel_regulator_ki must be finite and >= 0";
    return false;
  }
  // The bound is [0, 1), not [0, 1]: at exactly 1.0 a correction of -|setpoint|
  // would write exactly 0.0, indistinguishable at the actuator from an
  // HWR-30a stall latch (FR-14 item 2). A full-trim experiment is still
  // reachable at 0.99.
  if (!std::isfinite(config.max_correction_fraction) || config.max_correction_fraction < 0.0 ||
      config.max_correction_fraction >= 1.0)
  {
    error =
      "wheel_regulator_max_correction_fraction must be in [0, 1) -- STRICTLY below 1.0. It TRIMS "
      "the feedforward and must never be able to reduce it to zero (FR-14 item 2): at exactly 1.0 "
      "a correction of -|setpoint| writes exactly 0.0, which is indistinguishable at the actuator "
      "from an HWR-30a stall latch. Use 0.99 for a full-trim experiment.";
    return false;
  }
  if (!std::isfinite(config.max_sample_age_sec) || config.max_sample_age_sec <= 0.0) {
    error = "wheel_regulator_max_sample_age_sec must be > 0";
    return false;
  }
  if (!std::isfinite(config.output_limit_rad_s) || config.output_limit_rad_s <= 0.0) {
    error = "wheel regulator output limit (max_wheel_angular_speed) must be > 0";
    return false;
  }
  return true;
}

void WheelRegulator::configure(const WheelRegulatorConfig & config)
{
  config_ = config;
  reset();
}

void WheelRegulator::reset()
{
  integrator_.fill(0.0);
  regulating_.fill(false);
  have_previous_sample_ = false;
  previous_measured_.fill(0.0);
  previous_position_.fill(0.0);
  have_sample_time_ = false;
  last_sample_sec_ = 0.0;
}

bool WheelRegulator::detect_new_sample(const WheelRegulatorInput & input)
{
  bool changed = false;
  bool any_usable = false;

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    // Only finite values are evidence, and a non-finite one leaves the stored
    // previous value alone: NaN != NaN is TRUE, so comparing it would report a
    // new sample on every single cycle — the exact opposite of this function's
    // job.
    if (input.measured_valid[i] && std::isfinite(input.measured[i])) {
      any_usable = true;
      if (!have_previous_sample_ || input.measured[i] != previous_measured_[i]) {
        changed = true;
      }
      previous_measured_[i] = input.measured[i];
    }
    if (input.position_valid[i] && std::isfinite(input.position[i])) {
      any_usable = true;
      if (!have_previous_sample_ || input.position[i] != previous_position_[i]) {
        changed = true;
      }
      previous_position_[i] = input.position[i];
    }
  }

  if (any_usable) {
    have_previous_sample_ = true;
  }
  return changed;
}

WheelRegulatorResult WheelRegulator::update(const WheelRegulatorInput & input)
{
  WheelRegulatorResult result;
  // The default result IS the feedforward: commands == setpoint, corrections
  // zero, regulating false. Every path below either leaves it that way or adds
  // a bounded correction.
  result.commands = input.setpoint;

  if (!config_.enabled) {
    // Belt and braces. The controller does not call update() at all while the
    // regulator is disabled (swerve_controller.cpp), so this branch is the
    // second guarantee, not the first.
    integrator_.fill(0.0);
    regulating_.fill(false);
    result.status.fill(kRegulatorDisabled);
    return result;
  }

  const bool sample_is_new = detect_new_sample(input);

  double sample_dt = 0.0;
  bool integrate_step = false;
  if (sample_is_new) {
    if (have_sample_time_) {
      sample_dt = input.now_sec - last_sample_sec_;
      // Two real rejections, not defensive padding: a clock that ran backwards
      // (sim reset, a bag replayed from its start) must never be read as an
      // elapsed interval, and a gap longer than max_sample_age_sec is a HOLE in
      // the feed — integrating across it would credit the error with time
      // during which nothing was measured.
      integrate_step = sample_dt > 0.0 && sample_dt <= config_.max_sample_age_sec;
    }
    have_sample_time_ = true;
    last_sample_sec_ = input.now_sec;
  }

  const double sample_age = have_sample_time_
                              ? (input.now_sec - last_sample_sec_)
                              : std::numeric_limits<double>::infinity();
  const bool feed_fresh =
    have_sample_time_ && sample_age >= 0.0 && sample_age <= config_.max_sample_age_sec;

  for (std::size_t i = 0; i < kNumWheels; ++i) {
    // ------------------------------------------------- the six regulating gates
    // (a) per-wheel PROVENANCE is >= Live (FR-11 item 6) — stricter than
    //     LiveUnconfirmed (encoder initialised, never seen to count), since a
    //     loop closed on an ECHOED value has an identically zero error: perfect
    //     apparent tracking and no control whatsoever.
    // (b) a usable measurement,
    // (c) a usable setpoint,
    // (d) the command is ABOVE THE SLOW-END FLOOR,
    // (e) the feed is FRESH, and
    // (f) the wheel is NOT latched off by HWR-30a.
    //
    // (f) matters because the stall gate that runs AFTER this forces a latched
    // wheel's command to exactly 0.0 (see the ordering note in
    // SwerveController::write_wheel_commands()). A regulator that kept
    // regulating it would integrate (setpoint - 0) all through the latch and
    // hand back a fully wound-up correction the moment the latch cleared — so:
    // no correction, and the integrator is HELD reset.
    const bool provenance_live =
      config_.assume_live_provenance || input.provenance[i] >= kStallProvenanceLive;
    const bool measurement_usable = input.measured_valid[i] && std::isfinite(input.measured[i]);
    const bool setpoint_usable = std::isfinite(input.setpoint[i]);

    // ------------------------------------------------- THE SLOW-END FLOOR
    // FR-14 item 12, acceptance A17.
    //
    // The floor IS HWR-30a's arming threshold, and the coupling is the point:
    // the regulator may only add authority where HWR-30a is watching. Above the
    // threshold a stalled wheel is caught and latched off within
    // stall_window_sec; below it nothing is watching, so a regulator that kept
    // working there would push effort into a machine with no detector behind
    // it. The comparison is character-for-character the detector's own arming
    // test (`command_magnitude > config_.min_command_rad_s` in
    // stall_detector.cpp), on the same quantity — the requested command, before
    // any correction — and the value is passed in per cycle from the detector's
    // own config rather than copied, so the two provably cannot disagree, not
    // even at the boundary. Decoupling them removes this safety property.
    //
    // 2.0 rad/s is ~0.14 m/s at the measured rolling radius (0.070 m), safely
    // above the ~0.06 m/s below which the plant stops being proportional
    // (item 12) — conservative, not a minimal satisfaction of it. Extending the
    // usable slow range is the feedforward offset's job (OP-32 G1), not this
    // regulator's.
    //
    // NO HYSTERESIS, deliberately. A command chattering across the floor resets
    // that wheel's integrator on every crossing, which is bounded and safe: the
    // correction can only ever restart at kp*error. A release band would mean
    // regulating below the floor — i.e. adding authority where HWR-30a is not
    // watching, the one thing this gate exists to prevent — for a width nobody
    // has measured. (stall_release_command_rad_s is not reusable here: it sits
    // far below this floor.)
    const bool above_floor =
      setpoint_usable && std::fabs(input.setpoint[i]) > input.stall_min_command_rad_s;

    const bool regulate = provenance_live && measurement_usable && setpoint_usable && above_floor &&
                          feed_fresh && !input.stall_latched[i];

    regulating_[i] = regulate;
    result.regulating[i] = regulate;
    // WHY, not just whether (FR-14 item 7): the order of these tests is the
    // order of the gates above, and a wheel that fails more than one reports
    // the first, most fundamental one.
    //
    // The floor is tested ABOVE freshness on purpose: A17's own scenario is "a
    // command below the floor and zero measured speed", and a wheel that is not
    // moving also becomes STALE within max_sample_age_sec. Testing freshness
    // first would report kRegulatorOffStaleFeedback and hide the floor exactly
    // where it is the operative reason. It sits BELOW provenance and
    // measurement usability for the mirror-image reason: those are faults with
    // a remedy, the floor is a design limit, and a fault must not be masked by
    // a limit.
    if (!provenance_live) {
      result.status[i] = kRegulatorOffProvenance;
    } else if (!measurement_usable || !setpoint_usable) {
      result.status[i] = kRegulatorOffNoMeasurement;
    } else if (!above_floor) {
      result.status[i] = kRegulatorOffBelowFloor;
    } else if (!feed_fresh) {
      result.status[i] = kRegulatorOffStaleFeedback;
    } else if (input.stall_latched[i]) {
      result.status[i] = kRegulatorOffStallLatched;
    } else {
      result.status[i] = kRegulatorActive;
    }

    if (!regulate) {
      // Pure feedforward; the integrator is CLEARED rather than frozen, since a
      // frozen integrator would resume with authority earned under conditions
      // since declared untrustworthy — including a below-floor wheel, which
      // gets exactly the latched wheel's treatment: held at zero so a slow
      // crawl cannot hand a wound-up correction to the wheel the moment the
      // command rises back through the floor.
      integrator_[i] = 0.0;
      continue;
    }

    // ------------------------------------------------------- authority bound
    // `input.setpoint[i]` IS the feedforward output at this layer, so this line
    // is literally "max_correction_fraction of the feedforward output". The
    // bound is a fraction of THIS cycle's setpoint, so it shrinks with the
    // command.
    //
    // The floor (above) SUBSUMES the standstill case rather than duplicating
    // it: a zero setpoint is below any positive floor, so standstill never
    // reaches this line — it already returned with kRegulatorOffBelowFloor and
    // a zeroed integrator.
    const double bound = config_.max_correction_fraction * std::fabs(input.setpoint[i]);
    const double error = input.setpoint[i] - input.measured[i];

    if (integrate_step) {
      const double requested = integrator_[i] + config_.ki * error * sample_dt;
      double candidate = requested;

      // ANTI-WINDUP by back-calculation: when the step would drive the OUTPUT
      // past the bound, the integrator is set to exactly the value that puts
      // the output ON the bound, so it stores no more authority than it may
      // spend and the output leaves saturation on the very next sample once the
      // error reverses. Refusing the step outright instead would leave the
      // integrator short of the bound, so the regulator would never actually
      // attain its permitted authority.
      const double raw = config_.kp * error + candidate;
      if (raw > bound) {
        candidate = bound - config_.kp * error;
      } else if (raw < -bound) {
        candidate = -bound - config_.kp * error;
      }

      // The back-calculation may only TRUNCATE the requested step, never move
      // the integrator the other way — otherwise a large kp that saturates the
      // output on its own would drive the integrator to the OPPOSITE bound, and
      // that stored counter-correction would fire in full once the error
      // vanished.
      const double lower = std::min(integrator_[i], requested);
      const double upper = std::max(integrator_[i], requested);
      candidate = std::max(lower, std::min(upper, candidate));

      integrator_[i] = candidate;
    }

    // THE INTEGRATOR ITSELF IS CLAMPED, not just the output — clamping only the
    // output would leave a state wound past the authority bound that has to
    // unwind before the correction responds at all. This runs on EVERY cycle,
    // including ones with no new sample, because `bound` follows the setpoint:
    // when the command drops, the stored authority drops with it.
    integrator_[i] = std::max(-bound, std::min(bound, integrator_[i]));

    double correction = config_.kp * error + integrator_[i];
    correction = std::max(-bound, std::min(bound, correction));

    // The corrected command must respect the same wheel-speed ceiling the
    // control law already applied to the setpoint (max_wheel_angular_speed).
    // Expressed as a bound on the CORRECTION rather than a clamp on the sum, so
    // a setpoint already outside the ceiling could only ever be corrected back
    // towards it, never further out.
    const double correction_hi = std::max(0.0, config_.output_limit_rad_s - input.setpoint[i]);
    const double correction_lo = std::min(0.0, -config_.output_limit_rad_s - input.setpoint[i]);
    correction = std::max(correction_lo, std::min(correction_hi, correction));

    // AT THE AUTHORITY LIMIT is a DISTINCT REPORTED STATE (FR-14 item 7): a
    // regulator sitting on its bound is saying "the feedforward is wrong for
    // this surface" (OP-32), which must not look the same as one with nothing
    // to do. The test is exact, not tolerant, because the clamp above produces
    // exactly +-bound when it bites, and `bound > 0` excludes standstill, where
    // the bound is zero and a zero correction is not a saturated one.
    if (bound > 0.0 && std::fabs(correction) >= bound) {
      result.status[i] = kRegulatorAtAuthorityLimit;
    }

    result.correction[i] = correction;
    // TRIM, NEVER REPLACE: what leaves this class is always the kinematic
    // setpoint plus a bounded correction, never a raw controller output.
    result.commands[i] = input.setpoint[i] + correction;
  }

  return result;
}

}  // namespace gripperx_swerve_controller
