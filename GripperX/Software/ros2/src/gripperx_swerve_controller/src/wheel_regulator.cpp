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
  // A "correction" larger than the setpoint is not a correction. FR-14 item 2:
  // the regulator TRIMS the feedforward and "MUST NOT be able to reduce the
  // feedforward contribution to zero". AT EXACTLY 1.0 IT CAN: a correction of
  // -|setpoint| makes the written command exactly 0.0, which is
  // INDISTINGUISHABLE AT THE ACTUATOR FROM AN HWR-30a STALL LATCH — a wheel
  // silently switched off by the regulator, wearing the appearance of the tier-1
  // response. So the bound is [0, 1), not [0, 1]. A full-trim experiment is
  // still reachable at 0.99, which has to be typed and looks deliberate.
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
  // zero, regulating false. Every path below either leaves it that way or adds a
  // bounded correction to it.
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
      // Two rejections, both of them real cases rather than defensive padding:
      // a clock that ran backwards (sim time reset, a bag replayed from its
      // start) must never be read as an elapsed interval, and a gap longer than
      // max_sample_age_sec is a HOLE in the feed. Integrating across the hole
      // would credit the error with time during which nothing was measured.
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
    // ------------------------------------------------- the four regulating gates
    // (a) PER-WHEEL PROVENANCE — FR-11 item 6, and this class is its first
    //     consumer. "Echoed feedback MUST be a visible state [...] and MUST
    //     DISABLE any later closed-loop law rather than feeding it." The test is
    //     `>= Live`, the same one HWR-30a uses and stricter than FR-11's
    //     `>= LiveUnconfirmed` measurement test: LiveUnconfirmed means the
    //     encoder initialised and has never been seen to count, which is not a
    //     basis for closing a loop. A loop closed on an ECHO has an identically
    //     zero error, perfect apparent tracking and no control whatsoever —
    //     FR-11 calls that "the hardest kind of fault to find".
    // (b) a usable measurement,
    // (c) a usable setpoint,
    // (d) the command is ABOVE THE SLOW-END FLOOR,
    // (e) the feed is FRESH, and
    // (f) the wheel is NOT latched off by HWR-30a.
    //
    // (f) IS NOT A NICETY — see the ordering note in
    // SwerveController::write_wheel_commands(). A latched wheel's command is
    // forced to exactly 0.0 by the stall gate that runs AFTER this. If the
    // regulator kept regulating it, it would spend the whole latch integrating
    // an error of (setpoint - 0) that its own command cannot influence, and the
    // moment the latch cleared the wheel would receive a fully wound-up
    // correction. So: no correction, and the integrator is HELD reset.
    const bool provenance_live =
      config_.assume_live_provenance || input.provenance[i] >= kStallProvenanceLive;
    const bool measurement_usable = input.measured_valid[i] && std::isfinite(input.measured[i]);
    const bool setpoint_usable = std::isfinite(input.setpoint[i]);

    // ------------------------------------------------- THE SLOW-END FLOOR
    // FR-14 item 12, acceptance A17. USER DECISION 2026-08-21.
    //
    // THE FLOOR IS HWR-30a's ARMING THRESHOLD, AND THE COUPLING IS THE POINT,
    // NOT A CONVENIENCE: THE REGULATOR MAY ONLY ADD AUTHORITY WHERE HWR-30a IS
    // WATCHING. Above the threshold, a wheel that is commanded and does not turn
    // is caught and latched off within stall_window_sec; below it, nothing is
    // watching, and a regulator that kept working there would push effort into a
    // machine that will not start with no detector behind it. So the two use ONE
    // number: this comparison is character for character the detector's own
    // arming test (`command_magnitude > config_.min_command_rad_s` in
    // stall_detector.cpp), on the same quantity — the REQUESTED command, before
    // any correction — so the two cannot disagree, not even at the boundary.
    // The value is passed in per cycle from the detector's own config rather
    // than copied into this class's config, so moving stall_min_command_rad_s
    // moves the floor with it automatically. IF YOU EVER DECOUPLE THESE, THE
    // SAFETY PROPERTY GOES WITH THEM.
    //
    // WHY THIS FLOOR AND NOT A LOWER ONE. A very low floor was considered and
    // REJECTED by the user (2026-08-21) — one chosen so the regulator's own
    // authority would just reach the measured breakaway point (~0.0385 m/s).
    // Five reasons, all of them measured rather than argued: the full 30 %
    // authority is worth 1.67 PWM counts down there while breakaway itself is
    // only known to about 2 counts, so the effect sits INSIDE the uncertainty of
    // the threshold it is trying to cross; it would make permanent operation AT
    // the authority limit the normal state, which is the exact condition A17 and
    // OP-32 exist to flag; it would act entirely OUTSIDE HWR-30a's coverage; the
    // plant is not proportional there at all (measured: below ~0.06 m/s the
    // machine rolls at 0.03-0.07 m/s or stands still, and identical commands give
    // different outcomes); and it would be surface-dependent in the DANGEROUS
    // direction, because breakaway is static friction — on a lower-friction
    // surface the same extra counts become overspeed rather than starting help.
    // EXTENDING THE USABLE SLOW RANGE IS THE FEEDFORWARD OFFSET'S JOB (OP-32 G1,
    // runtime-settable `a`), not this regulator's.
    //
    // 2.0 rad/s is 0.14 m/s at the measured rolling radius 0.070 m, i.e. safely
    // ABOVE the ~0.06 m/s below which item 12 says the plant misbehaves. The
    // floor is therefore conservative with respect to item 12, not a minimal
    // satisfaction of it.
    //
    // NO HYSTERESIS, DELIBERATELY. A command oscillating across the floor toggles
    // regulation, and each toggle resets that wheel's integrator. That is the
    // SAFE direction and it is bounded: after a crossing the correction can only
    // ever restart at kp*error, so the worst a chattering command can produce is
    // a repeatedly small correction, never a wound-up one. A release band would
    // mean regulating BELOW the floor for as long as the command hovered there —
    // i.e. adding authority where HWR-30a is not watching, which is the one thing
    // this gate exists to prevent — and its width would be a number nobody has
    // measured. (stall_release_command_rad_s is NOT reused for it: 0.1 rad/s
    // would let the regulator work down to 0.007 m/s, far below the floor.) The
    // reported STATUS toggles with it, which is cosmetic.
    const bool above_floor =
      setpoint_usable && std::fabs(input.setpoint[i]) > input.stall_min_command_rad_s;

    const bool regulate = provenance_live && measurement_usable && setpoint_usable && above_floor &&
                          feed_fresh && !input.stall_latched[i];

    regulating_[i] = regulate;
    result.regulating[i] = regulate;
    // WHY, not just whether — FR-14 item 7. The order of these tests is the
    // order of the gates above; a wheel that fails more than one reports the
    // first, which is the most fundamental.
    //
    // THE FLOOR SITS ABOVE THE FRESHNESS TEST ON PURPOSE, AND A17 IS THE REASON.
    // A17's own scenario is "a command below the floor and a measured speed of
    // zero" — and a machine that is not moving emits a feed in which nothing
    // changes, so within max_sample_age_sec it also becomes STALE. If freshness
    // were tested first, the one case the criterion names would report
    // kRegulatorOffStaleFeedback and the floor would be invisible exactly where
    // it is the operative reason. The floor is also the more STRUCTURAL of the
    // two: staleness is transient, the floor is a property of the command.
    // It sits BELOW provenance and measurement usability for the mirror-image
    // reason: those are faults with a remedy (a dead encoder, an echoed feed),
    // the floor is a design limit, and a fault must not be masked by a limit.
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
      // Pure feedforward, and the integrator is cleared rather than frozen. A
      // frozen integrator would resume with authority earned under conditions
      // that have since been declared untrustworthy — which is the same reason
      // the latch case above resets rather than holds.
      //
      // A BELOW-FLOOR WHEEL GETS EXACTLY THE LATCHED WHEEL'S TREATMENT, and it
      // is this line that gives it: correction exactly 0.0, integrator reset and
      // HELD reset for as long as the command stays down there. So a slow crawl
      // cannot accumulate authority that is handed to the wheel the moment the
      // command rises back through the floor — the same wind-up A17 forbids and
      // the same reason the latch case resets.
      integrator_[i] = 0.0;
      continue;
    }

    // ------------------------------------------------------- authority bound
    // THE REFERENCE QUANTITY IS THE FEEDFORWARD OUTPUT (user decision
    // 2026-08-20, closing FR-14 item 8's open question). `input.setpoint[i]` IS
    // the feedforward output at this layer — it is what the open-loop control law
    // computed for this wheel this cycle — so this line reads "30 % of the
    // feedforward output" literally, not by approximation.
    //
    // Bounded as a fraction of THIS cycle's setpoint magnitude, so the bound
    // shrinks with the command and is exactly 0.0 at standstill: on the
    // zero-twist and stale-twist branches, where write_wheel_commands() is
    // called with zeros, the regulator provably cannot add anything and the
    // integrator is flushed by the clamp below.
    //
    // THE FLOOR AND THIS BOUND DO NOT FIGHT — the floor SUBSUMES the standstill
    // case rather than duplicating it. A zero setpoint is below any positive
    // floor, so standstill now never reaches this line at all; it returns above
    // with kRegulatorOffBelowFloor and a zeroed integrator, which is a stricter
    // and better-explained version of what the zero bound achieved silently.
    const double bound = config_.max_correction_fraction * std::fabs(input.setpoint[i]);
    const double error = input.setpoint[i] - input.measured[i];

    if (integrate_step) {
      const double requested = integrator_[i] + config_.ki * error * sample_dt;
      double candidate = requested;

      // ANTI-WINDUP, by back-calculation to the saturation limit rather than by
      // refusing the step outright. When the step would drive the OUTPUT past
      // the authority bound, the integrator is set to exactly the value that
      // puts the output ON the bound — so it stores the authority it may spend
      // and not one unit more. The moment the error reverses, the output leaves
      // saturation on the very next sample: there is no accumulated surplus to
      // unwind first, which is the entire failure mode "windup" names.
      //
      // Plain refusal was tried and rejected: it leaves the integrator up to one
      // step BELOW the value that reaches the bound, so the regulator gives away
      // a slice of its own permitted authority and the bound is never actually
      // attained.
      const double raw = config_.kp * error + candidate;
      if (raw > bound) {
        candidate = bound - config_.kp * error;
      } else if (raw < -bound) {
        candidate = -bound - config_.kp * error;
      }

      // The back-calculation may only ever TRUNCATE the requested step; it must
      // never move the integrator the other way. Without this, a large kp that
      // saturates the output on its own would drive the integrator to the
      // OPPOSITE bound to compensate — and that stored counter-correction would
      // be applied in full the moment the error went away.
      const double lower = std::min(integrator_[i], requested);
      const double upper = std::max(integrator_[i], requested);
      candidate = std::max(lower, std::min(upper, candidate));

      integrator_[i] = candidate;
    }

    // THE INTEGRATOR ITSELF IS CLAMPED, not just the output. Clamping only the
    // output leaves a state that has wound past the authority bound and then
    // has to unwind through it before the correction responds at all. This runs
    // on EVERY cycle, including cycles with no new sample, because `bound`
    // follows the setpoint: when the command drops, the stored authority drops
    // with it.
    integrator_[i] = std::max(-bound, std::min(bound, integrator_[i]));

    double correction = config_.kp * error + integrator_[i];
    correction = std::max(-bound, std::min(bound, correction));

    // The corrected command must respect the same wheel-speed ceiling the
    // control law already applied to the setpoint (max_wheel_angular_speed,
    // saturated in update() BEFORE the correction exists). Expressed as a bound
    // on the CORRECTION rather than a clamp on the sum, so that a setpoint
    // already outside the ceiling — which the control law does not produce —
    // could only ever be corrected back towards it, never further out.
    const double correction_hi = std::max(0.0, config_.output_limit_rad_s - input.setpoint[i]);
    const double correction_lo = std::min(0.0, -config_.output_limit_rad_s - input.setpoint[i]);
    correction = std::max(correction_lo, std::min(correction_hi, correction));

    // AT THE AUTHORITY LIMIT is a DISTINCT REPORTED STATE (FR-14 item 7), not an
    // inference a reader has to make: a regulator sitting on its bound is saying
    // "the feedforward is wrong for this surface" — the OP-32 condition — and
    // that must not look the same as a regulator with nothing to do.
    // The test is exact rather than tolerant because the clamp above produces
    // exactly +-bound when it bites; the output ceiling can only ever REDUCE the
    // magnitude further, so `>= bound` means the AUTHORITY clamp bound it and
    // not the speed ceiling. `bound > 0` excludes the standstill case, where the
    // bound is zero and a zero correction is not a saturated one.
    if (bound > 0.0 && std::fabs(correction) >= bound) {
      result.status[i] = kRegulatorAtAuthorityLimit;
    }

    result.correction[i] = correction;
    // TRIM, NEVER REPLACE (design point 2): what leaves this class is always the
    // kinematic setpoint plus a bounded correction, never a raw controller
    // output.
    result.commands[i] = input.setpoint[i] + correction;
  }

  return result;
}

}  // namespace gripperx_swerve_controller
