// Functional-equivalence harness, C++ side (NFR-10 acceptance 1).
//
// Reads one case per line from stdin:
//
//     vx vy omega d0 d1 d2 d3 [o0 o1 o2 o3]
//
// d0..d3 are the CURRENT steering angles in MODEL order (FL, BL, BR, FR),
// radians. o0..o3 are OPTIONAL and are the POST-ARBITRATION commanded steering
// angles for the same cycle, also in model order — i.e. what an active
// /teleop/direct_steer override (arbitration point A2) makes the controller
// command instead of the IK target. Their presence is what lets this harness
// exercise the reference change described below; without them a case is a
// plain no-override case.
//
// Prints, per case:
//
//     <status> a0 s0 c0 a1 s1 c1 a2 s2 c2 a3 s3 c3
//
// with a = commanded steering angle, s = commanded wheel angular velocity,
// c = the steer-alignment (slew braking) scale that produced s.
//
// --reference=ik         brake against the IK-derived target  (swerve_cmd_node's
//                        behaviour today; the DEFAULT, so an unqualified run of
//                        this harness still reproduces the old chain)
// --reference=commanded  brake against the post-arbitration commanded target
//                        (the controller's behaviour after the 2026-08-19 user
//                        decision)
//
// The two modes are IDENTICAL on every case that carries no override columns —
// there the commanded target IS the IK target. That is the property the 413-case
// comparison against test/dump_reference.py checks, and it is why the reference
// change does not invalidate the equivalence harness: it narrows its scope to
// the no-override cases and makes the override cases a separate, intended-
// difference set.
//
// test/dump_reference.py does the same through the LIVE Python modules
// (gripperx_control.steering_limits + swerve_kinematic_model + swerve_cmd_node's
// _steer_alignment_scale). It has no --reference switch on purpose: today's
// chain cannot see the override, so "ik" is the only reference it can offer.
// Diffing the two outputs is what makes "functionally equivalent to today's
// chain" a check rather than a claim.
//
// The steer-feedback differential is deliberately OUT of this harness: it is
// stateful (a first-order filter over dt), so it is not a pure function of one
// twist. It is ported line for line in the controller and reviewed there.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "gripperx_swerve_controller/steering_limits.hpp"
#include "gripperx_swerve_controller/swerve_kinematics.hpp"

using namespace gripperx_swerve_controller;  // NOLINT(build/namespaces)

namespace
{
// Values from gripperx_control/config/swerve_cmd.yaml, unchanged.
constexpr double kA = 0.1809;
constexpr double kB = 0.1087;
constexpr double kR = 0.070;
constexpr double kMinScale = 0.45;
constexpr double kDeadband = 0.12;
constexpr double kReference = 1.0472;
constexpr double kMaxWheelAngularSpeed = 12.0;

double alignment_scale(double target, double current)
{
  const double error = std::fabs(normalize_angle(target - current));
  if (error <= kDeadband) {
    return 1.0;
  }
  return std::max(kMinScale, 1.0 - (error / kReference));
}
}  // namespace

int main(int argc, char ** argv)
{
  bool reference_is_commanded = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--reference=commanded") == 0) {
      reference_is_commanded = true;
    } else if (std::strcmp(argv[i], "--reference=ik") == 0) {
      reference_is_commanded = false;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
      return 2;
    }
  }

  const SwerveKinematics model(kA, kB, kR);
  const auto limits = SteeringLimits::from_outward_inward(
                        kDefaultOutwardLimitDeg * M_PI / 180.0,
                        kDefaultInwardLimitDeg * M_PI / 180.0, kDefaultOutwardSign)
                        .in_model_order();

  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream stream(line);
    std::vector<double> values;
    double value = 0.0;
    while (stream >> value) {
      values.push_back(value);
    }
    if (values.empty()) {
      continue;
    }
    if (values.size() != 7 && values.size() != 11) {
      std::fprintf(stderr, "expected 7 or 11 numbers per line, got %zu\n", values.size());
      return 2;
    }

    const BodyTwist twist{values[0], values[1], values[2]};
    std::array<double, kNumWheels> current{};
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      current[i] = values[3 + i];
    }
    const bool override_active = values.size() == 11;
    std::array<double, kNumWheels> override_angles{};
    for (std::size_t i = 0; i < kNumWheels && override_active; ++i) {
      override_angles[i] = values[7 + i];
    }

    const auto limited = limit_twist_to_steering_range(model, twist, current, limits);
    const char * status = limited.status == LimitStatus::kOk             ? "ok"
                          : limited.status == LimitStatus::kOmegaReduced ? "omega_reduced"
                                                                        : "rejected";
    std::printf("%s", status);
    if (limited.status == LimitStatus::kRejected) {
      // Zero drive and the steering holds — no scale is computed, in the
      // controller or here, so the reference mode cannot change this line.
      std::printf("\n");
      continue;
    }
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      const auto & target = (*limited.targets)[i];
      // The one line the whole reference change lives on. Without override
      // columns the two modes pick the same number.
      const double reference_angle = (reference_is_commanded && override_active)
                                       ? override_angles[i]
                                       : target.angle;
      const double scale = alignment_scale(reference_angle, current[i]);
      double speed = (target.speed / kR) * scale;
      speed = std::max(-kMaxWheelAngularSpeed, std::min(kMaxWheelAngularSpeed, speed));
      // The COMMANDED angle is the override when one is active; the IK target
      // otherwise. Printed so a diff shows both halves of a case, not just the
      // speed the scale produced.
      const double commanded_angle = override_active ? override_angles[i] : target.angle;
      std::printf(" %.9f %.9f %.9f", commanded_angle, speed, scale);
    }
    std::printf("\n");
  }
  return 0;
}
