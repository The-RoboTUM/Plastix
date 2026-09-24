// Unit checks for the ported FR-5 / SR-6 machinery. No ROS in here on purpose —
// the same property the Python module has, and the reason both are testable at
// all. The numeric equivalence against the Python original is checked
// separately, twist by twist, by test/dump_reference.cpp vs.
// test/dump_reference.py.

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "gripperx_swerve_controller/steering_limits.hpp"
#include "gripperx_swerve_controller/swerve_kinematics.hpp"

using gripperx_swerve_controller::BodyTwist;
using gripperx_swerve_controller::kDefaultInwardLimitDeg;
using gripperx_swerve_controller::kDefaultOutwardLimitDeg;
using gripperx_swerve_controller::kDefaultOutwardSign;
using gripperx_swerve_controller::kNumWheels;
using gripperx_swerve_controller::LimitStatus;
using gripperx_swerve_controller::limit_twist_to_steering_range;
using gripperx_swerve_controller::SteeringLimits;
using gripperx_swerve_controller::SwerveKinematics;

namespace
{
constexpr double kDeg = M_PI / 180.0;

SteeringLimits default_joint_limits()
{
  return SteeringLimits::from_outward_inward(
    kDefaultOutwardLimitDeg * kDeg, kDefaultInwardLimitDeg * kDeg, kDefaultOutwardSign);
}
}  // namespace

TEST(SteeringLimits, JointOrderWindowsMatchTheMeasuredMachine)
{
  const auto limits = default_joint_limits();
  // FL [-125, +35]  FR [-35, +125]  BL [-35, +125]  BR [-125, +35]
  // Literals are spelled out rather than derived from kDefaultOutwardLimitDeg on
  // purpose: a test that reads the constant it is checking would pass for any
  // value.
  EXPECT_NEAR(limits.lower(0), -125.0 * kDeg, 1e-12);
  EXPECT_NEAR(limits.upper(0), 35.0 * kDeg, 1e-12);
  EXPECT_NEAR(limits.lower(1), -35.0 * kDeg, 1e-12);
  EXPECT_NEAR(limits.upper(1), 125.0 * kDeg, 1e-12);
  EXPECT_NEAR(limits.lower(2), -35.0 * kDeg, 1e-12);
  EXPECT_NEAR(limits.upper(2), 125.0 * kDeg, 1e-12);
  EXPECT_NEAR(limits.lower(3), -125.0 * kDeg, 1e-12);
  EXPECT_NEAR(limits.upper(3), 35.0 * kDeg, 1e-12);
}

TEST(SteeringLimits, InwardLimitIs35Not30)
{
  // Must stay 35.0, not 30.0: porting 30.0 would throw away 5 deg of inward
  // travel on every wheel and would be a FAILED port disguised as a faithful
  // one (NFR-10 acceptance 3a).
  EXPECT_DOUBLE_EQ(kDefaultInwardLimitDeg, 35.0);
}

TEST(SteeringLimits, ModelOrderIsTheLee2015Permutation)
{
  const auto joint = default_joint_limits();
  const auto model = joint.in_model_order();
  // model order FL, BL, BR, FR  <-  joint order FL, FR, BL, BR
  EXPECT_NEAR(model.lower(0), joint.lower(0), 1e-12);  // FL
  EXPECT_NEAR(model.lower(1), joint.lower(2), 1e-12);  // BL
  EXPECT_NEAR(model.lower(2), joint.lower(3), 1e-12);  // BR
  EXPECT_NEAR(model.lower(3), joint.lower(1), 1e-12);  // FR
}

TEST(SteeringLimits, InPlaceSpinPoseIsReachableUnderTheMeasuredSign)
{
  // Sanity check the Python module documents and demands: the spin pose must be
  // reachable on all four wheels. Under the refuted URDF-derived sign
  // (+1, -1, +1, -1) it would need >50 deg INWARD on three wheels and spin
  // would be impossible.
  const auto limits = default_joint_limits();
  const std::array<double, kNumWheels> spin{-50.7 * kDeg, 50.7 * kDeg, 50.7 * kDeg, -50.7 * kDeg};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_TRUE(limits.contains(i, spin[i])) << "wheel " << i;
  }
}

TEST(SteeringLimits, PureTranslationIsAlwaysOk)
{
  const SwerveKinematics model(0.1809, 0.1087, 0.070);
  const auto limits = default_joint_limits().in_model_order();
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const auto result =
    limit_twist_to_steering_range(model, BodyTwist{0.3, 0.0, 0.0}, current, limits);
  EXPECT_EQ(result.status, LimitStatus::kOk);
  ASSERT_TRUE(result.targets.has_value());
  for (const auto & target : *result.targets) {
    EXPECT_NEAR(target.angle, 0.0, 1e-9);
    EXPECT_NEAR(target.speed, 0.3, 1e-9);
  }
}

TEST(SteeringLimits, PureCrabResolvesViaTheModuleFlipAndIsNotRejected)
{
  // FR-7: pure sideways travel is reachable at -+90 deg via the +-180 module
  // flip. It must NOT land in the REJECTED bucket.
  const SwerveKinematics model(0.1809, 0.1087, 0.070);
  const auto limits = default_joint_limits().in_model_order();
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const auto result =
    limit_twist_to_steering_range(model, BodyTwist{0.0, 0.25, 0.0}, current, limits);
  EXPECT_EQ(result.status, LimitStatus::kOk);
}

TEST(SteeringLimits, TightCorneringReducesOmegaRatherThanClamping)
{
  // The expected value is NOT guessed: it is what the live Python chain
  // produces for this twist (gripperx_control.steering_limits), so this test
  // doubles as a pinned equivalence case.
  //
  // What this case also shows about the search: at vx = 0.1 an omega of 0.3 has
  // to be reduced, while an omega of 3.0 passes untouched. The reachable set
  // falls apart into two arcs because of the +-180 module flip, so the
  // bisection is CONSERVATIVE rather than monotone — documented in
  // limit_twist_to_steering_range and reproduced faithfully here.
  const SwerveKinematics model(0.1809, 0.1087, 0.070);
  const auto limits = default_joint_limits().in_model_order();
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const auto result =
    limit_twist_to_steering_range(model, BodyTwist{0.1, 0.0, 0.3}, current, limits);
  EXPECT_EQ(result.status, LimitStatus::kOmegaReduced);
  ASSERT_TRUE(result.targets.has_value());
  EXPECT_NEAR(result.twist.omega, 0.2724380493164062, 1e-12);
  EXPECT_DOUBLE_EQ(result.twist.vx, 0.1);  // (vx, vy) are kept exactly as commanded
  EXPECT_DOUBLE_EQ(result.twist.vy, 0.0);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_TRUE(limits.contains(i, (*result.targets)[i].angle)) << "wheel " << i;
  }
}

TEST(SteeringLimits, SteepDiagonalIsRejectedAndReportsViolations)
{
  // A direction of travel that no module solution fits, under the calibrated
  // window (125/35 deg). Rejected, with the violations available so the
  // rejection can be LOGGED and never silent — silent clamping is the failure
  // mode the machinery exists to remove (§3.1.4 (a) item 2). The binding
  // constraint on a steep diagonal is the INWARD limit, unchanged at 35 deg —
  // see the test below for the same twist family resolving once the OUTWARD
  // limit is wide enough.
  const SwerveKinematics model(0.1809, 0.1087, 0.070);
  const auto limits = default_joint_limits().in_model_order();
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const auto result =
    limit_twist_to_steering_range(model, BodyTwist{0.1, 0.1, 0.1}, current, limits);
  EXPECT_EQ(result.status, LimitStatus::kRejected);
  EXPECT_FALSE(result.targets.has_value());
  EXPECT_FALSE(result.violations.empty());
}

TEST(SteeringLimits, TheWiderOutwardWindowUnlocksADiagonalThatWasRejectedBefore)
{
  // {0.05, 0.05, 0.1} is an FR-8 diagonal with no module solution under a
  // tighter outward window; at the current 125 deg limit it resolves cleanly,
  // every wheel inside its own window. If this ever starts failing, the outward
  // limit has been lowered somewhere and the machine has quietly lost a
  // manoeuvre.
  const SwerveKinematics model(0.1809, 0.1087, 0.070);
  const auto limits = default_joint_limits().in_model_order();
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const auto result =
    limit_twist_to_steering_range(model, BodyTwist{0.05, 0.05, 0.1}, current, limits);
  EXPECT_EQ(result.status, LimitStatus::kOk);
  ASSERT_TRUE(result.targets.has_value());
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_TRUE(limits.contains(i, (*result.targets)[i].angle)) << "wheel " << i;
  }
}

TEST(SwerveKinematics, ZeroTwistResolvesToCentreWhichIsWhyStage2Exists)
{
  // atan2(0, 0) == 0.0 on every wheel. This is the defect OP-24 stage 2 closes:
  // the value is correct kinematics and a CENTRE command at the actuators, so
  // the controller must never let it reach the steering command interfaces.
  const SwerveKinematics model(0.1809, 0.1087, 0.070);
  const auto commands = model.inverse_kinematics(BodyTwist{0.0, 0.0, 0.0});
  for (const auto & command : commands) {
    EXPECT_DOUBLE_EQ(command.steering_angle, 0.0);
    EXPECT_DOUBLE_EQ(command.linear_speed, 0.0);
  }
}
