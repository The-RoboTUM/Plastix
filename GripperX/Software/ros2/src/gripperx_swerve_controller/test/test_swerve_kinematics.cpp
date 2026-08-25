// Unit checks for the CONTACT-POINT correction in the swerve inverse kinematics
// (measured 2026-08-21: one commanded nominal revolution in place turned the
// chassis ~270 deg, effective radius 0.26466 m against the 0.210950 m king-pin
// radius the controller was working in).
//
// No ROS in here, same as test_steering_limits.cpp — the whole IK/feasibility
// path is rclcpp-free on purpose.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>

#include "gripperx_swerve_controller/steering_limits.hpp"
#include "gripperx_swerve_controller/swerve_kinematics.hpp"

using gripperx_swerve_controller::BodyTwist;
using gripperx_swerve_controller::equivalent_solutions;
using gripperx_swerve_controller::kDefaultInwardLimitDeg;
using gripperx_swerve_controller::kDefaultOutwardLimitDeg;
using gripperx_swerve_controller::kDefaultOutwardSign;
using gripperx_swerve_controller::kModelToJointIndex;
using gripperx_swerve_controller::kNumWheels;
using gripperx_swerve_controller::normalize_angle;
using gripperx_swerve_controller::resolve_wheel_targets;
using gripperx_swerve_controller::SteeringLimits;
using gripperx_swerve_controller::SwerveKinematics;
using gripperx_swerve_controller::WheelTarget;

namespace
{
constexpr double kDeg = M_PI / 180.0;
constexpr double kA = 0.1809;
constexpr double kB = 0.1087;
constexpr double kR = 0.070;

// ros2_controllers.yaml `wheel_lateral_offset`, JOINT order FL, FR, BL, BR.
constexpr std::array<double, kNumWheels> kOffsetsJointOrder = {
  0.055572, -0.055372, 0.055572, -0.055571};

// The same values in MODEL order (FL, BL, BR, FR) — spelled out rather than
// permuted through kModelToJointIndex, so a wrong permutation there cannot make
// this test agree with itself.
constexpr std::array<double, kNumWheels> kOffsetsModelOrder = {
  0.055572, 0.055572, -0.055571, -0.055372};

SteeringLimits model_limits()
{
  return SteeringLimits::from_outward_inward(
           kDefaultOutwardLimitDeg * kDeg, kDefaultInwardLimitDeg * kDeg, kDefaultOutwardSign)
           .in_model_order();
}

SwerveKinematics corrected_model() { return SwerveKinematics(kA, kB, kR, kOffsetsModelOrder); }

// --- An INDEPENDENT exact rigid-body map -------------------------------------
//
// Deliberately NOT SwerveKinematics::forward_kinematics_body(): it is built from
// the rigid-body definition (contact point = king pin + R(delta) e, velocity =
// v + omega x r) with explicit 2x2 rotations and cross products, and solved by
// Cramer's rule rather than by the production eigen-solver. If the production
// algebra and this disagree, one of them is wrong; if the production code were
// its own oracle, neither could be.
struct Contact
{
  double x{0.0};
  double y{0.0};
};

Contact contact_point(std::size_t i, double delta)
{
  // King pins in model order w1 (a,b) w2 (-a,b) w3 (-a,-b) w4 (a,-b).
  const std::array<std::array<double, 2>, kNumWheels> king_pins{
    {{kA, kB}, {-kA, kB}, {-kA, -kB}, {kA, -kB}}};
  const double ex = 0.0;  // no trail modelled — see the note in inverse_kinematics()
  const double ey = kOffsetsModelOrder[i];
  const double c = std::cos(delta);
  const double s = std::sin(delta);
  return Contact{king_pins[i][0] + (c * ex) - (s * ey), king_pins[i][1] + (s * ex) + (c * ey)};
}

/// Contact-point velocity of wheel i under a body twist, in the body frame.
std::array<double, 2> contact_velocity(std::size_t i, double delta, const BodyTwist & twist)
{
  const Contact p = contact_point(i, delta);
  // v + omega_z_hat x r, with z_hat x (x, y) = (-y, x).
  return {twist.vx - (twist.omega * p.y), twist.vy + (twist.omega * p.x)};
}

/// Recover the body twist from three of the four rolling rows, EXACTLY.
///
/// A determined 3x3 solve by Cramer's rule, not a least-squares fit: with an
/// exact rigid-body motion any three independent rows already pin the twist
/// down, and the fourth row's consistency is checked separately (the pointwise
/// no-slip assertions in the round-trip test). This shares no algorithm with the
/// production Householder QR — it does not even form the same system — so an
/// error in either shows up as a disagreement rather than cancelling out.
///
/// The subset with the largest |det| is used, because at small omega three of
/// the four subsets can be close to singular.
BodyTwist recover_twist(
  const std::array<double, kNumWheels> & angles, const std::array<double, kNumWheels> & speeds)
{
  std::array<std::array<double, 3>, kNumWheels> rows{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const Contact p = contact_point(i, angles[i]);
    const double dx = std::cos(angles[i]);
    const double dy = std::sin(angles[i]);
    // Rolling speed = d . (v + omega * z_hat x p), with z_hat x (x, y) = (-y, x).
    rows[i] = {dx, dy, (-p.y * dx) + (p.x * dy)};
  }

  const auto det3 = [](const std::array<std::array<double, 3>, 3> & a) {
      return (a[0][0] * ((a[1][1] * a[2][2]) - (a[1][2] * a[2][1]))) -
             (a[0][1] * ((a[1][0] * a[2][2]) - (a[1][2] * a[2][0]))) +
             (a[0][2] * ((a[1][0] * a[2][1]) - (a[1][1] * a[2][0])));
    };

  std::array<std::array<double, 3>, 3> best_m{};
  std::array<double, 3> best_rhs{};
  double best_det = 0.0;
  for (std::size_t dropped = 0; dropped < kNumWheels; ++dropped) {
    std::array<std::array<double, 3>, 3> m{};
    std::array<double, 3> rhs{};
    std::size_t r = 0;
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      if (i == dropped) {
        continue;
      }
      m[r] = rows[i];
      rhs[r] = speeds[i];
      ++r;
    }
    const double d = det3(m);
    if (std::fabs(d) > std::fabs(best_det)) {
      best_det = d;
      best_m = m;
      best_rhs = rhs;
    }
  }

  BodyTwist out{};
  double * fields[3] = {&out.vx, &out.vy, &out.omega};
  for (int col = 0; col < 3; ++col) {
    std::array<std::array<double, 3>, 3> n = best_m;
    for (int r = 0; r < 3; ++r) {
      n[static_cast<std::size_t>(r)][static_cast<std::size_t>(col)] =
        best_rhs[static_cast<std::size_t>(r)];
    }
    *fields[col] = det3(n) / best_det;
  }
  return out;
}

/// The chain the controller runs: IK -> per-wheel fold/limit resolution.
std::array<WheelTarget, kNumWheels> resolve(
  const SwerveKinematics & model, const BodyTwist & twist,
  const std::array<double, kNumWheels> & current)
{
  const auto resolved = resolve_wheel_targets(model.inverse_kinematics(twist), current, model_limits());
  EXPECT_TRUE(resolved.has_value());
  return resolved.value_or(std::array<WheelTarget, kNumWheels>{});
}
}  // namespace

// --- 1. BACKWARD COMPATIBILITY -----------------------------------------------

TEST(ContactPointCorrection, ZeroOffsetsAreBitIdenticalToTheKingPinModel)
{
  // The guard on the all-zero default. Not "close to" — the SAME BITS, because
  // an un-migrated config must behave exactly as it does today rather than
  // differently-but-plausibly.
  const SwerveKinematics plain(kA, kB, kR);
  const SwerveKinematics explicit_zero(kA, kB, kR, {{0.0, 0.0, 0.0, 0.0}});
  const std::array<BodyTwist, 7> twists{
    {{0.0, 0.0, 0.0},
      {0.3, 0.0, 0.0},
      {0.0, 0.25, 0.0},
      {0.0, 0.0, 0.75847},
      {0.1, 0.0, 0.3},
      {-0.4, 0.2, -1.1},
      {0.05, 0.05, 0.1}}};
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};

  for (const auto & twist : twists) {
    const auto a = plain.inverse_kinematics(twist);
    const auto b = explicit_zero.inverse_kinematics(twist);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      // What the pre-correction code computed, recomputed here from its own
      // formulas rather than taken from the object under test.
      const std::array<std::array<double, 2>, kNumWheels> king_pins{
        {{kA, kB}, {-kA, kB}, {-kA, -kB}, {kA, -kB}}};
      const double ux = twist.vx - (king_pins[i][1] * twist.omega);
      const double uy = twist.vy + (king_pins[i][0] * twist.omega);
      EXPECT_DOUBLE_EQ(a[i].steering_angle, normalize_angle(std::atan2(uy, ux)));
      EXPECT_DOUBLE_EQ(a[i].linear_speed, std::hypot(ux, uy));
      EXPECT_DOUBLE_EQ(a[i].offset_speed, 0.0);
      EXPECT_DOUBLE_EQ(b[i].steering_angle, a[i].steering_angle);
      EXPECT_DOUBLE_EQ(b[i].linear_speed, a[i].linear_speed);
      EXPECT_DOUBLE_EQ(b[i].offset_speed, 0.0);
    }
    // Some of these twists have no reachable pose at all (that is deliberate —
    // the rejection path must stay bit-identical too).
    const auto plain_targets = resolve_wheel_targets(a, current, model_limits());
    const auto zero_targets = resolve_wheel_targets(b, current, model_limits());
    ASSERT_EQ(plain_targets.has_value(), zero_targets.has_value());
    if (!plain_targets.has_value()) {
      continue;
    }
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      // The pre-correction fold, spelled out.
      const std::array<std::pair<double, double>, 2> old_solutions{
        std::make_pair(a[i].steering_angle, a[i].linear_speed),
        std::make_pair(normalize_angle(a[i].steering_angle + M_PI), -a[i].linear_speed)};
      const bool matches_a = (*plain_targets)[i].speed == old_solutions[0].second;
      const bool matches_b = (*plain_targets)[i].speed == old_solutions[1].second;
      EXPECT_TRUE(matches_a || matches_b) << "wheel " << i;
      EXPECT_DOUBLE_EQ((*zero_targets)[i].angle, (*plain_targets)[i].angle) << "wheel " << i;
      EXPECT_DOUBLE_EQ((*zero_targets)[i].speed, (*plain_targets)[i].speed) << "wheel " << i;
    }
  }
}

TEST(ContactPointCorrection, EquivalentSolutionsDefaultArgumentIsTheOldFunction)
{
  // The property the default argument exists for: every pre-correction caller
  // and every pre-correction unit test keeps its exact result.
  for (const double angle : {-3.0, -1.2, 0.0, 0.4, 2.9}) {
    for (const double speed : {-0.7, 0.0, 0.31}) {
      const auto defaulted = equivalent_solutions(angle, speed);
      const auto zeroed = equivalent_solutions(angle, speed, 0.0);
      EXPECT_DOUBLE_EQ(defaulted[0].first, normalize_angle(angle));
      EXPECT_DOUBLE_EQ(defaulted[0].second, speed);
      EXPECT_DOUBLE_EQ(defaulted[1].first, normalize_angle(normalize_angle(angle) + M_PI));
      EXPECT_DOUBLE_EQ(defaulted[1].second, -speed);
      EXPECT_DOUBLE_EQ(zeroed[0].second, defaulted[0].second);
      EXPECT_DOUBLE_EQ(zeroed[1].second, defaulted[1].second);
    }
  }
}

TEST(ContactPointCorrection, TheParameterPermutationIsTheOneTheControllerApplies)
{
  // SwerveController reads `wheel_lateral_offset` in JOINT order and hands it to
  // the model through to_model_order(), i.e. through kModelToJointIndex. Getting
  // that permutation wrong swaps FR's 0.2 mm asymmetry onto BL and is otherwise
  // invisible — every other test here feeds the model order directly.
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_DOUBLE_EQ(kOffsetsModelOrder[i], kOffsetsJointOrder[kModelToJointIndex[i]])
      << "model wheel " << i;
  }
}

// --- 2. ROUND TRIP -----------------------------------------------------------

TEST(ContactPointCorrection, RoundTripAgainstAnIndependentRigidBodyMap)
{
  const auto model = corrected_model();
  std::mt19937_64 rng(20260821);
  std::uniform_real_distribution<double> linear(-0.6, 0.6);
  std::uniform_real_distribution<double> angular(-2.0, 2.0);
  std::uniform_int_distribution<int> coin(0, 1);

  double worst = 0.0;
  for (int trial = 0; trial < 200; ++trial) {
    const BodyTwist twist{linear(rng), linear(rng), angular(rng)};
    const auto commands = model.inverse_kinematics(twist);

    std::array<double, kNumWheels> angles{};
    std::array<double, kNumWheels> speeds{};
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      // Fold at random: the resolved command must describe the same motion on
      // either branch, which is exactly what fold-invariance means.
      const auto solutions = equivalent_solutions(
        commands[i].steering_angle, commands[i].linear_speed, commands[i].offset_speed);
      const auto & chosen = solutions[coin(rng) == 0 ? 0 : 1];
      angles[i] = chosen.first;
      speeds[i] = chosen.second;

      // Pointwise no-slip, checked before any solving: the commanded speed IS
      // the rolling component of the true contact-point velocity, and the
      // lateral component is zero.
      const auto v = contact_velocity(i, angles[i], twist);
      const double rolling = (v[0] * std::cos(angles[i])) + (v[1] * std::sin(angles[i]));
      const double lateral = (-v[0] * std::sin(angles[i])) + (v[1] * std::cos(angles[i]));
      EXPECT_NEAR(rolling, speeds[i], 1e-13) << "trial " << trial << " wheel " << i;
      EXPECT_NEAR(lateral, 0.0, 1e-13) << "trial " << trial << " wheel " << i;
    }

    const BodyTwist recovered = recover_twist(angles, speeds);
    worst = std::max(
      worst, std::max(
        std::fabs(recovered.vx - twist.vx),
        std::max(std::fabs(recovered.vy - twist.vy), std::fabs(recovered.omega - twist.omega))));
    EXPECT_NEAR(recovered.vx, twist.vx, 1e-12) << "trial " << trial;
    EXPECT_NEAR(recovered.vy, twist.vy, 1e-12) << "trial " << trial;
    EXPECT_NEAR(recovered.omega, twist.omega, 1e-12) << "trial " << trial;

    // And the production forward map must agree with the independent one.
    const BodyTwist production = model.forward_kinematics_body(angles, speeds);
    EXPECT_NEAR(production.vx, twist.vx, 1e-12) << "trial " << trial;
    EXPECT_NEAR(production.vy, twist.vy, 1e-12) << "trial " << trial;
    EXPECT_NEAR(production.omega, twist.omega, 1e-12) << "trial " << trial;
  }
  RecordProperty("worst_absolute_error", std::to_string(worst));
}

// --- 3. THE FOLD ------------------------------------------------------------

TEST(ContactPointCorrection, FoldedWheelsGetMinusOmegaTimesHNotPlusOmegaTimesH)
{
  // THE test this change exists to be guarded by. Putting the correction inside
  // inverse_kinematics() (so that linear_speed = s - omega*h) passes every other
  // test in this file and fails this one: equivalent_solutions() would then
  // negate the correction along with s and return -s + omega*h on the folded
  // branch. In an in-place spin FL and BL are EXACTLY the folded wheels, so that
  // version drives half the machine at 0.1554 m/s instead of 0.2665 m/s, breaks
  // the shared instantaneous centre and scrubs — strictly worse than no
  // correction at all.
  const auto model = corrected_model();
  const double omega = 0.75847;
  const BodyTwist spin{0.0, 0.0, omega};
  const auto commands = model.inverse_kinematics(spin);
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const auto targets = resolve(model, spin, current);

  // Model order FL, BL, BR, FR: FL and BL are the folded pair here.
  const std::array<bool, kNumWheels> expect_folded{true, true, false, false};
  // The king-pin speed, computed HERE from the geometry rather than read out of
  // commands[i].linear_speed — so that an implementation which folds the
  // correction into linear_speed cannot make this test agree with it.
  const double s = omega * std::hypot(kA, kB);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const double correction = omega * kOffsetsModelOrder[i];
    EXPECT_DOUBLE_EQ(commands[i].offset_speed, correction) << "wheel " << i;

    const double folded_delta =
      std::fabs(normalize_angle(targets[i].angle - commands[i].steering_angle));
    const bool folded = folded_delta > 1.0;
    EXPECT_EQ(folded, expect_folded[i]) << "wheel " << i;

    const double right = folded ? (-s - correction) : (s - correction);
    const double wrong = folded ? (-s + correction) : (s - correction);
    EXPECT_NEAR(targets[i].speed, right, 1e-12) << "wheel " << i;
    if (folded) {
      // Named explicitly so the failure message says what went wrong.
      EXPECT_GT(std::fabs(targets[i].speed - wrong), 1e-3)
        << "wheel " << i << " carries +omega*h: the correction was folded";
      EXPECT_GT(std::fabs(targets[i].speed), s)
        << "wheel " << i << " is SLOWER than the king-pin speed; the correction points inward";
    }
  }
}

// --- 4. SPIN-POSE REGRESSION -------------------------------------------------

TEST(ContactPointCorrection, SpinPoseRegressionAtTheMeasuredOmega)
{
  // omega = 0.75847 rad/s is the commanded nominal revolution of the 2026-08-21
  // hardware run. Angles must not move; speeds must go 2.2857 -> 2.8878 rad/s.
  const auto model = corrected_model();
  const SwerveKinematics king_pin(kA, kB, kR);
  const BodyTwist spin{0.0, 0.0, 0.75847};
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const auto targets = resolve(model, spin, current);
  const auto before = resolve(king_pin, spin, current);

  // Model order FL, BL, BR, FR.
  const double kSpinPoseDeg = std::atan2(kA, kB) / kDeg;
  const std::array<double, kNumWheels> expected_deg{
    -kSpinPoseDeg, kSpinPoseDeg, -kSpinPoseDeg, kSpinPoseDeg};
  // GEOMETRY-DERIVED, regenerated 2026-08-24 for a = 0.1809 / b = 0.1087.
  // Not independent evidence: only the 0.266571 below is measured.
  const std::array<double, kNumWheels> expected_rad_s{
    -2.888884, -2.888884, 2.888874, 2.886717};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_NEAR(targets[i].angle / kDeg, expected_deg[i], 1e-3) << "wheel " << i;
    // BIT-IDENTICAL to the uncorrected model, not merely close: the angle is the
    // king-pin answer and the correction must not have touched it.
    EXPECT_DOUBLE_EQ(targets[i].angle, before[i].angle) << "wheel " << i;
    EXPECT_NEAR(targets[i].speed / kR, expected_rad_s[i], 5e-5) << "wheel " << i;
    // Sign pattern unchanged against today's machine.
    EXPECT_EQ(targets[i].speed > 0.0, before[i].speed > 0.0) << "wheel " << i;
    // And what the run actually measured: 0.2666 m of arc per radian, not 0.21095.
    EXPECT_NEAR(std::fabs(targets[i].speed) / spin.omega, 0.266571, 3e-4) << "wheel " << i;
    EXPECT_NEAR(std::fabs(before[i].speed) / spin.omega, std::hypot(kA, kB), 1e-6) << "wheel " << i;
  }
}

TEST(ContactPointCorrection, SpinPoseFollowsTheDeclaredGeometryNotTheRetiredOne)
{
  // The number three comment blocks used to give as +-50.7. It belongs to the
  // retired b = 0.16556 geometry; with a = 0.180 / b = 0.110 the pose is
  // atan2(a, b) = 58.570 deg, which is 8 deg less steering margin than a reader
  // of the old number would compute.
  // Retired geometry b = 0.16556 gave 50.80 deg; the tape pair
  // 0.180 / 0.110 gave 58.57; the CAD pair 0.1809 / 0.1087 gives 59.00.
  // Asserted as a RANGE against the retired value rather than as a new
  // literal, so the next legitimate geometry change does not have to
  // edit this test at all.
  EXPECT_GT(std::atan2(kA, kB) / kDeg, 55.0);
  EXPECT_LT(std::atan2(kA, kB) / kDeg, 62.0);
}

// --- 5. CRAB / omega = 0 -----------------------------------------------------

TEST(ContactPointCorrection, AtZeroOmegaTheCorrectionIsIdenticallyZero)
{
  // The correction is omega*h. At omega = 0 there is no rotation, the contact
  // point and the king pin translate identically, and the correction must
  // vanish EXACTLY — at every steering angle, crab included.
  const auto model = corrected_model();
  const std::array<double, kNumWheels> current{0.0, 0.0, 0.0, 0.0};
  const SwerveKinematics king_pin(kA, kB, kR);

  for (double heading_deg = -180.0; heading_deg <= 180.0; heading_deg += 5.0) {
    const double heading = heading_deg * kDeg;
    const BodyTwist twist{0.4 * std::cos(heading), 0.4 * std::sin(heading), 0.0};
    const auto commands = model.inverse_kinematics(twist);
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_DOUBLE_EQ(commands[i].offset_speed, 0.0) << "heading " << heading_deg;
    }
    const auto corrected = resolve_wheel_targets(commands, current, model_limits());
    const auto plain =
      resolve_wheel_targets(king_pin.inverse_kinematics(twist), current, model_limits());
    ASSERT_EQ(corrected.has_value(), plain.has_value()) << "heading " << heading_deg;
    if (!corrected.has_value()) {
      continue;
    }
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      EXPECT_DOUBLE_EQ((*corrected)[i].angle, (*plain)[i].angle) << "heading " << heading_deg;
      EXPECT_DOUBLE_EQ((*corrected)[i].speed, (*plain)[i].speed) << "heading " << heading_deg;
    }
  }

  // The +-90 deg crab pose explicitly, since it is the one that reaches its
  // angle through the module fold.
  const auto crab = resolve(model, BodyTwist{0.0, 0.25, 0.0}, current);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    EXPECT_NEAR(std::fabs(crab[i].angle), 90.0 * kDeg, 1e-12) << "wheel " << i;
    EXPECT_NEAR(std::fabs(crab[i].speed), 0.25, 1e-12) << "wheel " << i;
  }
}
