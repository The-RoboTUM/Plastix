// Per-wheel, per-direction steering limits — C++ port of
// gripperx_control/steering_limits.py.
//
// SAFETY-RELEVANT: this is FR-5 / SR-6, not an optimisation (§3.1.4 (a) item 1).
// The steering range of this robot is asymmetric AND per wheel: ~100 deg
// OUTWARD, ~35 deg INWARD, and which SIGN of the joint angle is outward differs
// per wheel and was MEASURED, not derived:
//
//     steering_outward_sign = (-1, +1, +1, -1)   // FL, FR, BL, BR
//
// giving the robot-frame windows
//
//     FL [-100, +35]   FR [-35, +100]   BL [-35, +100]   BR [-100, +35]  (deg)
//
// Do NOT re-derive the sign from the URDF: a URDF-only reading gives
// (+1, -1, +1, -1), is wrong on the front pair, and under it the in-place spin
// pose (FL -58.570, FR +58.570, BL +58.570, BR -58.570 deg) would be unreachable.
//
// CORRECTED 2026-08-21: this line read +-50.7 deg. That number belongs to the
// RETIRED b = 0.16556 geometry; with the active a = 0.180 / b = 0.110 the spin
// pose is atan2(a, b) = 58.570 deg. It matters because it is the number a reader
// uses to compute the remaining steering margin against the 100 deg outward
// stop — 41.4 deg, not 49.3 deg.
//
// SOURCE OF TRUTH for the numbers is gripperx_control/config/steer_servo.yaml,
// whose clamp inside steer_servo_node is the last line of defence and stays so.
// Everything upstream must PLAN inside the same window, otherwise the servo node
// silently clamps a wheel, the pose no longer matches any single instantaneous
// centre of rotation, and the wheels scrub against each other.
//
// The defaults below mirror that file (inward 35.0, raised from 30.0 in commit
// a29e181; the 35 deg value itself is still TO-VERIFY — the tightest measured
// inward stop is BL at 35.60 deg, so it leaves only ~0.6 deg of margin).
// Runtime values always come from parameters so the two files can be aligned
// without a rebuild.
//
// This header intentionally depends on nothing from rclcpp, so the whole
// feasibility path can be unit-checked without ROS.

#ifndef GRIPPERX_SWERVE_CONTROLLER__STEERING_LIMITS_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__STEERING_LIMITS_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "gripperx_swerve_controller/swerve_kinematics.hpp"

namespace gripperx_swerve_controller
{

/// Joint order used by steer_servo.yaml / steer_servo_node / the command interfaces.
inline constexpr std::array<const char *, kNumWheels> kJointOrderLabels = {"FL", "FR", "BL", "BR"};
/// Paper/kinematic-model wheel order (w1..w4 in Lee 2015).
inline constexpr std::array<const char *, kNumWheels> kModelOrderLabels = {"FL", "BL", "BR", "FR"};
/// Index of each model-order wheel inside a joint-order array.
inline constexpr std::array<std::size_t, kNumWheels> kModelToJointIndex = {0, 2, 3, 1};
/// Index of each joint-order wheel inside a model-order array (inverse of the above).
inline constexpr std::array<std::size_t, kNumWheels> kJointToModelIndex = {0, 3, 1, 2};

inline constexpr double kDefaultOutwardLimitDeg = 100.0;
inline constexpr double kDefaultInwardLimitDeg = 35.0;
inline constexpr std::array<int, kNumWheels> kDefaultOutwardSign = {-1, 1, 1, -1};

/// Floating-point slack when testing "is this angle inside the window".
inline constexpr double kAngleToleranceRad = 1e-6;

/// Reachable joint-angle window per wheel, in one fixed wheel order.
class SteeringLimits
{
public:
  SteeringLimits() = default;

  /// Build the joint-order window from the steer_servo.yaml schema.
  /// `outward_rad`/`inward_rad` are MAGNITUDES; `outward_sign[i]` says which
  /// sign of wheel i's joint angle is physically outward.
  static SteeringLimits from_outward_inward(
    double outward_rad, double inward_rad, const std::array<int, kNumWheels> & outward_sign,
    const std::array<const char *, kNumWheels> & labels = kJointOrderLabels);

  /// Same windows, reordered from joint order to Lee-2015 wheel order.
  SteeringLimits in_model_order() const;

  bool contains(std::size_t index, double angle) const;
  double clamp(std::size_t index, double angle) const;

  double lower(std::size_t index) const { return lower_[index]; }
  double upper(std::size_t index) const { return upper_[index]; }
  const char * label(std::size_t index) const { return labels_[index]; }

  std::string describe() const;

private:
  std::array<double, kNumWheels> lower_{};
  std::array<double, kNumWheels> upper_{};
  std::array<const char *, kNumWheels> labels_{kJointOrderLabels};
};

/// One module's resolved command: steering angle and signed linear speed.
struct WheelTarget
{
  double angle{0.0};
  double speed{0.0};
};

struct LimitViolation
{
  std::size_t index{0};
  std::string label;
  double requested{0.0};
  double lower{0.0};
  double upper{0.0};

  std::string describe() const;
};

enum class LimitStatus
{
  kOk,
  kOmegaReduced,
  kRejected,
};

struct LimitedTwist
{
  LimitStatus status{LimitStatus::kOk};
  BodyTwist twist{};
  std::optional<std::array<WheelTarget, kNumWheels>> targets;
  double requested_omega{0.0};
  std::vector<LimitViolation> violations;
};

/// The two module solutions that realise the same wheel velocity vector.
///
/// `offset_speed` is WheelCommand::offset_speed = omega * h_i, the king-pin ->
/// contact-point speed correction. It is SUBTRACTED FROM BOTH branches with the
/// same sign, because  z_hat x R(delta) e  contributes -h_i along the rolling
/// direction independently of delta — and the fold changes delta by 180 deg, so
/// the correction is fold-INVARIANT while the king-pin speed is not:
///
///     branch A:  (delta,          +s - omega*h_i)
///     branch B:  (delta + 180deg, -s - omega*h_i)
///
/// Negating the correction along with s (which is what folding a pre-corrected
/// speed would do) is wrong on every folded wheel, and in an in-place spin FL
/// and BL are exactly the folded ones.
///
/// The default of 0.0 is what keeps every pre-correction caller — and every
/// pre-correction unit test — byte-identical.
std::array<std::pair<double, double>, 2> equivalent_solutions(
  double steering_angle, double linear_speed, double offset_speed = 0.0);

/// Pick a reachable module solution per wheel, or nullopt if one wheel has none.
///
/// Of the two equivalent solutions (angle, +v) / (angle+180, -v) the one closest
/// to the module's CURRENT angle is preferred — but only among the solutions
/// inside that wheel's window. With asymmetric limits the nearer solution is not
/// always the reachable one, so limit awareness happens HERE and not in a later
/// clamp.
std::optional<std::array<WheelTarget, kNumWheels>> resolve_wheel_targets(
  const std::array<WheelCommand, kNumWheels> & wheel_commands,
  const std::array<double, kNumWheels> & current_angles, const SteeringLimits & limits);

/// What the IK would ask for if there were no limits (diagnostics only).
std::array<WheelTarget, kNumWheels> unconstrained_targets(
  const std::array<WheelCommand, kNumWheels> & wheel_commands,
  const std::array<double, kNumWheels> & current_angles);

std::vector<LimitViolation> find_violations(
  const std::array<WheelTarget, kNumWheels> & targets, const SteeringLimits & limits);

/// Make a requested body twist reachable by reducing |omega| only.
///
/// Three options existed for a pose that violates a limit (decision 2026-08-13):
///  * clamp per wheel — the four wheels stop sharing one instantaneous centre of
///    rotation, so they fight each other and scrub;
///  * scale the whole twist — provably useless, delta_i = atan2(vy_i, vx_i) is
///    INVARIANT under uniform scaling of (vx, vy, omega);
///  * reduce the RATIO omega/(vx, vy) — this is what actually rotates the wheel
///    angles, because it moves the instantaneous centre outward.
/// The third is taken: keep (vx, vy) exactly as commanded and shrink |omega|
/// until every wheel is inside its own window. If even omega = 0 is unreachable
/// the requested DIRECTION OF TRAVEL itself cannot be steered to, and the result
/// is REJECTED — the caller must not move, and must say so.
LimitedTwist limit_twist_to_steering_range(
  const SwerveKinematics & model, const BodyTwist & body_twist,
  const std::array<double, kNumWheels> & current_angles, const SteeringLimits & limits,
  int iterations = 16);

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__STEERING_LIMITS_HPP_
