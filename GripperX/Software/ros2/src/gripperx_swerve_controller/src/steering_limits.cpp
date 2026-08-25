#include "gripperx_swerve_controller/steering_limits.hpp"

#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <utility>

namespace gripperx_swerve_controller
{

namespace
{
constexpr double kRadToDeg = 180.0 / M_PI;
}  // namespace

SteeringLimits SteeringLimits::from_outward_inward(
  double outward_rad, double inward_rad, const std::array<int, kNumWheels> & outward_sign,
  const std::array<const char *, kNumWheels> & labels)
{
  if (outward_rad <= 0.0 || inward_rad <= 0.0) {
    throw std::invalid_argument("outward/inward steering limits are magnitudes and must be > 0");
  }
  for (const int sign : outward_sign) {
    if (sign != -1 && sign != 1) {
      throw std::invalid_argument("outward_sign values must be +1 or -1");
    }
  }

  SteeringLimits limits;
  limits.labels_ = labels;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    if (outward_sign[i] > 0) {
      limits.lower_[i] = -inward_rad;
      limits.upper_[i] = outward_rad;
    } else {
      limits.lower_[i] = -outward_rad;
      limits.upper_[i] = inward_rad;
    }
  }
  return limits;
}

SteeringLimits SteeringLimits::in_model_order() const
{
  SteeringLimits reordered;
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    reordered.lower_[i] = lower_[kModelToJointIndex[i]];
    reordered.upper_[i] = upper_[kModelToJointIndex[i]];
    reordered.labels_[i] = kModelOrderLabels[i];
  }
  return reordered;
}

bool SteeringLimits::contains(std::size_t index, double angle) const
{
  return (lower_[index] - kAngleToleranceRad) <= angle &&
         angle <= (upper_[index] + kAngleToleranceRad);
}

double SteeringLimits::clamp(std::size_t index, double angle) const
{
  return std::max(lower_[index], std::min(upper_[index], angle));
}

std::string SteeringLimits::describe() const
{
  std::string out;
  char buffer[64];
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    std::snprintf(
      buffer, sizeof(buffer), "%s[%+.0f,%+.0f]", labels_[i], lower_[i] * kRadToDeg,
      upper_[i] * kRadToDeg);
    if (i != 0) {
      out += " ";
    }
    out += buffer;
  }
  return out;
}

std::string LimitViolation::describe() const
{
  char buffer[192];
  std::snprintf(
    buffer, sizeof(buffer), "%s needs %+.1fdeg, window [%+.1f, %+.1f]deg", label.c_str(),
    requested * kRadToDeg, lower * kRadToDeg, upper * kRadToDeg);
  return std::string(buffer);
}

std::array<std::pair<double, double>, 2> equivalent_solutions(
  double steering_angle, double linear_speed, double offset_speed)
{
  const double angle = normalize_angle(steering_angle);
  // offset_speed is subtracted with the SAME sign on both branches — see the
  // header. It is applied here, POST-fold, and nowhere upstream.
  return {
    std::make_pair(angle, linear_speed - offset_speed),
    std::make_pair(normalize_angle(angle + M_PI), -linear_speed - offset_speed)};
}

std::optional<std::array<WheelTarget, kNumWheels>> resolve_wheel_targets(
  const std::array<WheelCommand, kNumWheels> & wheel_commands,
  const std::array<double, kNumWheels> & current_angles, const SteeringLimits & limits)
{
  std::array<WheelTarget, kNumWheels> targets{};
  for (std::size_t index = 0; index < kNumWheels; ++index) {
    bool have_best = false;
    double best_distance = 0.0;
    WheelTarget best{};
    for (const auto & solution :
         equivalent_solutions(
           wheel_commands[index].steering_angle, wheel_commands[index].linear_speed,
           wheel_commands[index].offset_speed))
    {
      const double angle = solution.first;
      const double speed = solution.second;
      if (!limits.contains(index, angle)) {
        continue;
      }
      const double distance = std::fabs(normalize_angle(angle - current_angles[index]));
      if (!have_best || distance < best_distance) {
        have_best = true;
        best_distance = distance;
        best = WheelTarget{limits.clamp(index, angle), speed};
      }
    }
    if (!have_best) {
      return std::nullopt;
    }
    targets[index] = best;
  }
  return targets;
}

std::array<WheelTarget, kNumWheels> unconstrained_targets(
  const std::array<WheelCommand, kNumWheels> & wheel_commands,
  const std::array<double, kNumWheels> & current_angles)
{
  std::array<WheelTarget, kNumWheels> targets{};
  for (std::size_t index = 0; index < kNumWheels; ++index) {
    const auto solutions = equivalent_solutions(
      wheel_commands[index].steering_angle, wheel_commands[index].linear_speed,
      wheel_commands[index].offset_speed);
    const double d0 = std::fabs(normalize_angle(solutions[0].first - current_angles[index]));
    const double d1 = std::fabs(normalize_angle(solutions[1].first - current_angles[index]));
    // Python's min() over the solution list keeps the FIRST minimum on a tie.
    const auto & chosen = (d1 < d0) ? solutions[1] : solutions[0];
    targets[index] = WheelTarget{chosen.first, chosen.second};
  }
  return targets;
}

std::vector<LimitViolation> find_violations(
  const std::array<WheelTarget, kNumWheels> & targets, const SteeringLimits & limits)
{
  std::vector<LimitViolation> violations;
  for (std::size_t index = 0; index < kNumWheels; ++index) {
    if (!limits.contains(index, targets[index].angle)) {
      violations.push_back(
        LimitViolation{
          index, limits.label(index), targets[index].angle, limits.lower(index),
          limits.upper(index)});
    }
  }
  return violations;
}

LimitedTwist limit_twist_to_steering_range(
  const SwerveKinematics & model, const BodyTwist & body_twist,
  const std::array<double, kNumWheels> & current_angles, const SteeringLimits & limits,
  int iterations)
{
  LimitedTwist result;
  result.requested_omega = body_twist.omega;

  auto targets =
    resolve_wheel_targets(model.inverse_kinematics(body_twist), current_angles, limits);
  if (targets.has_value()) {
    result.status = LimitStatus::kOk;
    result.twist = body_twist;
    result.targets = targets;
    return result;
  }

  result.violations = find_violations(
    unconstrained_targets(model.inverse_kinematics(body_twist), current_angles), limits);

  const BodyTwist zero_omega_twist{body_twist.vx, body_twist.vy, 0.0};
  auto zero_omega_targets =
    resolve_wheel_targets(model.inverse_kinematics(zero_omega_twist), current_angles, limits);
  if (!zero_omega_targets.has_value()) {
    result.status = LimitStatus::kRejected;
    result.targets = std::nullopt;
    return result;
  }

  // Monotonicity: for fixed (vx, vy) the velocity vector of each wheel travels
  // along a straight line in the velocity plane as omega varies, so each wheel
  // angle sweeps monotonically and this bisection converges on the real
  // boundary. Where the +-180 module flip breaks the reachable set into two
  // arcs the result is CONSERVATIVE (a feasible omega possibly smaller than the
  // true maximum), never optimistic.
  double low = 0.0;
  double high = 1.0;
  BodyTwist best_twist = zero_omega_twist;
  auto best_targets = zero_omega_targets;
  const int loops = std::max(1, iterations);
  for (int i = 0; i < loops; ++i) {
    const double mid = 0.5 * (low + high);
    const BodyTwist candidate{body_twist.vx, body_twist.vy, mid * result.requested_omega};
    auto candidate_targets =
      resolve_wheel_targets(model.inverse_kinematics(candidate), current_angles, limits);
    if (!candidate_targets.has_value()) {
      high = mid;
    } else {
      low = mid;
      best_twist = candidate;
      best_targets = candidate_targets;
    }
  }

  result.status = LimitStatus::kOmegaReduced;
  result.twist = best_twist;
  result.targets = best_targets;
  return result;
}

}  // namespace gripperx_swerve_controller
