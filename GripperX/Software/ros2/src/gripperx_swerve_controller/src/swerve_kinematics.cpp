#include "gripperx_swerve_controller/swerve_kinematics.hpp"

#include <algorithm>
#include <stdexcept>

namespace gripperx_swerve_controller
{

namespace
{

/// Least-squares solve of the four rolling rows for (vx, vy, omega), by
/// Householder QR on the 4x3 system itself. Written out rather than pulled from
/// Eigen so this library keeps its only-the-standard-library property.
///
/// QR ON THE SYSTEM, NOT CHOLESKY ON THE NORMAL EQUATIONS, and the difference is
/// measurable rather than academic. The four rows go RANK DEFICIENT in a real
/// pose: at omega = 0 all four steering angles are equal, the vx and vy columns
/// become collinear, and the two are genuinely not separable from wheel speeds
/// alone. Near that pose the conditioning degrades like 1/omega, and forming
/// A^T A squares it — measured on the round-trip test, normal equations lose
/// 1e-11 at omega = 0.003 where QR loses 1e-13.
///
/// A vanishing diagonal of R means that unknown is not observable in this pose;
/// it is set to zero and the remaining ones are solved around it. omega is never
/// the unobservable one: its column is the +-(y_i + h_i) pattern, which stays
/// orthogonal to the constant vx/vy columns.
std::array<double, 3> solve_least_squares_4x3(
  std::array<std::array<double, 3>, kNumWheels> a, std::array<double, kNumWheels> b)
{
  constexpr std::size_t kRows = kNumWheels;
  for (std::size_t k = 0; k < 3; ++k) {
    double norm = 0.0;
    for (std::size_t i = k; i < kRows; ++i) {
      norm += a[i][k] * a[i][k];
    }
    norm = std::sqrt(norm);
    if (norm == 0.0) {
      continue;
    }
    const double alpha = (a[k][k] >= 0.0) ? -norm : norm;
    std::array<double, kRows> v{};
    for (std::size_t i = k; i < kRows; ++i) {
      v[i] = a[i][k];
    }
    v[k] -= alpha;
    double v_norm_sq = 0.0;
    for (std::size_t i = k; i < kRows; ++i) {
      v_norm_sq += v[i] * v[i];
    }
    if (v_norm_sq == 0.0) {
      continue;
    }
    for (std::size_t j = k; j < 3; ++j) {
      double dot = 0.0;
      for (std::size_t i = k; i < kRows; ++i) {
        dot += v[i] * a[i][j];
      }
      const double scale = 2.0 * dot / v_norm_sq;
      for (std::size_t i = k; i < kRows; ++i) {
        a[i][j] -= scale * v[i];
      }
    }
    double dot = 0.0;
    for (std::size_t i = k; i < kRows; ++i) {
      dot += v[i] * b[i];
    }
    const double scale = 2.0 * dot / v_norm_sq;
    for (std::size_t i = k; i < kRows; ++i) {
      b[i] -= scale * v[i];
    }
  }

  double pivot_scale = 0.0;
  for (std::size_t k = 0; k < 3; ++k) {
    pivot_scale = std::max(pivot_scale, std::fabs(a[k][k]));
  }
  const double tolerance = 1e-10 * pivot_scale;

  std::array<double, 3> solution{0.0, 0.0, 0.0};
  for (std::size_t step = 3; step-- > 0;) {
    if (std::fabs(a[step][step]) <= tolerance) {
      solution[step] = 0.0;  // unobservable in this pose
      continue;
    }
    double sum = b[step];
    for (std::size_t j = step + 1; j < 3; ++j) {
      sum -= a[step][j] * solution[j];
    }
    solution[step] = sum / a[step][step];
  }
  return solution;
}

}  // namespace

SwerveKinematics::SwerveKinematics(
  double a, double b, double wheel_radius,
  const std::array<double, kNumWheels> & lateral_offsets)
: a_(a), b_(b), wheel_radius_(wheel_radius)
{
  if (a <= 0.0) {
    throw std::invalid_argument("a must be positive");
  }
  if (b <= 0.0) {
    throw std::invalid_argument("b must be positive");
  }
  if (wheel_radius <= 0.0) {
    throw std::invalid_argument("wheel_radius must be positive");
  }
  for (const double offset : lateral_offsets) {
    if (!std::isfinite(offset)) {
      throw std::invalid_argument("lateral offsets must be finite");
    }
  }
  modules_ = {{{a_, b_}, {-a_, b_}, {-a_, -b_}, {a_, -b_}}};
  h_ = lateral_offsets;
}

std::array<std::array<double, 2>, kNumWheels> SwerveKinematics::wheel_velocity_components(
  const BodyTwist & body_twist) const
{
  std::array<std::array<double, 2>, kNumWheels> components{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const double x_i = modules_[i][0];
    const double y_i = modules_[i][1];
    components[i][0] = body_twist.vx - (y_i * body_twist.omega);
    components[i][1] = body_twist.vy + (x_i * body_twist.omega);
  }
  return components;
}

std::array<WheelCommand, kNumWheels> SwerveKinematics::inverse_kinematics(
  const BodyTwist & body_twist) const
{
  std::array<WheelCommand, kNumWheels> commands{};
  const auto components = wheel_velocity_components(body_twist);
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const double vx_i = components[i][0];
    const double vy_i = components[i][1];
    // atan2(0, 0) == 0.0. That is the identity behind OP-24 stage 2: a zero
    // twist resolves to steering 0 rad, i.e. CENTRE, on every wheel. The
    // controller must therefore never let a zero twist reach this function's
    // result — see SwerveController::update().
    //
    // THE ANGLE IS UNCHANGED BY THE CONTACT-POINT CORRECTION, and that is a
    // result, not an omission. The lateral no-slip constraint at the contact
    // point reads  n(delta) . u_i + omega * t_i = 0, where t_i is the LONGITUDINAL
    // king-pin -> contact offset; the LATERAL offset h_i drops out of it exactly.
    // With t_i = 0 the solution is delta_i = atan2(vy_i, vx_i) — the line below,
    // bit for bit.
    //
    // DROPPED TERM, named rather than hidden: t_i is not identically zero. The
    // URDF (gripperx_v1.core.xacro:303-306, *_wheel_offset_xyz x) gives
    // +-0.000065 m, which would tilt each wheel by eps_i = asin(omega t_i / s_i)
    // — 0.018 deg at the in-place spin pose, ~1/20 of one servo count and three
    // orders below h_i. It is deliberately NOT modelled: carrying it would make
    // the steering angles differ from today's in the last digits for no
    // measurable benefit, and "steering angles unchanged" is the property that
    // makes this change reviewable against the machine.
    commands[i].steering_angle = normalize_angle(std::atan2(vy_i, vx_i));
    commands[i].linear_speed = std::hypot(vx_i, vy_i);
    // The pair above is the KING-PIN solution and stays so. The correction rides
    // alongside and is applied AFTER the +-180 deg fold, because it is
    // fold-invariant — see WheelCommand::offset_speed and equivalent_solutions().
    commands[i].offset_speed = body_twist.omega * h_[i];
  }
  return commands;
}

BodyTwist SwerveKinematics::forward_kinematics_body(
  const std::array<double, kNumWheels> & steering_angles,
  const std::array<double, kNumWheels> & wheel_linear_speeds) const
{
  // One rolling row per wheel, at the CONTACT point:
  //
  //   v_i = vx cos d_i + vy sin d_i + omega [ (-y_i cos d_i + x_i sin d_i) - h_i ]
  //
  // with (x_i, y_i) the KING PIN. The h_i term is what  z_hat x R(d) e  contributes
  // along the rolling direction, and it is independent of d_i and of the body
  // velocity — the same closed form the inverse direction uses.
  //
  // Solved as least squares over the four rows. The previous 0.25 / 4(a^2+b^2)
  // closed form is GONE: it is the least-squares solution only when the normal
  // matrix is diagonal, which needs a symmetric fixed wheel layout and h_i = 0.
  // Keeping it would leave this function inverting a different model than
  // inverse_kinematics() produces, and the steer-feedback differential
  // (SwerveController::apply_steer_feedback_differential) differentiates one
  // against the other.
  std::array<std::array<double, 3>, kNumWheels> rows{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const double cos_delta = std::cos(steering_angles[i]);
    const double sin_delta = std::sin(steering_angles[i]);
    rows[i][0] = cos_delta;
    rows[i][1] = sin_delta;
    rows[i][2] = (-modules_[i][1] * cos_delta) + (modules_[i][0] * sin_delta) - h_[i];
  }

  const auto solution = solve_least_squares_4x3(rows, wheel_linear_speeds);
  return BodyTwist{solution[0], solution[1], solution[2]};
}

}  // namespace gripperx_swerve_controller
