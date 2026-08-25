// 4WIS4WID kinematics — C++ port of gripperx_control/swerve_kinematic_model.py.
//
// Source of the model: M.-H. Lee and T.-H. S. Li, "Kinematics, dynamics and
// control design of 4WIS4WID mobile robots", The Journal of Engineering, 2015.
//
// PORTED, NOT REWRITTEN. Every formula below is the same formula the Python
// module evaluates today, in the same wheel order, so that NFR-10 acceptance 1
// ("functionally equivalent to today's chain") can be checked twist by twist
// rather than argued. Only what the controller actually uses is ported: the
// forward kinematics and the paper's full state derivative have no consumer in
// the direct-IK path and are deliberately left behind (NFR-11).
//
// Wheel order here is the PAPER's order (w1..w4 = FL, BL, BR, FR), which is the
// order the IK works in. The controller's parameters and the servo bus use
// joint order (FL, FR, BL, BR). The two are converted explicitly, never
// implicitly — see kModelToJointIndex in steering_limits.hpp.

#ifndef GRIPPERX_SWERVE_CONTROLLER__SWERVE_KINEMATICS_HPP_
#define GRIPPERX_SWERVE_CONTROLLER__SWERVE_KINEMATICS_HPP_

#include <array>
#include <cmath>
#include <cstddef>

namespace gripperx_swerve_controller
{

constexpr std::size_t kNumWheels = 4;

/// atan2(sin, cos) — identical to steering_limits.normalize_angle in Python.
inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct BodyTwist
{
  double vx{0.0};
  double vy{0.0};
  double omega{0.0};
};

struct WheelCommand
{
  double steering_angle{0.0};
  double linear_speed{0.0};
  /// CONTACT-POINT CORRECTION, and it is deliberately NOT folded into
  /// linear_speed. The pair above is the KING-PIN solution; the tyre runs on a
  /// circle offset from the king pin by e_i = (t_i, h_i), and the rolling speed
  /// the contact patch needs is  (+-linear_speed) - omega*h_i, where the sign of
  /// linear_speed is chosen by the +-180 deg module fold.
  ///
  /// The `- omega*h_i` term is FOLD-INVARIANT: it carries the SAME sign on the
  /// folded and the unfolded branch (see equivalent_solutions()). Adding it to
  /// linear_speed here would let the fold negate it too, which is wrong on every
  /// folded wheel — and in an in-place spin exactly half the wheels are folded.
  /// It therefore travels separately and is applied AFTER the fold.
  double offset_speed{0.0};
};

/// Kinematic model for a symmetric 4WIS4WID robot.
/// a = half wheelbase (centroid to axle), b = half track, r = wheel radius.
///
/// `lateral_offsets` are the king-pin -> tyre-contact lateral offsets h_i in
/// MODEL order (FL, BL, BR, FR), signed in base_link y (positive = left). They
/// default to zero, and on zero this class behaves bit-for-bit as it did before
/// the correction existed.
class SwerveKinematics
{
public:
  SwerveKinematics(
    double a, double b, double wheel_radius,
    const std::array<double, kNumWheels> & lateral_offsets = {{0.0, 0.0, 0.0, 0.0}});

  double a() const { return a_; }
  double b() const { return b_; }
  double wheel_radius() const { return wheel_radius_; }
  /// King-pin -> contact lateral offset of one MODEL-order wheel, base_link y.
  double lateral_offset(std::size_t index) const { return h_[index]; }

  /// Per-wheel no-slip velocity components in the robot frame, model order.
  std::array<std::array<double, 2>, kNumWheels> wheel_velocity_components(
    const BodyTwist & body_twist) const;

  /// Body twist -> per-wheel steering angle and signed linear speed, model order.
  std::array<WheelCommand, kNumWheels> inverse_kinematics(const BodyTwist & body_twist) const;

  /// Wheel steering angles + CONTACT-POINT linear speeds -> body twist (model
  /// order). Used only by the steer-feedback differential, which reconstructs an
  /// omega from the MEASURED steering angles.
  ///
  /// This is the exact inverse of inverse_kinematics() + the contact-point
  /// correction, solved in the least-squares sense over the four rolling rows.
  /// It MUST stay the inverse of the model the differential differentiates
  /// against, which is why it carries h_i as well.
  BodyTwist forward_kinematics_body(
    const std::array<double, kNumWheels> & steering_angles,
    const std::array<double, kNumWheels> & wheel_linear_speeds) const;

private:
  double a_;
  double b_;
  double wheel_radius_;
  // Module positions in the paper's order: w1 (a,b) w2 (-a,b) w3 (-a,-b) w4 (a,-b).
  // These are the KING PINS; the contact points sit h_i further out in y.
  std::array<std::array<double, 2>, kNumWheels> modules_;
  // King-pin -> contact lateral offsets, model order, base_link y (positive = left).
  std::array<double, kNumWheels> h_{};
};

}  // namespace gripperx_swerve_controller

#endif  // GRIPPERX_SWERVE_CONTROLLER__SWERVE_KINEMATICS_HPP_
