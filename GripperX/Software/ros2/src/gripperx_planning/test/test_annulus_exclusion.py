"""P2b — no twist DWB can select may land in the forbidden steering annulus.

WHAT THIS TEST IS FOR
    This robot's steering stops make a band of turn radii physically
    unreachable.  A command inside that band is not refused: the steering
    limiter silently widens the radius, so the robot drives a different path
    from the one Nav2 scored (finding F2 of the internal rotation analysis of
    2026-09-18, tracked internally and not part of this repository).
    P2b closes the band by construction, by capping DWB's angular rate.  This
    test is what keeps it closed.

WHAT WOULD MAKE IT FAIL — deliberately, this is the point
    It reads THREE ACTIVE CONFIGURATION FILES and duplicates no geometric or
    kinematic constant:
      * gripperx_planning/config/nav2.yaml            (the velocity box)
      * gripperx_control/config/ros2_controllers.yaml (a, b, steering stops)
      * the controller's own reachability code, gripperx_control.steering_limits
    So it fails if somebody raises FollowPath.max_vel_theta, lowers
    min_speed_xy, narrows the steering stops, or moves the king-pin geometry —
    i.e. it fails on a change to the MACHINE'S configuration, not only on a
    change to this file.  A version of this test that hardcoded 0.36705 would
    prove nothing: it would agree with itself forever.

    The one thing it necessarily restates is DWB's own admissibility rule
    (isValidSpeed).  That is upstream logic, not a project constant, and it is
    transcribed from the pinned source with the version named below so a
    reviewer can diff it.
"""

import math
import os

import pytest

yaml = pytest.importorskip("yaml")

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.abspath(os.path.join(_HERE, "..", ".."))

NAV2_YAML = os.path.join(_SRC, "gripperx_planning", "config", "nav2.yaml")
CONTROLLERS_YAML = os.path.join(_SRC, "gripperx_control", "config", "ros2_controllers.yaml")


def _load(path):
    if not os.path.isfile(path):
        pytest.skip(f"active config not reachable from the source tree: {path}")
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


@pytest.fixture(scope="module")
def follow_path():
    return _load(NAV2_YAML)["controller_server"]["ros__parameters"]["FollowPath"]


@pytest.fixture(scope="module")
def controller_server():
    return _load(NAV2_YAML)["controller_server"]["ros__parameters"]


@pytest.fixture(scope="module")
def smoother():
    return _load(NAV2_YAML)["velocity_smoother"]["ros__parameters"]


@pytest.fixture(scope="module")
def annulus():
    """(R_inner, R_outer) from the CONTROLLER'S OWN reachability logic.

    Not the closed form, and not a stored number: every radius is put through
    resolve_wheel_targets(), the same call the drive path uses, so the bounds
    move automatically with the steering stops and the geometry.
    """
    steering_limits = pytest.importorskip("gripperx_control.steering_limits")
    kinematics = pytest.importorskip("gripperx_control.swerve_kinematic_model")

    params = _load(CONTROLLERS_YAML)["swerve_controller"]["ros__parameters"]
    limits = steering_limits.SteeringLimits.from_outward_inward(
        math.radians(params["steering_outward_limit_deg"]),
        math.radians(params["steering_inward_limit_deg"]),
        params["steering_outward_sign"],
    ).in_model_order()
    model = kinematics.FourWIS4WIDKinematicModel(
        params["a"], params["b"], params["wheel_radius"]
    )

    def reachable(radius):
        # Probe at a fixed vx; only the RATIO vx/omega decides the steering
        # angles, so the probe speed is irrelevant to the answer.
        twist = kinematics.BodyTwist(vx=0.1, vy=0.0, omega=0.1 / radius)
        targets = steering_limits.resolve_wheel_targets(
            model.inverse_kinematics(twist), [0.0] * 4, limits
        )
        return targets is not None

    def edge(low, high):
        at_low = reachable(low)
        for _ in range(60):
            mid = 0.5 * (low + high)
            if reachable(mid) == at_low:
                low = mid
            else:
                high = mid
        return 0.5 * (low + high)

    # Bracket the two transitions, then bisect. The brackets only have to
    # straddle an edge; their exact values carry no meaning.
    inner = edge(0.05, 0.30)
    outer = edge(0.30, 1.00)
    assert not reachable(0.5 * (inner + outer)), (
        "the band between the two edges is reachable — the annulus model in "
        "this test no longer describes the controller"
    )
    return inner, outer


def _is_valid_speed(vx, vy, theta, params):
    """DWB's own sample filter, nav2 1.3.12.

    Transcribed from dwb_plugins/include/dwb_plugins/kinematic_parameters.hpp
    (inline bool isValidSpeed). Epsilon terms omitted: they widen acceptance by
    ~1e-7 and this test's conclusion must not rest on them.
    """
    max_speed_xy = params.get("max_speed_xy", -1.0)
    min_speed_xy = params.get("min_speed_xy", -1.0)
    min_speed_theta = params.get("min_speed_theta", -1.0)

    magnitude = math.hypot(vx, vy)
    if max_speed_xy >= 0.0 and magnitude > max_speed_xy:
        return False
    if (
        min_speed_xy >= 0.0
        and magnitude < min_speed_xy
        and min_speed_theta >= 0.0
        and abs(theta) < min_speed_theta
    ):
        return False
    if magnitude == 0.0 and theta == 0.0:
        return False
    return True


def test_no_admissible_dwb_twist_lands_in_the_annulus(follow_path, annulus):
    """The P2b claim itself, over DWB's real sampling grid."""
    r_inner, r_outer = annulus

    max_vel_x = follow_path["max_vel_x"]
    min_vel_x = follow_path["min_vel_x"]
    max_vel_theta = follow_path["max_vel_theta"]
    vx_samples = follow_path["vx_samples"]
    vtheta_samples = follow_path["vtheta_samples"]

    offenders = []
    for i in range(vx_samples):
        vx = min_vel_x + (max_vel_x - min_vel_x) * i / max(vx_samples - 1, 1)
        for j in range(vtheta_samples):
            # theta is sampled over [-max_vel_theta, +max_vel_theta]; there is
            # no min_vel_theta key (kinematic_parameters.hpp getMinTheta()).
            theta = -max_vel_theta + 2.0 * max_vel_theta * j / max(vtheta_samples - 1, 1)
            if not _is_valid_speed(vx, 0.0, theta, follow_path):
                continue
            if theta == 0.0:
                continue  # straight line, R is infinite
            radius = vx / abs(theta)
            if r_inner < radius < r_outer:
                offenders.append((vx, theta, radius))

    assert not offenders, (
        f"{len(offenders)} admissible DWB twists fall inside the forbidden "
        f"annulus ({r_inner:.5f} m, {r_outer:.5f} m). First few: "
        f"{offenders[:5]}. The steering limiter would silently widen every one "
        f"of them. Re-derive FollowPath.max_vel_theta as "
        f"min_speed_xy / R_outer and round DOWN."
    )


def test_the_annulus_bound_is_the_binding_one(follow_path, annulus):
    """max_vel_theta must actually be at the derived bound, not far below it.

    Without this, the test above could be passed by setting max_vel_theta to
    zero, which would satisfy the letter of P2b and disable curve following.
    """
    _, r_outer = annulus
    derived = follow_path["min_speed_xy"] / r_outer
    actual = follow_path["max_vel_theta"]
    assert actual <= derived, (
        f"max_vel_theta {actual} exceeds the derived cap {derived:.6f} "
        f"= min_speed_xy / R_outer"
    )
    assert actual >= derived - 0.005, (
        f"max_vel_theta {actual} is more than one rounding step below the "
        f"derived cap {derived:.6f}; DWB is losing curve authority it could "
        f"legitimately have"
    )


def test_dwb_cannot_command_pure_rotation(follow_path):
    """P2b's mechanism: the disjunctive escape in isValidSpeed is closed.

    min_speed_theta must stay ABOVE max_vel_theta so that |theta| can never
    satisfy the theta floor, collapsing isValidSpeed to 'reject everything
    below min_speed_xy'. This is the inversion the comment block at
    min_speed_theta warns must not be 'repaired'.
    """
    assert follow_path["min_speed_theta"] > follow_path["max_vel_theta"]
    assert not _is_valid_speed(0.0, 0.0, follow_path["max_vel_theta"], follow_path)
    # ...and the loop-1 creep twist is rejected before it is ever scored.
    assert not _is_valid_speed(0.03, 0.0, 0.10, follow_path)


def test_rotation_shim_is_in_front_of_dwb(follow_path):
    assert follow_path["plugin"] == "nav2_rotation_shim_controller::RotationShimController"
    assert follow_path["primary_controller"] == "dwb_core::DWBLocalPlanner"


def test_shim_owns_the_closing_rotation(follow_path, controller_server):
    """P2b is only admissible while the shim performs the closing rotation.

    DWB has no pure rotation to offer, so RotateToGoal's window must lie
    strictly inside the region the shim has already taken over.
    """
    assert follow_path["rotate_to_goal_heading"] is True
    shim_takeover = controller_server["goal_checker"]["xy_goal_tolerance"]
    rotate_to_goal_window = follow_path["xy_goal_tolerance"]
    assert rotate_to_goal_window <= shim_takeover, (
        f"RotateToGoal window {rotate_to_goal_window} exceeds the shim's "
        f"takeover radius {shim_takeover}: DWB would be asked for a pure "
        f"rotation it cannot sample"
    )


def test_shim_rotation_is_above_the_knee_and_within_the_smoother(follow_path, smoother):
    """The shim's rotation must be executable end to end.

    Lower bound is min_speed_theta, which IS the measured knee (0.60 rad/s,
    2026-08-25) as declared in the active config — read from there rather than
    restated here, so the two cannot drift apart.
    """
    rotation_speed = follow_path["rotate_to_heading_angular_vel"]
    assert rotation_speed >= follow_path["min_speed_theta"], (
        "the shim would command rotations below the measured knee, where the "
        "machine delivers 58-79 % of what is asked"
    )
    assert rotation_speed <= smoother["max_velocity"][2], (
        "the velocity smoother would clip the shim's rotation"
    )


def test_shim_deceleration_is_deliverable(follow_path, smoother):
    """The shim plans sqrt(2*a*err); the smoother must be able to deliver a."""
    planned = follow_path["max_angular_accel"]
    assert planned <= smoother["max_accel"][2]
    assert planned <= abs(smoother["max_decel"][2]), (
        "the shim would plan a deceleration the smoother refuses to deliver, "
        "i.e. it would overshoot the yaw window"
    )


def test_shim_rate_limit_is_open_loop(follow_path):
    """closed_loop must stay false on this machine — see the P2a comment.

    With true, the alignment gate's hold (measured angular velocity pinned at
    zero for up to alignment_timeout_sec) clamps the rotation command at
    max_angular_accel * dt and it never climbs out.
    """
    assert follow_path["closed_loop"] is False
