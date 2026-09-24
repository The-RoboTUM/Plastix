"""Invariants of the `octopus_line` frame (spec sections 1-5).

These assert what the frame IS, not how the operator arrives at it, so they
survive the parts of the GripperX document that are still moving.
"""

import math

import pytest

from octopus_camera_transform.line_frame import (
    LineCalibration,
    LineCalibrationError,
)

L = 2.8
# A horizontal pair: A left, B right, 400 px apart, v constant.
A = (100.0, 240.0)
B = (500.0, 240.0)


def calib(**kwargs):
    return LineCalibration(A, B, L, **kwargs)


def test_post_a_sits_at_minus_half_l():
    x, y = calib().pixel_to_line(A)
    assert x == pytest.approx(-L / 2)
    assert y == pytest.approx(0.0)


def test_post_b_sits_at_plus_half_l():
    x, y = calib().pixel_to_line(B)
    assert x == pytest.approx(L / 2)
    assert y == pytest.approx(0.0)


def test_midpoint_is_the_origin():
    x, y = calib().pixel_to_line(calib().pixel_midpoint)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)


def test_left_of_a_to_b_is_positive_y():
    # u right, v DOWN, non-mirrored top-down image: left of A->B as seen from
    # above is UP the page, i.e. smaller v. This is the sign the spec warns
    # about in section 3 and lists as unverified in section 9.
    x, y = calib().pixel_to_line((300.0, 140.0))
    assert x == pytest.approx(0.0)
    assert y > 0.0


def test_mirrored_image_flips_y_only():
    point = (300.0, 140.0)
    x_plain, y_plain = calib().pixel_to_line(point)
    x_mirror, y_mirror = calib(mirrored=True).pixel_to_line(point)
    assert x_mirror == pytest.approx(x_plain)
    assert y_mirror == pytest.approx(-y_plain)


def test_scale_is_length_over_pixel_separation():
    c = calib()
    assert c.pixel_separation == pytest.approx(400.0)
    assert c.metres_per_pixel == pytest.approx(L / 400.0)


def test_distance_between_posts_is_l_whatever_the_orientation():
    # A diagonal pair must still put the posts exactly L apart: the frame is
    # defined by the posts, not by the image axes.
    diagonal = LineCalibration((100.0, 100.0), (400.0, 500.0), L)
    ax, ay = diagonal.pixel_to_line((100.0, 100.0))
    bx, by = diagonal.pixel_to_line((400.0, 500.0))
    assert math.hypot(bx - ax, by - ay) == pytest.approx(L)
    assert ay == pytest.approx(0.0)
    assert by == pytest.approx(0.0)


def test_swapping_a_and_b_turns_the_frame_by_180_degrees():
    # Spec section 2: swapping the posts is not an error, it silently negates
    # every coordinate. Pinning it here so nobody "fixes" it later.
    point = (300.0, 140.0)
    x_ab, y_ab = LineCalibration(A, B, L).pixel_to_line(point)
    x_ba, y_ba = LineCalibration(B, A, L).pixel_to_line(point)
    assert x_ba == pytest.approx(-x_ab)
    assert y_ba == pytest.approx(-y_ab)


def test_implied_height_matches_the_pinhole_relation():
    fx = 359.3292231592479
    c = calib()
    assert c.implied_camera_height_m(fx) == pytest.approx(c.metres_per_pixel * fx)


@pytest.mark.parametrize("bad_length", [2.49, 3.01, 0.0, -2.8, float("nan")])
def test_length_outside_the_plausibility_bound_is_refused(bad_length):
    with pytest.raises(LineCalibrationError):
        LineCalibration(A, B, bad_length)


def test_bound_is_adjustable_because_the_spec_still_moves():
    # The 2.5-3.0 m range is a stated plausibility bound, not a measurement.
    assert LineCalibration(A, B, 4.0, max_length_m=5.0).length_m == pytest.approx(4.0)


def test_marks_too_close_together_are_refused():
    with pytest.raises(LineCalibrationError):
        LineCalibration((300.0, 240.0), (305.0, 240.0), L)


def test_as_dict_is_json_safe():
    import json

    payload = json.loads(json.dumps(calib().as_dict()))
    assert payload["calibrated"] is True
    assert payload["length_m"] == pytest.approx(L)
