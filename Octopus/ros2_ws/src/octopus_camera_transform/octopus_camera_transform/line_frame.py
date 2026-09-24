#!/usr/bin/env python3
"""The shared reference line frame `octopus_line`.

Spec: `Octopus/docs/line_calibration.md`, which mirrors the GripperX document
`OCTOPUS_LINE_CALIBRATION.md` (draft, 2026-09-24). This module implements only
sections 1-5 of that document, which are concrete; everything from section 6 on
is still moving and is deliberately not encoded here.

The frame is defined by two vertical posts of the aluminium frame Eve hangs
from. Both sides measure the same two posts and express every exchanged
coordinate in the frame those posts span:

    origin = midpoint of the two post centres
    +x     = from post A towards post B
    +y     = +x turned 90 deg counter-clockwise SEEN FROM ABOVE ("left of A->B")
    +z     = up, right-handed (REP-103)

GripperX measures the post distance L with its LiDAR and is metric by
construction; Octopus does not measure L, it is told L and uses it to turn
pixels into metres. That makes L the single scale input of this module.

No ROS here on purpose: this is the part worth unit-testing, and the invariants
it guarantees (A at -L/2, B at +L/2, midpoint at the origin) come straight from
the spec and cannot change even if the procedure around them does.
"""

import math

# GripperX refuses a clicked pair whose L falls outside this range and shows a
# red REJECTED marker. It is a plausibility bound on the post spacing stated by
# the operator on 2026-09-24, NOT a measurement - see section 9 of the spec.
# Mirrored here so a typo on our side fails the same way theirs does, and kept
# as a parameter because the bound is in the part of the document still moving.
DEFAULT_MIN_LENGTH_M = 2.5
DEFAULT_MAX_LENGTH_M = 3.0

# Two clicks closer together than this cannot define a direction usefully: the
# angle error grows without bound as the pair degenerates to a point.
MIN_PIXEL_SEPARATION = 20.0


class LineCalibrationError(ValueError):
    """Raised when a clicked pair cannot define a frame."""


class LineCalibration:
    """A solved reference line: two post centres in pixels plus their distance.

    Args:
        pixel_a: (u, v) of post A's centre in the camera image.
        pixel_b: (u, v) of post B's centre.
        length_m: L, the distance between the post centres in metres, as
            measured by GripperX's LiDAR and read out by its operator.
        mirrored: set when the displayed image is not the plain view from above
            (mirrored horizontally, or the camera mounted so the image is
            flipped). This inverts +y. The spec's formula assumes u right and v
            DOWN on a non-mirrored top-down image; section 9 lists the real
            system's handedness as unverified, so this stays an explicit input
            rather than a baked-in assumption.
    """

    def __init__(self, pixel_a, pixel_b, length_m, mirrored=False,
                 min_length_m=DEFAULT_MIN_LENGTH_M,
                 max_length_m=DEFAULT_MAX_LENGTH_M):
        self.pixel_a = (float(pixel_a[0]), float(pixel_a[1]))
        self.pixel_b = (float(pixel_b[0]), float(pixel_b[1]))
        self.length_m = float(length_m)
        self.mirrored = bool(mirrored)

        if not math.isfinite(self.length_m):
            raise LineCalibrationError("L is not a finite number")
        if not (min_length_m <= self.length_m <= max_length_m):
            raise LineCalibrationError(
                f"L = {self.length_m:.3f} m, expected "
                f"{min_length_m:.3f}-{max_length_m:.3f} m"
            )

        du = self.pixel_b[0] - self.pixel_a[0]
        dv = self.pixel_b[1] - self.pixel_a[1]
        self.pixel_separation = math.hypot(du, dv)
        if self.pixel_separation < MIN_PIXEL_SEPARATION:
            raise LineCalibrationError(
                f"post marks are {self.pixel_separation:.1f} px apart, need at "
                f"least {MIN_PIXEL_SEPARATION:.0f} px to define a direction"
            )

        # Unit vector along A->B, in pixels.
        self.ex = (du / self.pixel_separation, dv / self.pixel_separation)
        # Midpoint of the pair: the origin of the line frame.
        self.pixel_midpoint = (
            (self.pixel_a[0] + self.pixel_b[0]) / 2.0,
            (self.pixel_a[1] + self.pixel_b[1]) / 2.0,
        )
        # The one scale number: metres per pixel along the line.
        self.metres_per_pixel = self.length_m / self.pixel_separation

    def pixel_to_line(self, pixel):
        """Map an image pixel onto the ground in line-frame metres.

        Section 3 of the spec. Note this is a similarity transform - rotation,
        uniform scale, translation - so it is exact only for a nadir camera over
        a flat floor, and exactly right only along the line itself. It does not
        model perspective or lens distortion; `flight_camera_transform_node`
        does both, which is why this module is a calibration source and not a
        replacement for that projection.
        """
        du = float(pixel[0]) - self.pixel_midpoint[0]
        dv = float(pixel[1]) - self.pixel_midpoint[1]
        x = self.metres_per_pixel * (du * self.ex[0] + dv * self.ex[1])
        y = self.metres_per_pixel * (du * self.ex[1] - dv * self.ex[0])
        if self.mirrored:
            y = -y
        return (x, y)

    def image_angle_rad(self):
        """Direction of A->B in the image, radians, atan2(dv, du).

        This is what the map rotation would be derived from. It is NOT the same
        quantity as `indoor_static_yaw_zero_rad`, which comes from the PX4
        compass; comparing the two is the point (see docs).
        """
        return math.atan2(self.ex[1], self.ex[0])

    def implied_camera_height_m(self, fx):
        """Camera height above the floor implied by the measured scale.

        For a nadir pinhole camera, ground metres per pixel at the principal
        point is h / fx, so h = metres_per_pixel * fx. This is the reason the
        line is worth having: today the height is a configured constant
        (`manual_height_above_ground_m`), and the whole map scales linearly with
        it. Here it becomes a measurement.

        An estimate, not a replacement: it assumes the line lies near the image
        centre and the camera looks straight down. Away from the centre, under
        tilt, or if the clicked post centres are not at floor level, it is
        biased. Use it as a cross-check against the configured height.
        """
        fx = float(fx)
        if not math.isfinite(fx) or fx <= 0.0:
            raise LineCalibrationError(f"fx must be positive and finite, got {fx}")
        return self.metres_per_pixel * fx

    def as_dict(self):
        """JSON-safe summary, for the status topic and the dashboard."""
        return {
            "calibrated": True,
            "pixel_a": list(self.pixel_a),
            "pixel_b": list(self.pixel_b),
            "length_m": self.length_m,
            "pixel_separation": self.pixel_separation,
            "metres_per_pixel": self.metres_per_pixel,
            "pixel_midpoint": list(self.pixel_midpoint),
            "direction_unit_px": list(self.ex),
            "image_angle_rad": self.image_angle_rad(),
            "image_angle_deg": math.degrees(self.image_angle_rad()),
            "mirrored": self.mirrored,
        }
