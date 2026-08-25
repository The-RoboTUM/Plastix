"""Geometric constants for GripperX.

GENERATED FROM gripperx_geometry/config/geometry.yaml — DO NOT EDIT

Import these rather than writing a number into a test. A literal in a
test file is a second source of truth that nothing keeps in step, which
is how three test files came to validate against a robot that had not
existed since 2026-08-19.
"""

# cad, accepted, 2026-08-19
WHEEL_RADIUS_GEOMETRIC = 0.070

# measured, accepted, 2026-08-19
WHEEL_RADIUS_EFFECTIVE = 0.070

# cad, accepted, 2026-08-19
HALF_WHEELBASE_KINGPIN = 0.1809

# cad, accepted, 2026-08-19
HALF_TRACK_KINGPIN = 0.1087

# cad, accepted, 2026-08-19
KINGPIN_TO_CONTACT_LATERAL = 0.055572

# derived, accepted, 2026-08-19
HALF_WHEELBASE_CONTACT = 0.180900

# derived, accepted, 2026-08-19
HALF_TRACK_CONTACT = 0.164222

# cad, accepted, 2026-08-19
WHEEL_POSITIONS_CONTACT = (
    0.180900,
    0.164272,
    -0.180900,
    0.164272,
    -0.180900,
    -0.164271,
    0.180900,
    -0.164072,
)

# cad, accepted, 2026-08-19
WHEEL_LATERAL_OFFSET_JOINT_ORDER = (
    0.055572,
    -0.055372,
    0.055572,
    -0.055571,
)

# derived, TO-VERIFY, 2026-08-19
TRACK_WIDTH = 0.2174

# derived, TO-VERIFY, 2026-08-19
WHEELBASE = 0.3618

# cad, accepted, 2026-08-19
STEER_MOUNT_Z_NOMINAL = 0.0303

# derived, accepted, 2026-08-19
BASE_FOOTPRINT_TO_BASE_LINK_Z = 0.1566
