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

# cad, accepted, 2026-10-07
ARM_MOUNT_XYZ = (
    0.171223,
    -0.000151,
    0.115294,
)

# cad, accepted, 2026-10-07
ARM_MOUNT_RPY = (
    3.141593,
    0.000000,
    1.570796,
)

# derived, TO-VERIFY, 2026-10-12
ARM_JOINT_LOWER = (
    -1.884956,
    -3.342715,
    -0.012217,
    -3.229200,
    -2.705260,
    -0.325204,
)

# derived, TO-VERIFY, 2026-10-12
ARM_JOINT_UPPER = (
    1.884956,
    0.159704,
    3.293627,
    0.112151,
    2.705260,
    1.825608,
)

# measured, TO-VERIFY, 2026-10-12
ARM_JOINT_STOP_LOWER = (
    -1.974233,
    -3.429981,
    -0.013806,
    -3.316466,
    -2.954447,
    -0.325204,
)

# measured, TO-VERIFY, 2026-10-12
ARM_JOINT_STOP_UPPER = (
    2.176719,
    0.246971,
    3.380894,
    0.199418,
    2.848602,
    1.912874,
)

# derived, TO-VERIFY, 2026-08-19
LIDAR_MOUNT_XYZ = (
    0.2488,
    0.0000,
    0.1034,
)

# measured, accepted, 2026-10-12
LIDAR_ENVELOPE_SIZE = (
    0.040,
    0.040,
    0.035,
)

# derived, accepted, 2026-10-13
LIDAR_ENVELOPE_CENTER = (
    0.247600,
    -0.000151,
    0.097500,
)

# cad, accepted, 2026-10-13
CHASSIS_BOX_BOTTOM = (
    -0.0960,
    -0.0850,
    -0.0800,
    0.0960,
    0.0880,
    -0.0040,
)

# cad, accepted, 2026-10-13
CHASSIS_BOX_LOWER_BODY = (
    -0.2251,
    -0.1651,
    -0.0040,
    0.2478,
    0.1663,
    0.0264,
)

# cad, accepted, 2026-10-13
CHASSIS_BOX_UPPER_BODY = (
    -0.2251,
    -0.1645,
    0.0264,
    0.2679,
    0.1646,
    0.0801,
)

# cad, accepted, 2026-10-13
CHASSIS_BOX_PEDESTAL = (
    0.0814,
    -0.0266,
    0.0801,
    0.1857,
    0.0264,
    0.1223,
)

# decision, accepted, 2026-10-13
CHASSIS_BOX_PADDING = (
    0.003,
    0.003,
    0.003,
    0.000,
)

# derived, accepted, 2026-10-20
IMU_MOUNT_XYZ = (
    0.0000,
    0.0000,
    0.0589,
)

# cad, accepted, 2026-10-20
IMU_MOUNT_RPY = (
    3.141593,
    0.000000,
    0.000000,
)
