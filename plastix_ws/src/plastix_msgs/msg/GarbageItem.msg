# GarbageItem.msg
# Represents a detected piece of garbage

# Unique identifier for this garbage item
string id

# Classification of garbage
uint8 TYPE_UNKNOWN = 0
uint8 TYPE_PLASTIC_SMALL = 1    # Small plastic items (Robby's target)
uint8 TYPE_PLASTIC_LARGE = 2    # Large plastic items (GripperX's target)
uint8 TYPE_FLOATING = 3         # Floating garbage (SharX's target)
uint8 TYPE_METAL = 4
uint8 TYPE_ORGANIC = 5
uint8 type

# Size classification
uint8 SIZE_SMALL = 0    # < 10cm - Robby handles
uint8 SIZE_MEDIUM = 1   # 10-50cm - Robby or GripperX
uint8 SIZE_LARGE = 2    # > 50cm - GripperX handles
uint8 size_class

# Position in world frame
geometry_msgs/Point position

# Confidence of detection (0.0 - 1.0)
float32 confidence

# Timestamp of detection
builtin_interfaces/Time detection_time

# Which robot should handle this (empty if unassigned)
string assigned_robot

# Status
uint8 STATUS_DETECTED = 0
uint8 STATUS_ASSIGNED = 1
uint8 STATUS_IN_PROGRESS = 2
uint8 STATUS_COLLECTED = 3
uint8 STATUS_FAILED = 4
uint8 status

