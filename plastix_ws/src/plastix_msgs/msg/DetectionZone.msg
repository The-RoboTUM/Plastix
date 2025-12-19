# DetectionZone.msg
# Defines a zone to be cleaned

string zone_id
string zone_name

# Zone boundaries (polygon points)
geometry_msgs/Point[] boundary_points

# Zone center
geometry_msgs/Point center

# Zone priority (higher = more urgent)
uint8 priority

# Zone type
uint8 ZONE_LAND = 0
uint8 ZONE_WATER = 1
uint8 ZONE_MIXED = 2
uint8 zone_type

# Assigned teams
string[] assigned_robots

# Cleaning progress (0.0 - 1.0)
float32 progress

# Is zone currently active
bool is_active

