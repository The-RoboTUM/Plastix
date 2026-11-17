# One external goal as received from the Octopus, plus its resolution into a
# robot standing pose.
#
# The incoming position is the OBJECT, not a robot pose. By default the arm is not
# aimed (the fixed pick of the taught pose), so the standing pose is then the only
# aiming mechanism. An aimed pick exists (PickPlastic `aimed`/`target`, enabled by the
# gateway parameter grasp.aimed_pick, see the gateway configs); it aims at the
# dispatched object position.

std_msgs/Header header

# Origin system, e.g. "octopus".
string source

# Target id as issued by the source, echoed back verbatim on trash_goal_done.
# String because the Octopus ids are ints today but restart at 1 on node
# restart (proposal item 3); we never do arithmetic on them.
string target_id

# --- as received -------------------------------------------------------
float64 object_latitude_deg
float64 object_longitude_deg

# Detector confidence, NaN when the source sent null.
float32 confidence

GeodeticDatum datum

# --- resolved ----------------------------------------------------------
# Object position in the map frame, from the flat-earth conversion.
float64 object_x
float64 object_y

# Robot standing pose: object_xy - R(yaw) * grasp_offset. Only meaningful when
# `resolved` is true; false while the grasp offset is still TO-VERIFY or no
# approach candidate was acceptable.
bool resolved
float64 robot_x
float64 robot_y
float64 robot_yaw

# Angular deviation of the chosen approach heading from the seed bearing
# (robot -> object), in radians. 0.0 for the straight-at-it normal case.
float32 approach_deviation_rad
