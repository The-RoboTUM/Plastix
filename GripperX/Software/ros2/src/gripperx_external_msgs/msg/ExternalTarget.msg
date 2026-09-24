# One entry of the external target list (/octopus/trash_gps).
#
# This is PREVIEW MATERIAL, not a goal. The counterpart pushes exactly one goal
# at a time on /octopus/trash_goal and advances only on acknowledgement, so the
# list exists to show the operator what is out there - never to be dispatched
# from.

string id

float64 latitude_deg
float64 longitude_deg

# The source's OWN map metres, verbatim. We do not navigate on these: x/y are
# re-derived from lat/lon with the shared datum so that one arithmetic path is
# authoritative. Kept because a mismatch between the two is exactly how the
# origin offset on their side becomes visible at rollout stage 2.
# NaN when the source omitted them.
float64 source_x
float64 source_y

# Detector confidence; NaN when the source sent null (a real value on their
# side, not a fault).
float32 confidence

bool collected
bool is_goal

# Epoch seconds; NaN when absent.
float64 last_seen
