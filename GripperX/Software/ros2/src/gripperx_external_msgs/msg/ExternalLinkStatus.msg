# Health of the rosbridge link to the external system.

std_msgs/Header header

string url
bool connected
uint32 reconnect_count

# Seconds since the last payload of any kind; -1.0 when nothing was ever
# received. Link loss is an auto-disarm trigger, so this is safety-relevant.
float32 last_message_age_sec

# Targets still open per /octopus/trash_gps. The Octopus stops publishing
# trash_goal when nothing is open, so open_count == 0 is the ONLY way to tell
# "mission complete" from "source went stale".
uint32 open_count
bool open_count_valid

bool datum_valid
uint32 goals_received
uint32 goals_rejected

# ---------------------------------------------------------------------------
# The Octopus's own map-frame transform, from
# /octopus/flight_camera_transform/status. Q1 was answered on 2026-08-21: their
# map +y is the DRONE'S HEADING AT STARTUP, and it RE-LOCKS whenever their
# transform node restarts - so an alignment measured before a re-lock is no
# longer true and nothing else in their protocol says so.
#
# THREE STATES, DELIBERATELY NOT COLLAPSED INTO ONE FLAG:
#   enabled=false                -> we are not subscribed at all
#   enabled=true, seen=false     -> subscribed, nothing has arrived yet
#   enabled=true, seen=true      -> ready/not-ready is a real observation
# An ABSENT signal is not evidence that their frame is bad. A counterpart that
# does not publish this topic at all must never be blocked by its silence, so
# only an OBSERVED not-ready may gate anything.
bool frame_status_enabled
bool frame_status_seen

# THEIR `state == "ready"` AND a non-null yaw lock AND the status is FRESH. The
# three are folded together here because the timestamps live in the link node,
# and a stale "ready" is exactly what their node dying looks like.
bool frame_ready

# map_yaw_offset_rad - yaw_zero_rad. Only meaningful while _valid; there is no
# sentinel value, because 0.0 would read as "the drone faces east".
float32 frame_align_angle_rad
bool frame_align_angle_valid

# Re-locks observed since the link node started.
uint32 frame_relocks
