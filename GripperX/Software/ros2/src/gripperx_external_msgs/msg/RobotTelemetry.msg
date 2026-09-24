# Robot telemetry, in typed internal form.
#
# goal_gateway_node assembles this; octopus_link_node converts it to the
# external JSON and publishes it on /octopus/devices/gripperx/status. The
# gateway therefore never speaks the wire format (FR-12 item 2), and the link
# node never makes a decision.
#
# EVERY FIELD REPORTS ITS OWN AVAILABILITY (FR-12 item 8). A value that cannot
# be measured is reported as unavailable WITH A REASON - never as 0, never as a
# plausible-looking default, never omitted in a way a consumer will read as
# zero. That is why each block carries a *_valid flag and a *_reason string
# rather than relying on a sentinel number.

std_msgs/Header header

string device_id

# --- pose in the map frame (TF map -> base_footprint) ---------------------
bool pose_valid
# Empty when valid; otherwise a machine-readable reason, e.g. "TF_UNAVAILABLE".
string pose_reason
float64 map_x
float64 map_y
float64 yaw_deg
float32 pose_age_sec

# --- the same pose as WGS84, in the Octopus's own expansion ---------------
# Separately flagged: the map pose can be perfectly known while the datum is
# not, and reporting a lat/lon derived from a missing or bootstrap datum would
# be a fabricated position on their dashboard.
bool latlon_valid
string latlon_reason
float64 latitude_deg
float64 longitude_deg

# --- odometry (/odometry/filtered) ----------------------------------------
bool odom_valid
string odom_reason
float64 speed_mps
float32 odom_age_sec

# --- navigation and the authority gate ------------------------------------
# "unavailable" while no action client exists (rollout stages 1 and 2 create
# none at all, deliberately).
string nav_state
string nav_state_reason
string active_goal_id

bool armed
float32 arming_seconds_remaining
string last_disarm_trigger

# Observed, NEVER written. Publishing /teleop/set_mode would let the external
# path arm the very chain it is gated by (SR-15 rules 6 and 9).
string teleop_mode

# Seconds since the last `/teleop/active_mode` message, -1.0 if none was ever
# seen. MONOTONIC wall seconds (SAFETY.md F-38): the mode topic carries no stamp
# of its own, so this is the gateway's reception time and a slowed ROS clock
# must not stretch it - a `teleop_mux` that stopped publishing has stopped in
# real time.
float32 teleop_mode_age_sec

# --- external link ---------------------------------------------------------
bool link_ok
float32 link_last_message_age_sec
uint32 link_reconnects

# --- validation counters and the local blacklist ---------------------------
uint32 goals_received
uint32 goals_accepted
uint32 goals_rejected
uint32 goals_preview
string last_reject_reason
# Ids that exhausted their attempts. Valid only within one link session: the
# source restarts its ids at 1, so an id reappearing after a reconnect must not
# be silently trusted as the same object (FR-12 item 7).
string[] blacklisted_ids

# --- battery ---------------------------------------------------------------
# Permanently "unavailable" with reason "NO_SENSOR_INSTALLED": there is no
# measurement chain on this robot at all - no shunt, no divider, no ADC channel
# in the ESP32 pin map. HWR-21 names "battery V/%" as display content but
# specifies no measurement. Shipping the field honestly means the dashboard
# path exists from day one and lights up the moment the hardware does; shipping
# a plausible number would be a lie the dashboard displays.
string battery_status
string battery_reason
# False => the percent is emitted as JSON null, not as 0.0.
bool battery_percent_valid
float32 battery_percent
