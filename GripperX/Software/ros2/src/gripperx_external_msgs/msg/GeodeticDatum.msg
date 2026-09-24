# Origin of the flat-earth conversion shared with the Octopus.
#
# The datum is NOT a configuration value: it arrives on
# /octopus/fake_eve_gps_start and moves whenever the Eve marker is dragged on
# the dashboard map. Config carries only a fallback plus the jump policy.

std_msgs/Header header

float64 latitude_deg
float64 longitude_deg

# True when this datum came from the Octopus topic, false when the locally
# configured fallback is in use.
bool from_topic

# True when the value equals the Octopus bootstrap fallback
# (Garching 48.2513611 / 11.6359722), i.e. the dashboard never supplied a real
# datum. Dispatch on such a datum is refused.
bool is_bootstrap_fallback

# Age of the last update in seconds; -1.0 when never received.
float32 age_sec
