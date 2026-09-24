# The full external target set, as one message.
#
# Published by octopus_link_node, which is the only node allowed to touch the
# external wire format (FR-12 item 2). Everything downstream sees this typed
# form, which is what keeps the transport swappable.
#
# BURST BEHAVIOUR - the reason this is ONE message and not N:
# their datum_callback treats any marker movement above 1e-9 deg (~0.1 mm) as a
# datum change and immediately republishes EVERY target. Carrying the whole set
# in a single message means a burst cannot grow a queue on our side and the
# preview converges to one consistent state instead of flickering through
# intermediate ones (FR-12 item 5).

std_msgs/Header header

# Origin system, e.g. "octopus".
string source

GeodeticDatum datum

# Id the source currently designates as the goal; empty when the source has
# gone quiet because nothing is open.
string goal_id

# Targets still open. 0 means MISSION COMPLETE, not stale - the source stops
# publishing trash_goal entirely when nothing is open, so this is the only
# signal that distinguishes the two.
uint32 open_count
bool open_count_valid

ExternalTarget[] targets
