# Lifecycle of one external goal, including every rejection verdict.
#
# Rejections are surfaced here as well as logged and counted on /diagnostics
# (SR-13: a lost path must produce an active signal, not an absence).

std_msgs/Header header

string target_id

uint8 STATE_UNKNOWN=0
uint8 STATE_RECEIVED=1
uint8 STATE_REJECTED=2
uint8 STATE_PREVIEW=3       # validated but deliberately not dispatched (disarmed / dry_run)
uint8 STATE_NAVIGATING=4
uint8 STATE_REACHED=5
uint8 STATE_PICKING=6
uint8 STATE_DONE=7
uint8 STATE_FAILED=8
uint8 STATE_BLACKLISTED=9   # max_attempts_per_target exhausted; deliberately NOT acknowledged
uint8 state

# Machine-readable verdict from the validation pipeline, e.g.
# "BOOTSTRAP_FALLBACK_DATUM". Empty on success.
string reason
# Human-readable elaboration; never parsed.
string detail

uint16 attempts

# True once trash_goal_done was published for this id. Means "collected" to the
# Octopus, so it is set only after a successful pick when auto-pick is on.
bool acknowledged
