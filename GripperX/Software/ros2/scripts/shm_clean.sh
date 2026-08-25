#!/bin/bash
# Clean leaked FastDDS shared-memory segments.
#
# WHY THIS EXISTS
# ---------------
# A SIGKILLed FastDDS process cannot run its destructors, so it leaves its
# /dev/shm/fastrtps_* segments behind. Measured on this laptop 2026-08-19:
# one killed node leaks ~8 entries (2 participant segments + shared port files
# and their semaphores); three kills leave 12 entries.
#
# The failure mode is nasty because it does not look like a resource problem.
# At roughly 180 leftovers, newly started participants stop being visible to
# the `ros2` CLI while node-to-node discovery still works -- i.e. `ros2 node
# list` shows nothing although the node's own log shows a clean startup. It
# looks exactly like "the node failed to start".
#
# Kill-based tests are the source: the external-link disarm triggers (LINK_LOST,
# NODE_SHUTDOWN) and the SAFETY.md C-8 acceptance are all verified by killing
# processes, so they produce these in bulk. Cleanly exiting nodes leak nothing
# (verified 2026-08-19, and again 2026-08-20 -- see the per-participant figures
# below; note the count returns to the BASELINE that was there before, which is
# not necessarily 0).
#
# SAFE TO RUN WHILE TESTS ARE RUNNING
# -----------------------------------
# `fastdds shm clean` checks ownership and reports "N segments in use". Verified
# 2026-08-19: with one live node plus one zombie present, it cleaned the zombie,
# spared the live node's segments, and the live node kept running and exited
# normally. So this does NOT require stopping other sessions -- which matters,
# because several worktrees share this laptop (domains 20 real / 220 / 221).
#
# WHAT IT ACTUALLY RECLAIMS -- AND WHAT IT DOES NOT
# -------------------------------------------------
# MEASURED 2026-08-20 18:34 CEST. Conditions, because a reclaim figure without
# them is not comparable to the next one: machine otherwise QUIET, domain 220
# IDLE (the parallel Nav2 session had ended and cleaned up), no test nodes
# running, two `ros2` CLI daemons alive (domains 20 and 221).
#
#     126 entries before  ->  126 after.  ZERO reclaimed.
#
# An earlier reading the same day, taken while the 220 session was still live,
# was 284 -> 198 (43 zombie segments cleaned) and was written up as "the
# remainder belongs to other people's live processes". THAT WAS TOO GENEROUS TO
# THIS SCRIPT AND IS CORRECTED HERE. With 220 idle and nothing of ours running,
# the remainder is almost entirely OURS -- and unreclaimable anyway. The 126:
#
#     4    participant segments (fastrtps_<id> + _el) -- the two live `ros2`
#          daemons, correctly spared;
#     118  port files and their semaphores (59 fastrtps_port<n> + 59
#          sem.fastrtps_port<n>_mutex) owned by NOTHING alive, the oldest
#          timestamped 06:38 the same morning -- 94% of the residue;
#     4    other.
#
# So the honest statement is neither "this removes the exposure" nor "the
# remainder is other sessions' live state". It is: THIS REMOVES RECENT
# PARTICIPANT ZOMBIES AND DOES NOT TOUCH A LONG-LIVED RESIDUE OF ORPHANED PORT
# ENTRIES, WHICH ACCUMULATES ACROSS SESSIONS AND OUTLIVES EVERY PROCESS THAT
# MADE IT.
#
# What it IS good for, measured the same day, one participant at a time -- and
# this is why calling it before a kill-based test suite earns its place:
#
#     clean exit (SIGINT)   126 -> 130 -> 126   leaks NOTHING
#     SIGKILL               126 -> 130 -> 130   leaks 4 entries
#     then this script      130 -> 122          reclaims those 4, plus a little
#                                               older residue
#
# (2026-08-19 recorded ~8 entries per killed node against the 4 per killed
# PARTICIPANT measured here; a node may hold more than one participant, and the
# two figures are reported as taken rather than reconciled.)
#
# The port residue needs a sweep by name with nothing running on ANY domain.
# That is what `--ports` below is, added by user decision 2026-08-20; the plain
# clean above still does NOT do it, and neither may a test suite, while other
# domains may be live. See the guard section for what makes it safe enough to
# exist at all.
#
# For scale: discovery still worked at 126 entries (a freshly started node was
# visible to `ros2 node list`), so the ~180 figure above is not contradicted by
# any of these numbers -- we were below it throughout.
#
# Usage: scripts/shm_clean.sh                   (clean; safe while tests run)
#        scripts/shm_clean.sh --count           (report only, changes nothing)
#        scripts/shm_clean.sh --ports           (DRY RUN: list the orphaned port
#                                                entries a sweep would remove)
#        scripts/shm_clean.sh --ports --delete  (actually sweep them; refuses
#                                                unless NOTHING ROS is running)
#
# THE --ports MODE, AND WHY IT IS GUARDED THE WAY IT IS
# ----------------------------------------------------
# Added 2026-08-20 by user decision, on the measurement above: the port residue
# accumulates across sessions, nothing reclaims it, and ~180 entries is where
# the `ros2` CLI goes blind. Without a sweep we eventually walk into that, and
# it will happen in the middle of a measurement.
#
# THE GUARD IS THE FEATURE. `fastdds shm clean` can be run at any time because
# it checks ownership itself; this mode CANNOT, because it deletes files by
# name. A port entry that looks orphaned to `ls` is being used by any
# participant that has the port open, so sweeping while somebody else's node is
# running breaks THEIR run, not ours. Hence:
#
#   * it refuses to delete unless nothing ROS is running ANYWHERE on the
#     machine -- all domains, not just ours. `ros2` CLI daemons count: they are
#     live participants (`ros2 daemon stop` ends them; they respawn by
#     themselves on the next CLI call);
#   * it is a DRY RUN unless `--delete` is also given, so there are two chances
#     to notice you are on the wrong machine;
#   * it NEVER globs. See the note at the matcher itself;
#   * it is NOT wired into any test suite and must not be. The suite's setup
#     call is the plain clean above, which reclaims a SIGKILLed participant's
#     ~4 entries and needs no guard. A port sweep is an explicit human act.
#
# On ANY doubt it refuses: a false refusal costs a retry, a false proceed costs
# somebody else's running work.

set -u

count() { ls /dev/shm 2>/dev/null | grep -c fastrtps || true; }

before="$(count)"

if [ "${1:-}" = "--count" ]; then
    echo "[shm_clean] /dev/shm fastrtps entries: $before"
    exit 0
fi

# --- --ports: sweep the orphaned port residue -----------------------------
if [ "${1:-}" = "--ports" ]; then
    delete=0
    [ "${2:-}" = "--delete" ] && delete=1

    # WHO IS ALIVE. Two independent tests, because they fail differently: the
    # /proc scan is definitive (a process that MAPS a fastrtps segment is a
    # participant, whatever it is called) but only sees processes whose maps we
    # may read; the pattern match catches ROS processes that have not mapped a
    # segment yet -- including one that is starting up RIGHT NOW, which is
    # exactly the race this guard exists for.
    mapped_pids=""
    for maps in /proc/[0-9]*/maps; do
        pid="${maps#/proc/}"; pid="${pid%/maps}"
        [ "$pid" = "$$" ] && continue
        if grep -qs '/dev/shm/fastrtps' "$maps" 2>/dev/null; then
            mapped_pids="$mapped_pids $pid"
        fi
    done

    # Broad, and deliberately including the `ros2` CLI daemons and the
    # discovery server. Matched on COMMAND TOKENS rather than on the substring
    # "ros2", because the substring also matches every process whose command
    # line merely contains a path like .../Software/ros2/... -- measured: it
    # flagged this laptop's editor daemon, and a guard that refuses forever is a
    # guard someone deletes. `(^|/)ros2( |$)` is the ros2 executable itself;
    # a path component is followed by "/" and so does not match.
    pattern='(^|/)ros2( |$)|ros2cli\.daemon|/opt/ros/|ros_ign|gz sim|gzserver|rviz|micro_ros|fast-discovery-server|fastdds|component_container'
    # Exclude ourselves, our parent, and any shell whose command line merely
    # QUOTES this script (a wrapper, a CI line, an agent transcript). Without
    # this the mode reliably refuses because of the invocation that asked for
    # it -- pgrep matching its own caller is the oldest trap in this file.
    self_pids="$(pgrep -f 'shm_clean\.sh' 2>/dev/null || true)"
    pattern_pids="$(pgrep -f "$pattern" 2>/dev/null \
        | grep -vx "$$" | grep -vx "$PPID" \
        | { if [ -n "$self_pids" ]; then grep -vxF "$self_pids"; else cat; fi; } || true)"

    if [ -n "${mapped_pids// /}" ] || [ -n "$pattern_pids" ]; then
        echo "[shm_clean] --ports REFUSED: ROS processes are running on this machine." >&2
        echo "[shm_clean] This mode deletes files BY NAME and cannot check ownership the" >&2
        echo "[shm_clean] way 'fastdds shm clean' does, so sweeping now could break a run" >&2
        echo "[shm_clean] on ANY domain -- including another session's." >&2
        for pid in $(printf '%s\n' $mapped_pids $pattern_pids | sort -un); do
            [ -r "/proc/$pid/cmdline" ] || continue
            echo "[shm_clean]   pid $pid: $(tr '\0' ' ' < /proc/$pid/cmdline | cut -c1-90)" >&2
        done
        echo "[shm_clean] Stop them (including 'ros2 daemon stop' -- CLI daemons are live" >&2
        echo "[shm_clean] participants and respawn by themselves) and re-run." >&2
        if [ "$delete" -eq 1 ]; then
            echo "[shm_clean] NOTHING WAS DELETED." >&2
            exit 2
        fi
        echo "[shm_clean] Continuing as a DRY RUN only; deletion stays blocked." >&2
        echo "" >&2
    fi

    # THE MATCHER. NARROW ON PURPOSE, and this is load-bearing: /dev/shm is NOT
    # ours. Other applications on this laptop use it heavily, and a glob such as
    # `rm -f /dev/shm/fastrtps*` would also take participant segments belonging
    # to live processes -- which is the one thing the plain clean above is
    # careful never to do. Only two shapes are ever matched:
    #
    #     fastrtps_port<digits>
    #     sem.fastrtps_port<digits>_mutex
    #
    # Anything else in /dev/shm is skipped, including `fastrtps_<hexid>` and
    # `fastrtps_<hexid>_el`, WHICH ARE THE PARTICIPANT SEGMENTS AND MUST SURVIVE.
    # Do not "simplify" this into a glob.
    #
    # KNOWN AND DELIBERATE GAP: `fastrtps_port<n>_el` (the port's event-listener
    # companion) does NOT match either shape and is therefore never swept. Seen
    # 2026-08-20: the four `_el` files present all belonged to LIVE ports, so
    # sparing them was right -- but a killed participant can leave an orphaned
    # one, and this mode will not reclaim it. The sweep is INCOMPLETE BY
    # CONSTRUCTION and errs towards leaving litter rather than towards deleting
    # something in use. Widening the match to `_el` needs its own evidence that
    # a live port never depends on one, which nobody has gathered.
    victims=""
    victim_count=0
    for entry in /dev/shm/*; do
        name="${entry##*/}"
        case "$name" in
            fastrtps_port*)
                echo "$name" | grep -qE '^fastrtps_port[0-9]+$' || continue ;;
            sem.fastrtps_port*)
                echo "$name" | grep -qE '^sem\.fastrtps_port[0-9]+_mutex$' || continue ;;
            *) continue ;;
        esac
        # Belt and braces on top of the guard: never touch something a live
        # process has mapped, even if the guard was somehow satisfied.
        skip=0
        for pid in $mapped_pids; do
            if grep -qs "/dev/shm/$name\b" "/proc/$pid/maps" 2>/dev/null; then skip=1; break; fi
        done
        [ "$skip" -eq 1 ] && continue
        victims="$victims $name"
        victim_count=$((victim_count + 1))
    done

    echo "[shm_clean] conditions: $(date '+%Y-%m-%d %H:%M %Z'), $(count) fastrtps entries present"
    echo "[shm_clean] --ports would remove $victim_count orphaned port entr(y|ies):"
    for name in $victims; do echo "[shm_clean]   $name"; done
    if [ "$victim_count" -eq 0 ]; then
        echo "[shm_clean] nothing to sweep."
        exit 0
    fi
    if [ "$delete" -eq 0 ]; then
        echo "[shm_clean] DRY RUN -- nothing deleted. Re-run with '--ports --delete' to sweep."
        exit 0
    fi
    for name in $victims; do rm -f -- "/dev/shm/$name"; done
    echo "[shm_clean] swept. before: $before entries / after: $(count) entries"
    echo "[shm_clean] record the date and conditions with these numbers; a reclaim"
    echo "[shm_clean] figure without them is not comparable to the next one."
    exit 0
fi

if ! command -v fastdds >/dev/null 2>&1; then
    # fastdds ships with the ROS 2 installation; without it we do NOT fall back
    # to `rm -f /dev/shm/fastrtps*`, because that would also delete segments
    # belonging to live processes on the other domains.
    echo "[shm_clean] ERROR: 'fastdds' not on PATH -- source /opt/ros/jazzy/setup.bash first." >&2
    echo "[shm_clean] Refusing to rm /dev/shm/fastrtps* blindly: it would hit live sessions too." >&2
    exit 1
fi

echo "[shm_clean] before: $before entries"
fastdds shm clean
after="$(count)"
echo "[shm_clean] after:  $after entries"

if [ "$after" -gt 0 ]; then
    # NOT "they belong to live processes" -- measured 2026-08-20 on a quiet
    # machine, 94% of the remainder was orphaned port files and semaphores owned
    # by nothing alive, which this script does not reclaim. Saying otherwise
    # sends the reader away satisfied with a residue that is really still there.
    echo "[shm_clean] note: $after entries remain. Some belong to live processes and are"
    echo "[shm_clean] correctly spared; the rest is typically ORPHANED PORT entries"
    echo "[shm_clean] (fastrtps_port*, sem.fastrtps_port*_mutex) that THIS mode does not"
    echo "[shm_clean] reclaim -- they accumulate across sessions and outlive their processes."
    echo "[shm_clean] Re-running this will not shift those. To see them:"
    echo "[shm_clean]     scripts/shm_clean.sh --ports          (dry run, deletes nothing)"
    echo "[shm_clean] and to sweep them, with EVERYTHING ROS stopped on every domain:"
    echo "[shm_clean]     scripts/shm_clean.sh --ports --delete"
fi
