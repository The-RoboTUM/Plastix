#!/bin/bash
# Offline test harness for scripts/gripperx_desk.sh.
#
# What it is: a plain-bash test of the parts of the desk launcher that can be
# checked without a robot - the Pi stack settle wait before the local-mapping
# decision (incident 2026-10-16, Mode R restart = short press of the Pi's shutdown
# button), Ctrl-C during that wait, the source guard, and the teardown order
# (command sources before passive processes).
# Status: 2026-10-16. Run: bash Software/ros2/scripts/test/test_gripperx_desk.sh
#
# NOTHING HERE TOUCHES THE ROBOT OR ROS. The script under test is SOURCED (its
# source guard defines the functions and runs nothing; main is never run), and:
#   * `ssh` is a stub on PATH that runs the script's REAL ssh payload locally,
#     against stubs of systemctl / pgrep / docker, so the payload's quoting is
#     exercised and not only the parser. The stubs answer only the exact
#     properties / patterns the payload is expected to ask for and log anything
#     else to stub_errors, which a test requires to stay empty;
#   * REAL_PRELUDE is replaced, so every `ros2 ...` lands on a stub that logs the
#     call and prints a canned node list - no DDS traffic, no launch;
#   * the teardown tests signal only dummy python processes this harness starts.
# Do NOT point this at a gripperx_desk.sh without the source guard: sourcing that
# would run its real-mode main.
set -u

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DESK="${DESK_UNDER_TEST:-$HERE/../gripperx_desk.sh}"
T="$(mktemp -d)"
trap 'rm -rf "$T"' EXIT
STUB="$T/stub"
mkdir -p "$STUB"
export STUB_ERRORS="$T/stub_errors"
: >"$STUB_ERRORS"
PASS=0
FAIL=0

check() { # description condition-exit-status
    if [ "$2" -eq 0 ]; then
        PASS=$((PASS + 1)); echo "  PASS  $1"
    else
        FAIL=$((FAIL + 1)); echo "  FAIL  $1"
    fi
}

# --- stubs --------------------------------------------------------------------
# Scenario = numbered step directories under a scenario dir; the N-th ssh call
# answers from step N (the last step repeats). Per step:
#   units       "<unit> <is-active answer>" lines (missing unit -> inactive)
#   jobs        raw `systemctl list-jobs --no-legend` text
#   pgrep       "<pattern>\t<count>" lines, exact pgrep -f pattern (missing -> 0)
#   stackstop_a `pgrep -a` lines (PID + cmdline) for the stop-sequence pattern
#   map_age     seconds gripperx-mapping has been active (missing -> never)
# Scenario-level flags: fail_ssh (every ssh fails), slow_ssh (ssh hangs 30 s).
cat >"$STUB/ssh" <<'EOF'
#!/bin/bash
[ -f "$SCEN/fail_ssh" ] && exit 255
[ -f "$SCEN/slow_ssh" ] && exec sleep 30.123
n=$(( $(cat "$SCEN/count" 2>/dev/null || echo 0) + 1 ))
echo "$n" >"$SCEN/count"
last=$(ls "$SCEN" | grep -E '^[0-9]+$' | sort -n | tail -n1)
[ "$n" -gt "$last" ] && n=$last
export SCEN_CUR="$SCEN/$n"
payload="${@: -1}"
exec bash -c "$payload"
EOF
cat >"$STUB/systemctl" <<'EOF'
#!/bin/bash
bad() { echo "systemctl stub: unexpected call: $*" >>"$STUB_ERRORS"; exit 1; }
case "$1" in
    is-active)
        case "$2" in
            gripperx-agent|gripperx-bringup|gripperx-mapping|gripperx-navigation|gripperx-external) ;;
            *) bad "$@" ;;
        esac
        st=$(awk -v u="$2" '$1 == u { print $2 }' "$SCEN_CUR/units" 2>/dev/null)
        echo "${st:-inactive}" ;;
    list-jobs)
        [ "$*" = "list-jobs --no-legend" ] || bad "$@"
        cat "$SCEN_CUR/jobs" 2>/dev/null ;;
    show)
        [ "$*" = "show -p ActiveEnterTimestampMonotonic --value gripperx-mapping" ] || bad "$@"
        if [ -f "$SCEN_CUR/map_age" ]; then
            up=$(cut -d. -f1 /proc/uptime)
            echo $(( (up - $(cat "$SCEN_CUR/map_age")) * 1000000 ))
        else
            echo 0
        fi ;;
    *) bad "$@" ;;
esac
EOF
# Every pgrep -f pattern the payload sends, verbatim as the remote shell passes it.
cat >"$STUB/patterns" <<'EOF'
real_robot[.]launch[.]py
arm_action_serve[r]
steer_servo_nod[e]
swerve_cmd_nod[e]
joint_command_bridg[e]
teleop_mux_nod[e]
lidar_power_nod[e]
robot_state_publishe[r]
ros2_control_nod[e]
ldlidar_stl_ros2_nod[e]
scan_range_filte[r]
rf2o_laser_odometry_nod[e]
laser_scan_matche[r]
ekf_nod[e]
async_slam_toolbox_nod[e]
bin/gripperx-(stack-sto[p]|restar[t]|sto[p]|shutdow[n])( |$)
EOF
cat >"$STUB/pgrep" <<'EOF'
#!/bin/bash
mode="$1"; pat="${@: -1}"
if ! grep -qxF -- "$pat" "$(dirname "$0")/patterns"; then
    echo "pgrep stub: unexpected pattern: $pat" >>"$STUB_ERRORS"
    [ "$mode" = "-c" ] && echo 0
    exit 1
fi
if [ "$mode" = "-a" ]; then
    case "$pat" in bin/gripperx-*) cat "$SCEN_CUR/stackstop_a" 2>/dev/null ;; esac
    exit 0
fi
c=$(awk -F'\t' -v p="$pat" '$1 == p { print $2 }' "$SCEN_CUR/pgrep" 2>/dev/null)
echo "${c:-0}"
EOF
cat >"$STUB/docker" <<'EOF'
#!/bin/bash
echo "Up 1 hour"
EOF
# ros2: "node list" prints $ROS2_NODES (default: nothing), "topic list" a
# plausible live-view set, "launch" fails. Every call is logged to $ROS2_LOG.
cat >"$STUB/ros2" <<'EOF'
#!/bin/bash
echo "$*" >>"${ROS2_LOG:-/dev/null}"
case "$1 $2" in
    "node list")  printf '%s' "${ROS2_NODES:-}" ;;
    "topic list") printf '/map\n/scan\n/robot_description\n' ;;
    *) exit 99 ;;
esac
EOF
chmod +x "$STUB"/ssh "$STUB"/systemctl "$STUB"/pgrep "$STUB"/docker "$STUB"/ros2

P_STACKSTOP='bin/gripperx-(stack-sto[p]|restar[t]|sto[p]|shutdow[n])( |$)'
P_SLAM='async_slam_toolbox_nod[e]'
P_LSM='laser_scan_matche[r]'
P_EKF='ekf_nod[e]'

step() { # scenario step  -> creates and echoes the step dir
    mkdir -p "$1/$2"; echo "$1/$2"
}
units_all() { # dir state
    local u
    for u in agent bringup mapping navigation external; do echo "gripperx-$u $2"; done >"$1/units"
}
mapping_nodes_up() { # dir
    printf '%s\t1\n%s\t1\n%s\t1\n' "$P_SLAM" "$P_LSM" "$P_EKF" >>"$1/pgrep"
}
stackstop_running() { # dir cmdline...   (default: the Mode R pair)
    local d="$1"; shift
    [ "$#" -gt 0 ] || set -- \
        "/usr/bin/sudo -n /usr/local/bin/gripperx-stack-stop --mode=restart --approve-motion" \
        "/bin/bash /usr/local/bin/gripperx-stack-stop --mode=restart --approve-motion"
    printf '%s\t%s\n' "$P_STACKSTOP" "$#" >>"$d/pgrep"
    local pid=2571 c
    : >"$d/stackstop_a"
    for c in "$@"; do echo "$pid $c" >>"$d/stackstop_a"; pid=$((pid + 1)); done
}

# Runs pi_health_check in a subshell against scenario $1 and prints, after the
# script's own output, marker lines the tests grep for.
run_health() { # scenario [extra assignments...]
    local scen="$1"; shift
    (
        export SCEN="$scen"
        export PATH="$STUB:$PATH"
        export ROS2_LOG="$scen/ros2_calls"
        # shellcheck source=/dev/null
        source "$DESK"
        REAL_PRELUDE="export PATH='$STUB':\$PATH;"
        # Virtual clock: the settle wait measures with $SECONDS and paces with
        # interruptible_sleep, so a fake that advances SECONDS makes a 180 s bound
        # a sub-second test without changing what is measured.
        interruptible_sleep() { SECONDS=$((SECONDS + ${1%.*})); }
        sleep() { SECONDS=$((SECONDS + ${1%.*})); }
        for a in "$@"; do eval "$a"; done
        pi_health_check
        echo "@@RC=$?"
        echo "@@EVIDENCE=$PI_MAPPING_EVIDENCE"
        echo "@@LEFT=$PI_TRANSITION_LEFT"
        echo "@@SSH_CALLS=$(cat "$scen/count" 2>/dev/null)"
        if [ -n "${RUN_LOCAL_MAP:-}" ]; then
            start_local_mapping
            echo "@@LOCALMAP_RC=$?"
            echo "@@LOCALMAP_STATE=$LOCAL_MAP_STATE"
        fi
    ) 2>&1
}
field() { sed -n "s/^@@$2=//p" <<<"$1"; }
ros2_called() { # scenario word -> 0 if a logged ros2 call starts with it
    grep -q "^$2" "$1/ros2_calls" 2>/dev/null
}

echo "== settle wait / mapping evidence =="

# 1. Incident replay, 2026-10-16: desk started during the stop sequence (all
#    units inactive, only the stack-stop process tells), then the start phase
#    (mapping activating in its ExecStartPre sleep, navigation's start job
#    waiting), then settled with the full mapping node set.
S="$T/s1"
d=$(step "$S" 1); units_all "$d" inactive; stackstop_running "$d"
d=$(step "$S" 2); units_all "$d" active
    sed -i 's/^gripperx-mapping .*/gripperx-mapping activating/; s/^gripperx-navigation .*/gripperx-navigation inactive/; s/^gripperx-external .*/gripperx-external inactive/' "$d/units"
    stackstop_running "$d"
    printf '%s\n' "4711 gripperx-navigation.service start waiting" >"$d/jobs"
d=$(step "$S" 3); units_all "$d" active; mapping_nodes_up "$d"; echo 30 >"$d/map_age"
o=$(run_health "$S")
check "incident: transition announced (stack-stop seen while units read inactive)" \
      "$(grep -q 'IN TRANSITION: stop sequence running (gripperx-stack-stop, mode: restart)' <<<"$o"; echo $?)"
check "incident: queued job and activating unit reported while waiting" \
      "$(grep -q 'gripperx-navigation.service:start:waiting' <<<"$o" && grep -q 'gripperx-mapping activating' <<<"$o"; echo $?)"
check "incident: settles and re-evaluates" "$(grep -q 'Pi stack settled after' <<<"$o"; echo $?)"
check "incident: evidence 'active' after settling (no local stack)" "$([ "$(field "$o" EVIDENCE)" = active ]; echo $?)"
check "incident: exactly 3 probes" "$([ "$(field "$o" SSH_CALLS)" = 3 ]; echo $?)"

# 2. The pre-change behaviour that must survive: a steady, genuinely inactive Pi
#    (no stack-stop, no job, nothing activating) -> no wait, "inactive", the
#    local stack passes the Pi gates (the stub DDS graph then stops it).
S="$T/s2"
d=$(step "$S" 1); units_all "$d" active
    sed -i 's/^gripperx-mapping .*/gripperx-mapping inactive/; s/^gripperx-navigation .*/gripperx-navigation inactive/' "$d/units"
o=$(run_health "$S" 'export ROS2_NODES=/slam_toolbox' 'RUN_LOCAL_MAP=1')
check "steady inactive: no wait" "$(! grep -q 'IN TRANSITION' <<<"$o"; echo $?)"
check "steady inactive: one probe" "$([ "$(field "$o" SSH_CALLS)" = 1 ]; echo $?)"
check "steady inactive: evidence 'inactive'" "$([ "$(field "$o" EVIDENCE)" = inactive ]; echo $?)"
check "steady inactive: local mapping passes the Pi gates (stopped only by the DDS-graph check)" \
      "$([ "$(field "$o" LOCALMAP_STATE)" = 'skipped (mapping node already in DDS graph)' ]; echo $?)"

# 3. Bounded: the stop sequence never ends -> gives up at the bound, evidence
#    "unsettled", local mapping refused BEFORE any DDS probe or launch;
#    --force-local-map passes the refusal.
S="$T/s3"
d=$(step "$S" 1); units_all "$d" inactive; stackstop_running "$d"
o=$(run_health "$S" 'PI_SETTLE_MAX_SEC=20' 'PI_SETTLE_POLL_SEC=5' 'RUN_LOCAL_MAP=1')
check "timeout: gives up with a message" "$(grep -qE 'still in transition after 2[0-9] s' <<<"$o"; echo $?)"
check "timeout: progress lines while waiting" "$([ "$(grep -c 'waiting for the Pi stack' <<<"$o")" -ge 3 ]; echo $?)"
check "timeout: evidence 'unsettled'" "$([ "$(field "$o" EVIDENCE)" = unsettled ]; echo $?)"
check "timeout: reason kept" "$(grep -q '^@@LEFT=stop sequence running' <<<"$o"; echo $?)"
check "timeout: health-check summary says SKIPPED" "$(grep -q 'will be SKIPPED - the Pi stack has not settled' <<<"$o"; echo $?)"
check "timeout: local mapping refused (rc 1, no ros2 node list, no ros2 launch)" \
      "$([ "$(field "$o" LOCALMAP_RC)" = 1 ] && ! ros2_called "$S" 'node list' && ! ros2_called "$S" launch; echo $?)"
check "timeout: refusal state recorded" \
      "$([ "$(field "$o" LOCALMAP_STATE)" = 'skipped (Pi stack unsettled - refused to guess)' ]; echo $?)"
rm -f "$S/count" "$S/ros2_calls"
o=$(run_health "$S" 'PI_SETTLE_MAX_SEC=5' 'PI_SETTLE_POLL_SEC=5' 'LOCAL_MAP=force' 'export ROS2_NODES=/slam_toolbox' 'RUN_LOCAL_MAP=1')
check "force: passes the unsettled refusal (stopped only by the DDS-graph check)" \
      "$([ "$(field "$o" LOCALMAP_STATE)" = 'skipped (mapping node already in DDS graph)' ] && ros2_called "$S" 'node list'; echo $?)"

# 4. Mapping active for 10 s with slam_toolbox not up yet -> waited for, then active.
S="$T/s4"
d=$(step "$S" 1); units_all "$d" active; printf '%s\t1\n' "$P_EKF" >"$d/pgrep"; echo 10 >"$d/map_age"
d=$(step "$S" 2); units_all "$d" active; mapping_nodes_up "$d"; echo 15 >"$d/map_age"
o=$(run_health "$S")
check "mapping coming up: waited for" "$(grep -q 'gripperx-mapping active for 10s, its nodes not all up yet' <<<"$o"; echo $?)"
check "mapping coming up: evidence 'active'" "$([ "$(field "$o" EVIDENCE)" = active ]; echo $?)"

# 5. Mapping active for 10 minutes with slam_toolbox missing -> not waited for,
#    but no longer "inactive" (that let the local stack start until 2026-10-16).
S="$T/s5"
d=$(step "$S" 1); units_all "$d" active; printf '%s\t1\n' "$P_EKF" >"$d/pgrep"; echo 600 >"$d/map_age"
o=$(run_health "$S" 'RUN_LOCAL_MAP=1')
check "mapping stuck half-up: no wait" "$(! grep -q 'IN TRANSITION' <<<"$o"; echo $?)"
check "mapping stuck half-up: evidence 'unsettled'" "$([ "$(field "$o" EVIDENCE)" = unsettled ]; echo $?)"
check "mapping stuck half-up: local mapping refused before any DDS probe" \
      "$([ "$(field "$o" LOCALMAP_RC)" = 1 ] && ! ros2_called "$S" 'node list'; echo $?)"

# 6. A crash-looping mapping unit (Restart=on-failure reads `activating`) -> timeout.
S="$T/s6"
d=$(step "$S" 1); units_all "$d" active; sed -i 's/^gripperx-mapping .*/gripperx-mapping activating/' "$d/units"
o=$(run_health "$S" 'PI_SETTLE_MAX_SEC=10' 'PI_SETTLE_POLL_SEC=5')
check "crash loop: evidence 'unsettled'" "$([ "$(field "$o" EVIDENCE)" = unsettled ]; echo $?)"

# 7. ssh down -> unchanged: return 1, evidence stays 'unknown'.
S="$T/s7"
d=$(step "$S" 1); touch "$S/fail_ssh"
o=$(run_health "$S")
check "ssh down: rc 1" "$([ "$(field "$o" RC)" = 1 ]; echo $?)"
check "ssh down: evidence 'unknown'" "$([ "$(field "$o" EVIDENCE)" = unknown ]; echo $?)"

# 8. Garbage in the settle env -> falls back to the default, does not wait for ever.
o=$(GRIPPERX_DESK_PI_SETTLE_SEC=abc bash -c "source '$DESK'; echo \"@@MAX=\$PI_SETTLE_MAX_SEC\"")
check "non-numeric GRIPPERX_DESK_PI_SETTLE_SEC falls back to 180" "$([ "$(field "$o" MAX)" = 180 ]; echo $?)"

echo "== stop/halt sequences (nothing comes back) =="

# 9. Mode H (long press): --mode=halt only -> announced, NOT waited for; the
#    units are all inactive by now, so the ordinary evidence rules decide.
S="$T/s9"
d=$(step "$S" 1); units_all "$d" inactive
    stackstop_running "$d" "/usr/bin/sudo -n /usr/local/bin/gripperx-stack-stop --mode=halt" \
                           "/bin/bash /usr/local/bin/gripperx-stack-stop --mode=halt"
o=$(run_health "$S")
check "halt: announced as STOPPED, mode halt" "$(grep -q 'Pi stack is being STOPPED (gripperx-stack-stop, mode: halt)' <<<"$o"; echo $?)"
check "halt: no transition wait, one probe" "$(! grep -q 'IN TRANSITION' <<<"$o" && [ "$(field "$o" SSH_CALLS)" = 1 ]; echo $?)"
check "halt: evidence 'inactive' (Pi will not map again by itself)" "$([ "$(field "$o" EVIDENCE)" = inactive ]; echo $?)"

# 10. halt while units are still deactivating -> waits for the deactivation only.
S="$T/s10"
d=$(step "$S" 1); units_all "$d" inactive; sed -i 's/^gripperx-bringup .*/gripperx-bringup deactivating/' "$d/units"
    stackstop_running "$d" "/bin/bash /usr/local/bin/gripperx-stack-stop --mode=halt"
d=$(step "$S" 2); units_all "$d" inactive
    stackstop_running "$d" "/bin/bash /usr/local/bin/gripperx-stack-stop --mode=halt"
o=$(run_health "$S")
check "halt + deactivating: waits for the unit, not for the stack-stop" \
      "$(grep -q 'IN TRANSITION: gripperx-bringup deactivating$' <<<"$o" && [ "$(field "$o" SSH_CALLS)" = 2 ]; echo $?)"

# 11. Mode R as it really looks while bringup stops: the outer --mode=restart plus
#     bringup's own ExecStop (--mode=stop --from-unit) -> still a restart, waited for.
S="$T/s11"
d=$(step "$S" 1); units_all "$d" inactive
    stackstop_running "$d" "/bin/bash /usr/local/bin/gripperx-stack-stop --mode=restart --approve-motion" \
                           "/bin/bash /usr/local/bin/gripperx-stack-stop --mode=stop --from-unit"
o=$(run_health "$S" 'PI_SETTLE_MAX_SEC=5' 'PI_SETTLE_POLL_SEC=5')
check "restart + ExecStop stop: waited for (mode: restart stop)" \
      "$(grep -q 'IN TRANSITION: stop sequence running (gripperx-stack-stop, mode: restart stop)' <<<"$o" && ! grep -q 'being STOPPED' <<<"$o"; echo $?)"

# 12. Operator names (symlinks to gripperx-stack-stop, dispatch on $0).
S="$T/s12"
d=$(step "$S" 1); units_all "$d" inactive; stackstop_running "$d" "/bin/bash /usr/local/bin/gripperx-restart"
o=$(run_health "$S" 'PI_SETTLE_MAX_SEC=5' 'PI_SETTLE_POLL_SEC=5')
check "gripperx-restart (no --mode): seen as restart, waited for" \
      "$(grep -q 'stop sequence running (gripperx-stack-stop, mode: restart)' <<<"$o"; echo $?)"
S="$T/s13"
d=$(step "$S" 1); units_all "$d" inactive; stackstop_running "$d" "/bin/bash /usr/local/bin/gripperx-shutdown"
o=$(run_health "$S")
check "gripperx-shutdown (no --mode): seen as halt, not waited for" \
      "$(grep -q 'being STOPPED (gripperx-stack-stop, mode: halt)' <<<"$o" && ! grep -q 'IN TRANSITION' <<<"$o"; echo $?)"
S="$T/s14"
d=$(step "$S" 1); units_all "$d" inactive; stackstop_running "$d" "/bin/bash /usr/local/bin/gripperx-stop --mode=restart"
o=$(run_health "$S" 'PI_SETTLE_MAX_SEC=5' 'PI_SETTLE_POLL_SEC=5')
check "explicit --mode wins over the name (gripperx-stop --mode=restart -> restart)" \
      "$(grep -q 'stop sequence running (gripperx-stack-stop, mode: restart)' <<<"$o"; echo $?)"
S="$T/s15"
d=$(step "$S" 1); units_all "$d" inactive; stackstop_running "$d" "/bin/bash /usr/local/bin/gripperx-stack-stop"
o=$(run_health "$S" 'PI_SETTLE_MAX_SEC=5' 'PI_SETTLE_POLL_SEC=5')
check "unreadable mode is treated like a restart (waited for)" \
      "$(grep -q 'stop sequence running (gripperx-stack-stop, mode: ?)' <<<"$o"; echo $?)"

echo "== Ctrl-C during the settle wait =="
# Runs in a job-control subshell (set -m), because a background job of a
# non-interactive shell starts with SIGINT ignored and bash cannot trap it then.
# REAL time here (no virtual clock): the point is how fast the run stops.
# "start_real" is a stub that only leaves a marker - it stands for "the run went
# on and started RViz/teleop/leader".
sigint_run() { # scenario  -> prints rc, seconds to exit, marker state
    local scen="$1"
    (
        set -m
        (
            export SCEN="$scen" PATH="$STUB:$PATH"
            source "$DESK"
            install_traps
            # >= 4 s on purpose: the INT lands 2 s into a poll sleep, so a
            # foreground (uninterruptible) sleep would hold the trap back ~3 s.
            PI_SETTLE_POLL_SEC=5
            start_real() { : >"$scen/STARTED"; }
            pi_health_check
            start_real
        ) >"$scen/out" 2>&1 &
        local pid=$! t0
        sleep 2
        t0=$(date +%s.%N)
        kill -INT "$pid"
        wait "$pid"
        echo "@@RC=$?"
        echo "@@EXIT_SEC=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')"
    )
    echo "@@STARTED=$([ -f "$scen/STARTED" ] && echo yes || echo no)"
}
S="$T/s20"
d=$(step "$S" 1); units_all "$d" inactive; stackstop_running "$d"
o=$(sigint_run "$S" 2>/dev/null)
check "SIGINT in the settle wait: exit status 130" "$([ "$(field "$o" RC)" = 130 ]; echo $?)"
check "SIGINT in the settle wait: nothing started afterwards" "$([ "$(field "$o" STARTED)" = no ]; echo $?)"
check "SIGINT in the settle wait: exits within 2 s ($(field "$o" EXIT_SEC) s)" \
      "$(awk -v s="$(field "$o" EXIT_SEC)" 'BEGIN { exit !(s < 2.0) }'; echo $?)"
check "SIGINT in the settle wait: says so" "$(grep -q 'interrupted - stopping' "$S/out"; echo $?)"

S="$T/s21"
d=$(step "$S" 1); units_all "$d" inactive; touch "$S/slow_ssh"
o=$(sigint_run "$S" 2>/dev/null)
sleep 0.5
check "SIGINT during a hanging ssh probe: exit 130 within 2 s ($(field "$o" EXIT_SEC) s)" \
      "$([ "$(field "$o" RC)" = 130 ] && awk -v s="$(field "$o" EXIT_SEC)" 'BEGIN { exit !(s < 2.0) }'; echo $?)"
check "SIGINT during a hanging ssh probe: the probe is killed, nothing started" \
      "$(! pgrep -f 'sleep 30[.]123' >/dev/null && [ "$(field "$o" STARTED)" = no ]; echo $?)"
pkill -f 'sleep 30[.]123' 2>/dev/null

echo "== source guard =="
# The guard must fire only when sourced. Executed as "bash file", "bash < file"
# and bash -c "$(cat file)", main must run WITH its traps: the copy below replaces
# main's first line by a print of the EXIT trap and an exit.
probe_main() { sed 's/^MODE="real"$/trap -p EXIT; echo @@MAIN_RAN; exit 0/' "$DESK"; }
probe_main >"$T/desk_probe.sh"
o=$(bash "$T/desk_probe.sh" 2>&1)
check "bash file: main runs with the EXIT trap" "$(grep -q '@@MAIN_RAN' <<<"$o" && grep -q "trap -- 'teardown' EXIT" <<<"$o"; echo $?)"
o=$(bash <"$T/desk_probe.sh" 2>&1)
check "bash < file: main runs with the EXIT trap" "$(grep -q '@@MAIN_RAN' <<<"$o" && grep -q "trap -- 'teardown' EXIT" <<<"$o"; echo $?)"
# (bash -c "$(cat file)" itself no longer works at all: the file is beyond the
# kernel's 128 KiB limit for one argument. eval in a bash -c is the same case -
# no BASH_SOURCE, top-level return not allowed.)
o=$(bash -c 'eval "$(cat "$1")"' desk "$T/desk_probe.sh" 2>&1)
check "bash -c 'eval \"\$(cat file)\"': main runs with the EXIT trap" "$(grep -q '@@MAIN_RAN' <<<"$o" && grep -q "trap -- 'teardown' EXIT" <<<"$o"; echo $?)"
o=$(bash -c "source '$T/desk_probe.sh'; echo @@AFTER_SOURCE; trap -p EXIT" 2>&1)
check "sourced: main does not run, no traps left behind" \
      "$(! grep -q '@@MAIN_RAN' <<<"$o" && grep -q '@@AFTER_SOURCE' <<<"$o" && ! grep -q 'teardown' <<<"$o"; echo $?)"

echo "== teardown =="
# Dummy processes whose cmdline carries the tracked pattern; each logs its label
# into $ORDER when SIGINT (or SIGTERM) arrives. Python, because a background job
# of a non-interactive shell starts with SIGINT ignored and bash cannot un-ignore it.
ORDER="$T/order"
dummy() { # label [ignore_int]
    local ign="${2:-0}"
    python3 -c "
import signal, sys, time
LABEL = '$1'
def h(*_):
    if $ign:
        return
    open('$ORDER', 'a').write(LABEL + '\n'); sys.exit(0)
signal.signal(signal.SIGINT, h)
signal.signal(signal.SIGTERM, lambda *_: (open('$ORDER', 'a').write(LABEL + ':TERM\n'), sys.exit(0)))
while True:
    time.sleep(0.05)
" >/dev/null 2>&1 &
    echo $!
}
o=$( {
    source "$DESK"
    p_map=$(dummy "gripperx_bringup mapping.launch.py")
    p_rviz=$(dummy "rviz2")
    p_tnode=$(dummy "keyboard_teleop_node")
    p_tlaunch=$(dummy "gripperx_teleop laptop_teleop.launch.py")
    p_dead=$(dummy "web_teleop_node")
    p_leader=$(dummy "leader_arm_node" 1)
    sleep 0.5
    kill -KILL "$p_dead"
    # Same calls, same order as the real start sequence (mapping, RViz, teleop,
    # leader); the dead one stands for a teleop node that crashed before teardown.
    track "local mapping launch" "$p_map" "gripperx_bringup mapping.launch.py"
    track "real RViz" "$p_rviz" "rviz2"
    track "real teleop node" "$p_tnode" "keyboard_teleop_node" "" "" node
    track "real teleop launch" "$p_tlaunch" "gripperx_teleop laptop_teleop.launch.py" "" "" launch
    track "real teleop node (web)" "$p_dead" "web_teleop_node" "" "" node
    track "leader arm node" "$p_leader" "leader_arm_node" 1 "killed note" command
    teardown
    echo "@@DONE=1"
} 2>&1 )
got=$(tr '\n' ',' <"$ORDER")
echo "  signal order seen: $got"
check "teardown: teleop node, teleop launch, leader (SIGTERM), then mapping, RViz" \
      "$([ "$got" = 'keyboard_teleop_node,gripperx_teleop laptop_teleop.launch.py,leader_arm_node:TERM,gripperx_bringup mapping.launch.py,rviz2,' ]; echo $?)"
check "teardown: dead node reported, teardown completed" \
      "$(grep -q 'real teleop node (web) (PID [0-9]*) already gone' <<<"$o" && grep -q '^@@DONE=1' <<<"$o" && grep -q 'Teardown complete' <<<"$o"; echo $?)"
check "teardown: escalation warned for the leader" "$(grep -q 'leader arm node (PID [0-9]*) still running 1 s after SIGINT - SIGTERM' <<<"$o"; echo $?)"

# Web shape (start_teleop_web): the LAUNCH is tracked before the node, as in the
# real start sequence. The node must still be signalled first, and a launch that
# ignores SIGINT (as a backgrounded "bash -c ...; exec ros2 launch" does) must be
# sent SIGTERM at once instead of waiting out the SIGINT grace on a no-op.
# Measured 2026-10-22: such a launch dies on SIGTERM without forwarding anything.
dummy_ign() { # label -> python dummy that starts with SIGINT ignored (SigIgn bit), TERM logged
    python3 -c "
import signal, sys, time
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, lambda *_: (open('$ORDER', 'a').write('$1:TERM\n'), sys.exit(0)))
while True:
    time.sleep(0.05)
" >/dev/null 2>&1 &
    echo $!
}
: >"$ORDER"
o=$( {
    source "$DESK"
    p_wl=$(dummy_ign "gripperx_teleop web_teleop.launch.py")
    p_wn=$(dummy "web_teleop_node")
    sleep 0.5
    track "real teleop launch (web)" "$p_wl" "gripperx_teleop web_teleop.launch.py" "" "" launch
    track "real teleop node (web)" "$p_wn" "web_teleop_node" "" "" node
    t0=$(date +%s.%N)
    teardown
    echo "@@SEC=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')"
} 2>&1 )
got=$(tr '\n' ',' <"$ORDER")
echo "  signal order seen: $got"
check "teardown (web shape): node is signalled before its launch, though tracked after it" \
      "$([ "$got" = 'web_teleop_node,gripperx_teleop web_teleop.launch.py:TERM,' ]; echo $?)"
check "teardown (web shape): SIGINT-ignoring launch gets SIGTERM at once, no 3 s SIGINT wait" \
      "$(grep -q 'teleop launch (web) (PID [0-9]*) ignores SIGINT - SIGTERM at once' <<<"$o" \
         && awk -v s="$(sed -n 's/^@@SEC=//p' <<<"$o")" 'BEGIN { exit !(s < 1.5) }'; echo $?)"
check "teardown (web shape): no 'still running after SIGINT' warning for the launch" \
      "$(! grep -q 'launch (web) (PID [0-9]*) still running' <<<"$o"; echo $?)"

# The SIGTERM-at-once shortcut is for LAUNCH entries only. A node or the leader
# started by a bare exec under "bash -c ... &" also begins with SIGINT ignored,
# until rclpy installs its handler; it must still get SIGINT and its grace
# (the leader's torque-off).
#
# Observable here: stop_tracked lands while SIGINT is still ignored (a SIGINT
# sent then is lost - existing behaviour, third audit finding 1), so the entry
# must sit out its full SIGINT grace and only then get SIGTERM. The shortcut
# would TERM it at once and log "ignores SIGINT".
for stg in command node; do
    : >"$ORDER"
    o=$( {
        source "$DESK"
        p=$(dummy_ign "late_$stg")
        sleep 0.3
        track "late $stg" "$p" "late_$stg" 1 "killed note" "$stg"
        t0=$(date +%s.%N)
        stop_tracked 0
        echo "@@SEC=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')"
    } 2>&1 )
    check "stop_tracked: $stg-stage entry with SIGINT ignored is not short-cut (full SIGINT grace, then TERM)" \
          "$([ "$(tr '\n' ',' <"$ORDER")" = "late_$stg:TERM," ] && ! grep -q 'ignores SIGINT' <<<"$o" \
             && awk -v s="$(sed -n 's/^@@SEC=//p' <<<"$o")" 'BEGIN { exit !(s >= 1.0) }'; echo $?)"
done

# The SIGTERM wait leaves as soon as the process is gone (not a flat 2 s).
: >"$ORDER"
o=$( {
    source "$DESK"
    p=$(dummy_ign "term_only")
    sleep 0.3
    track "term only" "$p" "term_only" 0 "" launch
    t0=$(date +%s.%N)
    stop_tracked 0
    echo "@@SEC=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')"
} 2>&1 )
check "stop_tracked: SIGTERM poll returns as soon as the process is gone (< 1 s, bound is 2 s)" \
      "$(awk -v s="$(sed -n 's/^@@SEC=//p' <<<"$o")" 'BEGIN { exit !(s < 1.0) }'; echo $?)"

# An early teardown with nothing tracked must not be "spent": what is tracked
# after it is still torn down by the next call (the EXIT trap in a real run).
: >"$ORDER"
o=$( {
    source "$DESK"
    teardown
    p=$(dummy "rviz2")
    sleep 0.3
    track "real RViz" "$p" "rviz2"
    teardown
} 2>&1 )
check "teardown: an empty early call does not leave later processes unowned" \
      "$([ "$(tr '\n' ',' <"$ORDER")" = 'rviz2,' ]; echo $?)"

echo "== Ctrl-C during teardown and in start-up windows =="
# Generic: run $2 (shell code) in a job-control subshell with the script sourced
# and its traps installed, SIGINT that subshell's PID after $1 s (twice, 0.3 s
# apart, if $3 = twice), and report its exit status.
export DUMMY_ORDER="$T/order2"
mkdummy() { # path label [ignore_int] -> executable python stand-in at path
    cat >"$1" <<EOF
#!/usr/bin/env python3
import os, signal, sys, time
LABEL = '$2'
# As "ros2": the CLI queries (node/topic list, lifecycle) answer at once with
# nothing; only a "launch" stays up as the long-running process under test.
if sys.argv[1:2] in (['node'], ['topic'], ['lifecycle']):
    sys.exit(0)
def note(s):
    open(os.environ['DUMMY_ORDER'], 'a').write(s + '\n')
def on_int(*_):
    if $3:
        return
    note(LABEL); sys.exit(0)
signal.signal(signal.SIGINT, on_int)
signal.signal(signal.SIGTERM, lambda *_: (note(LABEL + ':TERM'), sys.exit(0)))
while True:
    time.sleep(0.05)
EOF
    chmod +x "$1"
}
D="$T/dummies"
mkdir -p "$D/bin" "$D/ws/install/gripperx_leader/lib/gripperx_leader" "$D/ws/install/gripperx_leader/share/gripperx_leader/config"
mkdummy "$D/bin/rviz2" rviz2 0
mkdummy "$D/bin/ros2" "ros2 launch" 0
mkdummy "$D/bin/keyboard_teleop_node" keyboard_teleop_node 0
mkdummy "$D/ws/install/gripperx_leader/lib/gripperx_leader/leader_arm_node" leader_arm_node 0
mkdummy "$D/slow_leader_arm_node" slow_leader_arm_node 1
: >"$D/ws/install/gripperx_leader/share/gripperx_leader/config/leader_calibration.yaml"
: >"$D/fake_leader_dev"
# Stand-in terminal emulator: starts the "teleop node" 2 s later, on domain 199.
cat >"$D/fake_term" <<EOF
#!/bin/bash
( sleep 2; ROS_DOMAIN_ID=199 exec "$D/bin/keyboard_teleop_node" ) >/dev/null 2>&1 &
EOF
chmod +x "$D/fake_term"
TPRE="export PATH='$D/bin':\$PATH;"
sigint_job() { # delay code [twice]
    local delay="$1" code="$2" twice="${3:-}"
    : >"$DUMMY_ORDER"
    (
        set -m
        (
            source "$DESK"
            install_traps
            eval "$code"
            echo "@@AFTER=1"
        ) >"$T/job_out" 2>&1 &
        local pid=$! t0
        sleep "$delay"
        t0=$(date +%s.%N)
        kill -INT "$pid"
        if [ -n "$twice" ]; then sleep 0.3; kill -INT "$pid"; fi
        wait "$pid"
        echo "@@RC=$?"
        echo "@@EXIT_SEC=$(awk -v a="$t0" -v b="$(date +%s.%N)" 'BEGIN { printf "%.1f", b - a }')"
    ) 2>/dev/null
    sleep 0.3
}
order2() { tr '\n' ',' <"$DUMMY_ORDER"; }
alive() { pgrep -f "$D/" >/dev/null; }

# 1. The regression of b3958a7: Ctrl-C (twice) DURING an ordinary teardown, while
#    the slow leader stand-in (ignores SIGINT) sits in its 3 s grace period.
o=$(sigint_job 1 "
    p_map=\$($D/bin/ros2 launch gripperx_bringup mapping.launch.py >/dev/null 2>&1 & echo \$!)
    p_rviz=\$($D/bin/rviz2 >/dev/null 2>&1 & echo \$!)
    p_lead=\$($D/slow_leader_arm_node >/dev/null 2>&1 & echo \$!)
    sleep 0.3
    track 'local mapping launch' \"\$p_map\" 'gripperx_bringup mapping.launch.py'
    track 'real RViz' \"\$p_rviz\" rviz2
    track 'leader arm node' \"\$p_lead\" slow_leader_arm_node 3 'killed note' command
    teardown
" twice)
echo "  order: $(order2)  rc=$(field "$o" RC)"
check "Ctrl-C x2 during teardown: teardown still finishes (leader TERM, then mapping, RViz)" \
      "$([ "$(order2)" = 'slow_leader_arm_node:TERM,ros2 launch,rviz2,' ]; echo $?)"
check "Ctrl-C x2 during teardown: nothing left alive" "$(! alive; echo $?)"
check "Ctrl-C x2 during teardown: exits 130 afterwards, does not carry on" \
      "$([ "$(field "$o" RC)" = 130 ] && ! grep -q '@@AFTER' "$T/job_out"; echo $?)"
pkill -f "$D/" 2>/dev/null

# 2. Ctrl-C inside RViz's start-up check (sleep 1 after the launch).
o=$(sigint_job 0.4 "start_rviz \"$TPRE\" /nonexistent.rviz 199 test")
check "Ctrl-C in RViz start-up: RViz stopped" "$([ "$(order2)" = 'rviz2,' ] && ! alive; echo $?)"
pkill -f "$D/" 2>/dev/null

# 3. Ctrl-C inside local mapping's start-up check (sleep 3 after the launch).
o=$(sigint_job 1 "REAL_PRELUDE=\"$TPRE\"; PI_MAPPING_EVIDENCE=inactive; LOCAL_MAP=force
    pids_for_pattern_on_domain() { :; }
    start_local_mapping")
check "Ctrl-C in local mapping start-up: launch stopped" "$([ "$(order2)" = 'ros2 launch,' ] && ! alive; echo $?)"
pkill -f "$D/" 2>/dev/null

# 4. Ctrl-C inside the leader's 15 s ready poll (the stand-in never says ready).
o=$(sigint_job 1 "WS='$D/ws'; LEADER_DEV='$D/fake_leader_dev'
    LEADER_CALIBRATION='$D/ws/install/gripperx_leader/share/gripperx_leader/config/leader_calibration.yaml'
    LEADER=on
    start_leader '' 199")
check "Ctrl-C in leader start-up poll: leader stopped" "$([ "$(order2)" = 'leader_arm_node,' ] && ! alive; echo $?)"
pkill -f "$D/" 2>/dev/null

# 5. Ctrl-C inside the keyboard teleop poll, BEFORE its node exists (it appears
#    2 s after the launch): teardown waits for it and stops it.
o=$(sigint_job 0.5 "TERM_CMD='$D/fake_term'; start_teleop_keyboard \"$TPRE\" 199 test twin")
check "Ctrl-C before the keyboard teleop node exists: claimed and stopped by teardown" \
      "$([ "$(order2)" = 'keyboard_teleop_node,' ] && ! alive && grep -q 'launch still pending' "$T/job_out"; echo $?)"
pkill -f "$D/" 2>/dev/null

echo "== stubs =="
check "no unexpected systemctl property / pgrep pattern was asked for" \
      "$([ ! -s "$STUB_ERRORS" ]; echo $?)"
[ -s "$STUB_ERRORS" ] && sed 's/^/    /' "$STUB_ERRORS"

echo
echo "passed: $PASS  failed: $FAIL"
[ "$FAIL" -eq 0 ]
