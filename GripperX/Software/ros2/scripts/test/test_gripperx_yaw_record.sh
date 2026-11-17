#!/bin/bash
# Offline test harness for scripts/gripperx_yaw_record.sh.
#
# What it is: a plain-bash test of the parts of the yaw recorder that can be checked
# without a robot - argument parsing, directory/bag naming, the pre-flight report,
# --strict abort on a silent required topic, --preflight-only creating nothing, and
# the meta.yaml content. Status: 2026-10-22. Run:
#   bash Software/ros2/scripts/test/test_gripperx_yaw_record.sh
#
# NOTHING HERE TOUCHES THE ROBOT OR ROS: GRIPPERX_YAW_SKIP_ENV skips sourcing ROS,
# GRIPPERX_YAW_PROBE is a stub that answers a canned rate list (a topic named in
# $STUB_SILENT is reported "none"), GRIPPERX_YAW_ROS2 is a stub that only logs
# `bag record` and creates the bag directory, and `ssh` is a stub on PATH that
# answers a canned "<epoch> <rev> <dirty>" line (or fails when $STUB_NO_SSH is set).
set -u
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT="$HERE/../gripperx_yaw_record.sh"
T="$(mktemp -d)"; trap 'rm -rf "$T"' EXIT
STUB="$T/stub"; mkdir -p "$STUB" "$T/bags"
PASS=0; FAIL=0
check() { if [ "$2" -eq 0 ]; then PASS=$((PASS+1)); echo "  PASS  $1"; else FAIL=$((FAIL+1)); echo "  FAIL  $1"; fi; }

cat >"$STUB/probe" <<'S'
#!/bin/bash
shift; shift   # window, latched list
for t in "$@"; do
    case " ${STUB_SILENT:-} " in *" $t "*) echo "$t none"; continue ;; esac
    case "$t" in /tf_static|/hw/wheel_feedback_valid) echo "$t latched(1-pub)" ;; *) echo "$t 10.00" ;; esac
done
S
cat >"$STUB/ros2" <<'S'
#!/bin/bash
if [ "$1 $2" = "bag record" ]; then
    echo "bag record $*" >>"$STUB_LOG"
    while [ $# -gt 0 ]; do
        [ "$1" = "-o" ] && bag="$2"
        [ "$1" = "--qos-profile-overrides-path" ] && cp "$2" "$STUB_QOS_COPY"
        shift
    done
    mkdir -p "$bag"
    if [ -n "${STUB_WAIT_INT:-}" ]; then   # behave like rosbag2: run until SIGINT, then finalise
        trap 'got=1' INT
        echo "[INFO] [rosbag2_recorder]: Recording..."
        while [ -z "${got:-}" ]; do sleep 0.1; done
        sleep 0.3   # a tee that died of the same SIGINT would lose everything printed below
    fi
    {   echo "rosbag2_bagfile_information:"
        echo "  duration: {nanoseconds: 10000000000}"
        echo "  topics_with_message_count:"
        for t in /scan /wheel/odom /joint_states /imu/bno085/data /laser/odom /odometry/filtered /tf /tf_static /cmd_vel \
                 /teleop/active_mode /teleop/autonomous/cmd_vel /hw/steer_states /hw/joint_states /hw/wheel_feedback_valid; do
            n=100; [ "$t" = /tf_static ] && n=1
            case " ${STUB_ZERO:-} " in *" $t "*) n=0 ;; esac
            case " ${STUB_LOW:-} " in *" $t "*) n=20 ;; esac
            echo "  - topic_metadata: {name: $t}"
            echo "    message_count: $n"
        done
    } >"$bag/metadata.yaml"
    [ -n "${STUB_NO_LOSS_LINE:-}" ] || echo "[WARN] [rosbag2_recorder]: Number of messages lost on the transport layer: 7"
    [ -n "${STUB_ABORTED:-}" ] || echo "[INFO] [rosbag2_recorder]: Recording stopped"
    exit 0
fi
echo "unexpected ros2 call: $*" >>"$STUB_ERRORS"; exit 1
S
cat >"$STUB/ssh" <<'S'
#!/bin/bash
[ -n "${STUB_NO_SSH:-}" ] && exit 255
case "${@: -1}" in
    *'git -C'*) echo "$(date +%s).000000000 abc1234 0"; echo "TS ${STUB_NTP:-yes} ${STUB_TS_OFFSET:--12us}" ;;
    *) echo "$(date +%s).000000000" ;;
esac
S
chmod +x "$STUB"/*
export PATH="$STUB:$PATH" STUB_LOG="$T/log" STUB_ERRORS="$T/errors" STUB_QOS_COPY="$T/qos_copy.yaml"
: >"$STUB_LOG"; : >"$STUB_ERRORS"
export GRIPPERX_YAW_SKIP_ENV=1 GRIPPERX_YAW_PROBE="$STUB/probe" GRIPPERX_YAW_ROS2="$STUB/ros2" \
       GRIPPERX_BAGS_DIR="$T/bags" GRIPPERX_YAW_SSH=stubhost GRIPPERX_YAW_STAMP_OFFSETS="/imu/bno085/data=0.004,/joint_states=0.006,/scan=0.012"
run() { bash "$SCRIPT" "$@" >"$T/out" 2>"$T/err"; echo $? >"$T/rc"; }
rc() { cat "$T/rc"; }

echo "== argument parsing =="
run;                         check "no label -> exit 1"            "$([ "$(rc)" = 1 ]; echo $?)"
run "bad label";             check "label with space rejected"     "$([ "$(rc)" = 1 ]; echo $?)"
run a b;                     check "two labels rejected"           "$([ "$(rc)" = 1 ]; echo $?)"
run x --bogus;               check "unknown option rejected"       "$([ "$(rc)" = 1 ]; echo $?)"
run x --window 0;            check "window 0 rejected"             "$([ "$(rc)" = 1 ]; echo $?)"
run --help;                  check "--help exits 0 and prints usage" "$([ "$(rc)" = 0 ] && grep -q 'Usage:' "$T/out"; echo $?)"
check "nothing recorded by the failed parses" "$([ ! -s "$STUB_LOG" ] && [ -z "$(ls -A "$T/bags")" ]; echo $?)"

echo "== normal run =="
run my_label
D="$T/bags/$(date +%F)_my_label"
check "exit 0"                          "$([ "$(rc)" = 0 ]; echo $?)"
check "session dir <date>_<label>"      "$([ -d "$D" ]; echo $?)"
check "bag dir <label>_<date>_<HHMM>"   "$(ls "$D" | grep -Eq '^my_label_[0-9]{8}_[0-9]{4}$'; echo $?)"
check "bag record got mcap + all required topics" \
      "$(grep -q -- '-s mcap' "$STUB_LOG" && for t in /scan /wheel/odom /joint_states /imu/bno085/data /laser/odom /odometry/filtered /tf /tf_static /cmd_vel; do grep -q -- " $t\b" "$STUB_LOG" || exit 1; done; echo $?)"
check "meta.yaml has git revs, both clocks, rates" \
      "$(grep -q 'git_pi_ws: {rev: abc1234' "$D/meta.yaml" && grep -q 'git_laptop:' "$D/meta.yaml" \
         && grep -q 'start_pi_epoch:' "$D/meta.yaml" && grep -q '"/scan": "10.00"' "$D/meta.yaml" \
         && grep -q 'required_missing: \[\]' "$D/meta.yaml"; echo $?)"
check "meta.yaml records transport loss from the recorder log" "$(grep -q 'transport_lost_messages: "7"' "$D/meta.yaml"; echo $?)"
check "all 5 extra topics reach bag record" \
      "$(for t in /teleop/active_mode /teleop/autonomous/cmd_vel /hw/steer_states /hw/joint_states /hw/wheel_feedback_valid; do grep -q -- " $t\b" "$STUB_LOG" || exit 1; done; echo $?)"
check "/joint_states QoS override passed: volatile + reliable" \
      "$(grep -q -- '--qos-profile-overrides-path' "$STUB_LOG" && grep -q '^/joint_states:' "$STUB_QOS_COPY" && grep -q 'durability: volatile' "$STUB_QOS_COPY" && grep -q 'reliability: reliable' "$STUB_QOS_COPY"; echo $?)"
check "meta.yaml: per-topic counts, duration, nothing flagged" \
      "$(grep -q 'bag_duration_sec: 10.00' "$D/meta.yaml" && grep -q '"/scan": 100' "$D/meta.yaml" && grep -q 'incomplete_required_topics: \[\]' "$D/meta.yaml"; echo $?)"
check "meta.yaml: header-stamp offsets and timesync state, no skew warning" \
      "$(grep -q '"/scan": 0.0120' "$D/meta.yaml" && grep -q 'pi_ntp_synchronized: yes' "$D/meta.yaml" && grep -q 'pi_timesyncd_offset_sec: -0.000012' "$D/meta.yaml" && ! grep -q 'WARNING' "$T/err"; echo $?)"
check "latched optional topic not shown as none" "$(grep -q 'hw/wheel_feedback_valid *latched' "$T/out"; echo $?)"
check "README.md written"               "$([ -s "$D/README.md" ]; echo $?)"
check "pre-flight table printed"        "$(grep -q '/imu/bno085/data' "$T/out"; echo $?)"

echo "== silent required topic =="
export STUB_SILENT="/imu/bno085/data"
: >"$STUB_LOG"
run strict_case --strict
check "--strict aborts with exit 2"     "$([ "$(rc)" = 2 ]; echo $?)"
check "--strict: warning names the topic" "$(grep -q 'NOT publishing: /imu/bno085/data' "$T/err"; echo $?)"
check "--strict: no recorder started, no directory" "$([ ! -s "$STUB_LOG" ] && [ ! -d "$T/bags/$(date +%F)_strict_case" ]; echo $?)"
run lenient_case
check "without --strict: warns but records" "$([ "$(rc)" = 0 ] && grep -q 'NOT publishing' "$T/err" && [ -s "$STUB_LOG" ]; echo $?)"
check "meta.yaml lists the missing topic" "$(grep -q 'required_missing: \[/imu/bno085/data\]' "$T/bags/$(date +%F)_lenient_case/meta.yaml"; echo $?)"

echo "== topic dies mid-run (count check) =="
STUB_ZERO="/imu/bno085/data"; export STUB_ZERO
run dead_case
check "0 messages on a required topic: exit 3 + warning" "$([ "$(rc)" = 3 ] && grep -q 'imu/bno085/data: 0 messages' "$T/err"; echo $?)"
check "flag written to meta.yaml" "$(grep -q 'incomplete_required_topics: \[/imu/bno085/data\]' "$T/bags/$(date +%F)_dead_case/meta.yaml"; echo $?)"
unset STUB_ZERO
STUB_LOW="/scan"; export STUB_LOW
run low_case
check "far below pre-flight rate x duration: flagged (20 < 0.5 x 10 Hz x 10 s)" "$([ "$(rc)" = 3 ] && grep -q 'incomplete_required_topics: \[/scan\]' "$T/bags/$(date +%F)_low_case/meta.yaml"; echo $?)"
unset STUB_LOW
STUB_ZERO="/hw/steer_states"; export STUB_ZERO
run extra_dead
check "optional topic with 0 messages is not flagged" "$([ "$(rc)" = 0 ]; echo $?)"
unset STUB_ZERO

echo "== transport loss line =="
STUB_NO_LOSS_LINE=1; export STUB_NO_LOSS_LINE
run noloss
check "completed run without a loss line -> 0" "$(grep -q 'transport_lost_messages: "0"' "$T/bags/$(date +%F)_noloss/meta.yaml"; echo $?)"
STUB_ABORTED=1; export STUB_ABORTED
run aborted
check "run without 'Recording stopped' -> unknown" "$(grep -q 'transport_lost_messages: "unknown"' "$T/bags/$(date +%F)_aborted/meta.yaml"; echo $?)"
unset STUB_NO_LOSS_LINE STUB_ABORTED

echo "== clock indicators =="
STUB_TS_OFFSET="120ms"; export STUB_TS_OFFSET
run skew_ts
check "timesyncd offset 120 ms > limit: warning" "$(grep -q 'timesyncd offset 0.120000' "$T/err"; echo $?)"
unset STUB_TS_OFFSET
STUB_NTP=no; export STUB_NTP
run skew_ntp
check "Pi not NTP-synchronised: warning" "$(grep -q 'not NTP-synchronised' "$T/err"; echo $?)"
unset STUB_NTP
GRIPPERX_YAW_STAMP_OFFSETS="/imu/bno085/data=0.004,/joint_states=0.9,/scan=0.012"; export GRIPPERX_YAW_STAMP_OFFSETS
run skew_stamp
check "median(receive - header stamp) 0.9 s: warning names /joint_states" "$(grep -q '/joint_states: bag receive time - header stamp = 0.900' "$T/err"; echo $?)"
GRIPPERX_YAW_STAMP_OFFSETS="/imu/bno085/data=0.004,/joint_states=0.006,/scan=0.012"
echo "== Ctrl-C (SIGINT to the whole process group) =="
SIGOUT="$T/sig.out"; SIGERR="$T/sig.err"
# A shell's `&` job starts with SIGINT ignored, which would make every trap below moot; the
# launcher restores the default disposition and makes its own process group, as a terminal does.
STUB_WAIT_INT=1 python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.setsid(); os.execvp("bash", ["bash", sys.argv[1], "sigint_case"])' "$SCRIPT" >"$SIGOUT" 2>"$SIGERR" &
LPID=$!
for _ in $(seq 100); do grep -q 'Recording\.\.\.' "$SIGOUT" 2>/dev/null && break; sleep 0.1; done
PGID="$(ps -o pgid= -p "$LPID" | tr -d ' ')"
kill -INT -- "-$PGID"
for _ in $(seq 100); do kill -0 "$LPID" 2>/dev/null || break; sleep 0.1; done
M="$T/bags/$(date +%F)_sigint_case/meta.yaml"
check "script survives group SIGINT and prints Done." "$(grep -q '^Done\.' "$SIGOUT"; echo $?)"
check "record.log has the recorder's last lines (tee survived)" "$(grep -q 'Recording stopped' "$T/bags/$(date +%F)_sigint_case/record.log"; echo $?)"
check "meta.yaml: end_laptop, recorder_exit 0, loss = stub's 7" \
      "$(grep -q '^end_laptop:' "$M" && grep -q 'recorder_exit: 0' "$M" && grep -q 'transport_lost_messages: "7"' "$M"; echo $?)"
check "meta.yaml: bag report lines present" "$(grep -q 'bag_duration_sec: 10.00' "$M" && grep -q 'incomplete_required_topics: \[\]' "$M"; echo $?)"

echo "== silent optional topic =="
export STUB_SILENT="/hw/steer_states"
: >"$STUB_LOG"
run opt_case --strict
check "--strict does not abort for an optional topic" "$([ "$(rc)" = 0 ] && [ -s "$STUB_LOG" ]; echo $?)"
unset STUB_SILENT

echo "== preflight-only =="
before="$(ls "$T/bags" | wc -l)"; : >"$STUB_LOG"
run --preflight-only
check "--preflight-only without label works, records nothing" \
      "$([ "$(rc)" = 0 ] && [ ! -s "$STUB_LOG" ] && [ "$(ls "$T/bags" | wc -l)" = "$before" ]; echo $?)"

echo "== Pi unreachable =="
export STUB_NO_SSH=1
run nossh
check "recording still runs, meta says unreachable" \
      "$([ "$(rc)" = 0 ] && grep -q 'git_pi_ws: {rev: unreachable' "$T/bags/$(date +%F)_nossh/meta.yaml" && grep -q 'Pi not reachable' "$T/err"; echo $?)"
unset STUB_NO_SSH

echo "== stubs =="
check "no unexpected ros2 call" "$([ ! -s "$STUB_ERRORS" ]; echo $?)"
echo; echo "passed: $PASS  failed: $FAIL"
[ "$FAIL" -eq 0 ]
