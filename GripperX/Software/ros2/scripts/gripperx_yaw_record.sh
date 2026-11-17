#!/bin/bash
# GripperX yaw cross-check recorder
#
# What it is: records the topics needed to compare the three yaw sources (encoders,
# LiDAR / slam_toolbox, IMU gyro) into one rosbag2 bag (mcap storage), after a
# pre-flight that checks every topic is alive and prints its rate. Sub-issue 1.1 of
# claude/docs/ISSUE_DRAFT_LIDAR_IMU_2026-10-22.md. Status: 2026-10-22.
#
# Usage:
#   gripperx_yaw_record.sh <label> [--strict] [--preflight-only] [--window SEC]
#   gripperx_yaw_record.sh -h | --help
#     <label>           required, [A-Za-z0-9_-]+, e.g. static_imu, inplace_left_v1
#     --strict          abort (exit 2) if any REQUIRED topic does not publish
#     --preflight-only  run the pre-flight and print rates, record nothing (no bag
#                       and no directory is created; use for a desk check)
#     --window SEC      discovery window and rate window, each, default 4 (chosen, not derived)
#
# Output: ${GRIPPERX_BAGS_DIR:-~/gripperx_bags}/<YYYY-MM-DD>_<label>/
#   <label>_<YYYYMMDD>_<HHMM>/   the bag (mcap, uncompressed - same as 2026-10-08)
#   meta.yaml                    git rev of laptop and Pi ~/ws, both clocks, rates
#   README.md                    one-screen description for whoever opens the folder
#   record.log                   recorder output
# Stop recording with Ctrl-C (SIGINT lets rosbag2 close the mcap file cleanly).
#
# WHERE IT RUNS: on the LAPTOP, on DDS domain 20 (the real robot), over the network -
# exactly how the bags of 2026-10-07/08 in ~/gripperx_bags were made (they sit on the
# laptop, mcap, and carry the /scan etc. of the Pi). Same RMW as gripperx_desk.sh
# (rmw_fastrtps_cpp). The price is that a WiFi dropout loses samples; the pre-flight
# rates are taken over the same path, so they show what the bag will actually get.
# The Pi is only touched by ONE read-only ssh (git rev + clock), retried a few times
# because the link is unstable; if it stays unreachable the recording still runs and
# meta.yaml says "unreachable".
#
# NOTHING HERE MOVES THE ROBOT: it only subscribes. It publishes nothing, calls no
# service, restarts nothing. Whether the robot moves during a recording is decided
# by whoever drives it, with the per-segment approval of SR-1 (motor-enable process
# rule) - see claude/docs/YAW_TEST_PROTOCOL_TEMPLATE.md.
#
# Topics. REQUIRED (strict mode aborts when one is silent):
#   /scan /wheel/odom /joint_states /imu/bno085/data /laser/odom /odometry/filtered
#   /tf /tf_static /cmd_vel
#   /cmd_vel is the teleop_mux OUTPUT topic: teleop_mux_node.py publishes Twist on
#   '/cmd_vel' (self._cmd_pub) and the controllers subscribe there
#   (cmd_vel_topic: /cmd_vel in gripperx_control/config/ros2_controllers.yaml and
#   swerve_cmd.yaml). The mux INPUTS (/teleop/keyboard/cmd_vel,
#   /teleop/autonomous/cmd_vel) are deliberately not required.
#   Steering angles are IN /joint_states (joint_state_broadcaster: *steer joints).
# EXTRA (recorded, a silent one only warns, even with --strict):
#   /teleop/active_mode      mux mode, to see who commanded a segment
#   /teleop/autonomous/cmd_vel  Nav2 side of the mux (empty in a hand-driven test)
#   /hw/steer_states         raw steering feedback from steer_servo_node
#   /hw/joint_states         raw wheel feedback before the controller stamps it
#   /hw/wheel_feedback_valid feedback validity flag
# /joint_states has TWO publishers with different durability (joint_state_broadcaster
# TRANSIENT_LOCAL: wheels + steering; arm_joint_state_relay VOLATILE: arm joints). rosbag2
# would pick one by discovery order and drop the other, so the recorder is given a QoS
# override (volatile, reliable) for /joint_states; both are then recorded.
# Clock check: the ssh round trip makes "Pi date - laptop date" wrong by up to ~1 s, so it is
# kept only as a gross sanity check (GROSS_CLOCK_SEC, the Pi has no RTC). The real indicators
# are the Pi's systemd-timesyncd state and offset (timedatectl, read-only) before recording,
# and after recording the median of (bag receive time - header stamp) for the Pi-stamped
# topics /imu/bno085/data, /joint_states, /scan (includes network latency, so a few ms to
# tens of ms are normal). After recording, per-topic message counts and the duration are
# read from the bag metadata and every required topic with 0 messages or fewer than
# COUNT_FRACTION x expected is flagged (exit 3).
# Clock note for the later analysis: /joint_states is stamped at controller read time,
# not at ESP32 sample time (stale-sample note in localization.yaml); /hw/joint_states
# is recorded to make that visible.
set -u

REQUIRED_TOPICS=(/scan /wheel/odom /joint_states /imu/bno085/data /laser/odom
                 /odometry/filtered /tf /tf_static /cmd_vel)
EXTRA_TOPICS=(/teleop/active_mode /teleop/autonomous/cmd_vel /hw/steer_states
              /hw/joint_states /hw/wheel_feedback_valid)
LATCHED_TOPICS=(/tf_static /hw/wheel_feedback_valid)   # once / on change; judged by publisher count, no rate

REAL_DOMAIN=20
RMW_IMPL=rmw_fastrtps_cpp
BAGS_ROOT="${GRIPPERX_BAGS_DIR:-$HOME/gripperx_bags}"
SSH_ALIAS="${GRIPPERX_YAW_SSH:-gripperx}"
ROS2_BIN="${GRIPPERX_YAW_ROS2:-ros2}"      # overridden by the offline test
# All limits below are CHOSEN, not derived from data:
GROSS_CLOCK_SEC=60       # coarse date check only: catches the no-RTC failure (days off), not ms skew
TIMESYNC_WARN_SEC=0.05   # Pi timesyncd offset / median(bag receive time - header stamp) worth a warning
COUNT_FRACTION=0.5       # a required topic with fewer messages than this x (pre-flight rate x duration) is flagged
WINDOW=4                                   # seconds; pre-flight takes up to 2x this

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() { sed -n '2,/^set -u/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//' | sed -n '1,22p'; }

die() { echo "gripperx_yaw_record: $*" >&2; exit 1; }

is_latched() { local t; for t in "${LATCHED_TOPICS[@]}"; do [ "$t" = "$1" ] && return 0; done; return 1; }

parse_args() {
    LABEL=""; STRICT=0; PREFLIGHT_ONLY=0
    while [ $# -gt 0 ]; do
        case "$1" in
            -h|--help) usage; exit 0 ;;
            --strict) STRICT=1 ;;
            --preflight-only) PREFLIGHT_ONLY=1 ;;
            --window) shift; [ $# -gt 0 ] || die "--window needs a value"
                      WINDOW="$1" ;;
            -*) die "unknown option: $1" ;;
            *) [ -z "$LABEL" ] || die "more than one label given ('$LABEL', '$1')"
               LABEL="$1" ;;
        esac
        shift
    done
    [[ "$WINDOW" =~ ^[0-9]+$ ]] && [ "$WINDOW" -ge 1 ] || die "--window must be a whole number of seconds >= 1"
    if [ "$PREFLIGHT_ONLY" -eq 0 ] || [ -n "$LABEL" ]; then
        [ -n "$LABEL" ] || die "label is required (usage: gripperx_yaw_record.sh <label> [--strict])"
        [[ "$LABEL" =~ ^[A-Za-z0-9_-]+$ ]] || die "label may only contain A-Z a-z 0-9 _ - (got '$LABEL')"
    fi
}

# Same guard sim_env.sh / gripperx_desk.sh use against a stale ~/ros2_ws on ~/.bashrc.
setup_env() {
    [ "${GRIPPERX_YAW_SKIP_ENV:-0}" = 1 ] && return 0
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
    set +u   # ROS setup files read unset variables
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash || die "cannot source /opt/ros/jazzy/setup.bash"
    set -u
    export ROS_DOMAIN_ID=$REAL_DOMAIN RMW_IMPLEMENTATION=$RMW_IMPL
}

# Dir name for today's recording; HHMM goes into the bag name so a second recording
# with the same label on the same day does not collide.
session_dir() { echo "$BAGS_ROOT/$(date +%F)_$LABEL"; }
bag_name()    { echo "${LABEL}_$(date +%Y%m%d_%H%M)"; }

# Prints "<topic> <rate|none>" per topic. ONE rclpy node subscribes to all topics at
# once (parallel `ros2 topic hz` processes starve each other's DDS discovery and read
# "none" on a live robot - measured 2026-10-22). Best-effort QoS connects to reliable and
# best-effort publishers alike. The window starts when the node is up; the first
# message of a topic is not counted as an interval. Latched topics (/tf_static) are
# judged by publisher count, since they publish once. Needs the ROS environment.
read -r -d '' PROBE_PY <<'PY'
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
window = float(sys.argv[1]); latched = set(sys.argv[2].split(','))
topics = sys.argv[3:]
rclpy.init()
node = Node('gripperx_yaw_preflight')
count = {t: 0 for t in topics}; first = {}; last = {}
def mk(t):
    def cb(_m):
        now = time.monotonic()
        first.setdefault(t, now); last[t] = now; count[t] += 1
    return cb
# wait (bounded) for discovery so the types are known
t_end = time.monotonic() + window
types = {}
while time.monotonic() < t_end and len(types) < len(topics):
    for name, tt in node.get_topic_names_and_types():
        if name in count and name not in types and tt:
            types[name] = tt[0]
    rclpy.spin_once(node, timeout_sec=0.2)
subs = []
for t, ty in types.items():
    if t in latched:
        continue
    subs.append(node.create_subscription(get_message(ty), t, mk(t), qos_profile_sensor_data))
t_end = time.monotonic() + window
while time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.1)
for t in topics:
    if t in latched:
        n = node.count_publishers(t)
        print(t, 'latched(%d-pub)' % n if n else 'none')
    elif count[t] >= 2 and last[t] > first[t]:
        print(t, '%.2f' % ((count[t] - 1) / (last[t] - first[t])))
    else:
        print(t, 'none')
node.destroy_node(); rclpy.shutdown()
PY

preflight() {
    local latched; latched="$(IFS=,; echo "${LATCHED_TOPICS[*]}")"
    if [ -n "${GRIPPERX_YAW_PROBE:-}" ]; then   # offline test hook
        "$GRIPPERX_YAW_PROBE" "$WINDOW" "$latched" "${REQUIRED_TOPICS[@]}" "${EXTRA_TOPICS[@]}"
    else
        python3 -c "$PROBE_PY" "$WINDOW" "$latched" "${REQUIRED_TOPICS[@]}" "${EXTRA_TOPICS[@]}"
    fi
}

# One read-only ssh. Line 1: "<epoch> <git rev> <dirty-file-count>"; line 2: "TS <NTPSynchronized>
# <timesyncd offset>" (the Pi uses systemd-timesyncd, no chrony). Retried (unstable link).
pi_snapshot() {
    local out n
    for n in 1 2 3; do
        out="$(ssh -o BatchMode=yes -o ConnectTimeout=5 "$SSH_ALIAS" \
               'echo "$(date +%s.%N) $(git -C ~/ws rev-parse --short HEAD) $(git -C ~/ws status --porcelain | wc -l)"; echo "TS $(timedatectl show -p NTPSynchronized --value 2>/dev/null) $(timedatectl timesync-status 2>/dev/null | sed -n "s/^ *Offset: *//p")"' 2>/dev/null)" \
            && [ -n "$out" ] && { echo "$out"; return 0; }
        sleep 2
    done
    echo "unreachable"
}

# "-12us" / "1.2ms" / "0.5s" -> seconds; anything else -> unknown
offset_to_sec() {
    awk -v v="$1" 'BEGIN{ if (match(v, /^-?[0-9.]+/)) { n = substr(v, RSTART, RLENGTH) + 0; u = substr(v, RLENGTH + 1);
        if (u == "us") n /= 1e6; else if (u == "ms") n /= 1e3; else if (u != "s") { print "unknown"; exit }
        printf "%.6f", n } else print "unknown" }'
}

# After recording: counts + duration from the bag metadata, flags, header-stamp offsets.
# stdout = meta.yaml lines, stderr = warnings. Exit 3 when a required topic is flagged.
# Args: bagdir fraction latched-csv warn-sec required-csv then "topic=preflight-rate" ...
read -r -d '' REPORT_PY <<'PY'
import sys, os, statistics, yaml
bagdir, fraction, latched, warn, required = sys.argv[1], float(sys.argv[2]), set(sys.argv[3].split(',')), float(sys.argv[4]), sys.argv[5].split(',')
rates = dict(a.rsplit('=', 1) for a in sys.argv[6:])
try:
    info = yaml.safe_load(open(os.path.join(bagdir, 'metadata.yaml')))['rosbag2_bagfile_information']
    counts = {t['topic_metadata']['name']: int(t['message_count']) for t in info['topics_with_message_count']}
    dur = info['duration']['nanoseconds'] / 1e9
except Exception as e:
    print('bag_report: "unavailable (%s)"' % str(e).replace('"', "'"))
    print('bag report unavailable: %s' % e, file=sys.stderr); sys.exit(3)
print('bag_duration_sec: %.2f' % dur)
print('message_counts:')
for t in sorted(counts): print('  "%s": %d' % (t, counts[t]))
flag = []
for t in required:
    n = counts.get(t, 0)
    try: rate = float(rates.get(t, 'none'))
    except ValueError: rate = None
    if t in latched or rate is None: low = n < 1
    else: low = n < fraction * rate * dur
    if low: flag.append(t)
    if low: print('WARNING: %s: %d messages in %.1f s (pre-flight rate %s)' % (t, n, dur, rates.get(t, 'none')), file=sys.stderr)
print('incomplete_required_topics: [%s]' % ', '.join(flag))
# header stamp vs receive time, Pi-stamped topics
offs = {}
hook = os.environ.get('GRIPPERX_YAW_STAMP_OFFSETS')
if hook:
    offs = {k: float(v) for k, v in (kv.split('=') for kv in hook.split(',') if kv)}
else:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        rd = rosbag2_py.SequentialReader()
        rd.open(rosbag2_py.StorageOptions(uri=bagdir, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        types = {m.name: m.type for m in rd.get_all_topics_and_types()}
        want = [t for t in ('/imu/bno085/data', '/joint_states', '/scan') if t in types]
        rd.set_filter(rosbag2_py.StorageFilter(topics=want))
        acc = {t: [] for t in want}; cls = {t: get_message(types[t]) for t in want}
        while rd.has_next() and all(len(v) < 400 for v in acc.values()):
            t, data, rx = rd.read_next()
            if len(acc[t]) >= 400: continue
            st = deserialize_message(data, cls[t]).header.stamp
            acc[t].append(rx / 1e9 - (st.sec + st.nanosec / 1e9))
        offs = {t: statistics.median(v) for t, v in acc.items() if v}
    except Exception as e:
        print('stamp_offsets: "unavailable (%s)"' % str(e).replace('"', "'"))
if offs:
    print('receive_minus_header_stamp_median_sec:')
    for t, v in offs.items():
        print('  "%s": %.4f' % (t, v))
        if abs(v) > warn:
            print('WARNING: %s: bag receive time - header stamp = %.3f s (> %.3f s, chosen limit): '
                  'network latency or Pi/laptop clock offset' % (t, v, warn), file=sys.stderr)
sys.exit(3 if flag else 0)
PY

main() {
    parse_args "$@"
    setup_env

    local t0_laptop t1_laptop pi laptop_rev laptop_dirty pi_epoch pi_rev pi_dirty skew ts_sync ts_offset ts_off_sec
    t0_laptop="$(date +%s.%N)"
    pi="$(pi_snapshot)"
    t1_laptop="$(date +%s.%N)"
    t0_laptop="$(awk -v a="$t0_laptop" -v b="$t1_laptop" 'BEGIN{printf "%.3f", (a+b)/2}')"  # ssh midpoint
    laptop_rev="$(git -C "$HERE" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    laptop_dirty="$(git -C "$HERE" status --porcelain 2>/dev/null | wc -l)"
    ts_sync=unknown; ts_off_sec=unknown
    if [ "$pi" = unreachable ]; then
        pi_epoch=unreachable; pi_rev=unreachable; pi_dirty=unreachable; skew=unknown
        echo "WARNING: Pi not reachable over ssh - Pi clock, timesync state and git rev not recorded." >&2
    else
        read -r pi_epoch pi_rev pi_dirty <<<"$(echo "$pi" | sed -n 1p)"
        ts_sync="$(echo "$pi" | sed -n 's/^TS \([^ ]*\).*/\1/p')"; ts_sync="${ts_sync:-unknown}"
        ts_offset="$(echo "$pi" | sed -n 's/^TS [^ ]* *//p')"
        ts_off_sec="$(offset_to_sec "$ts_offset")"
        # COARSE only: includes the ssh round trip (up to ~1 s). Catches a no-RTC Pi that is days off.
        skew="$(awk -v a="$pi_epoch" -v b="$t0_laptop" 'BEGIN{printf "%.1f", a-b}')"
        if awk -v s="$skew" -v w="$GROSS_CLOCK_SEC" 'BEGIN{exit !((s<0?-s:s) > w)}'; then
            echo "WARNING: Pi date differs from the laptop by ${skew} s (> ${GROSS_CLOCK_SEC} s, gross check): Pi clock is wrong." >&2
        fi
        if [ "$ts_sync" != yes ]; then
            echo "WARNING: Pi is not NTP-synchronised (timedatectl NTPSynchronized=${ts_sync}); stamps from the Pi may be offset." >&2
        elif [ "$ts_off_sec" != unknown ] && awk -v s="$ts_off_sec" -v w="$TIMESYNC_WARN_SEC" 'BEGIN{exit !((s<0?-s:s) > w)}'; then
            echo "WARNING: Pi timesyncd offset ${ts_off_sec} s (> ${TIMESYNC_WARN_SEC} s, chosen limit)." >&2
        fi
    fi
    echo "Clocks: laptop $(date -d @"${t0_laptop%.*}" '+%F %T %z'), Pi epoch ${pi_epoch} (coarse difference ${skew} s, includes ssh latency)"
    echo "Pi time sync: NTPSynchronized=${ts_sync}, timesyncd offset ${ts_off_sec} s"
    echo "Git:    laptop ${laptop_rev} (${laptop_dirty} modified files), Pi ~/ws ${pi_rev} (${pi_dirty} modified files)"

    echo "Pre-flight, up to ${WINDOW} s discovery + ${WINDOW} s measurement (robot must stand still):"
    local rates missing=() extra_missing=() line topic rate
    rates="$(preflight)"
    while read -r topic rate; do
        printf '  %-28s %s\n' "$topic" "$rate"
        if [ "$rate" = none ]; then
            if printf '%s\n' "${REQUIRED_TOPICS[@]}" | grep -qx "$topic"; then missing+=("$topic"); else extra_missing+=("$topic"); fi
        fi
    done <<<"$rates"
    [ ${#extra_missing[@]} -eq 0 ] || echo "NOTE: optional topics silent: ${extra_missing[*]}"
    if [ ${#missing[@]} -gt 0 ]; then
        echo "WARNING: required topics NOT publishing: ${missing[*]}" >&2
        if [ "$STRICT" -eq 1 ]; then
            echo "Aborting (--strict). Nothing recorded." >&2
            exit 2
        fi
        echo "Continuing without --strict: the bag will be incomplete." >&2
    fi
    [ "$PREFLIGHT_ONLY" -eq 1 ] && { echo "Pre-flight only - nothing recorded."; exit 0; }

    local dir bag; dir="$(session_dir)"; bag="$(bag_name)"
    mkdir -p "$dir" || die "cannot create $dir"
    [ ! -e "$dir/$bag" ] || die "$dir/$bag already exists (same label recorded within this minute)"

    {
        echo "label: $LABEL"
        echo "bag: $bag"
        echo "storage: mcap"
        echo "recorded_on: laptop (DDS domain $REAL_DOMAIN)"
        echo "strict: $STRICT"
        echo "start_laptop: \"$(date -d @"${t0_laptop%.*}" '+%F %T %z')\""
        echo "start_laptop_epoch: $t0_laptop"
        echo "start_pi_epoch: $pi_epoch"
        echo "pi_minus_laptop_coarse_sec: $skew   # gross check only, includes ssh latency (up to ~1 s)"
        echo "pi_ntp_synchronized: $ts_sync"
        echo "pi_timesyncd_offset_sec: $ts_off_sec"
        echo "git_laptop: {rev: $laptop_rev, modified_files: $laptop_dirty}"
        echo "git_pi_ws: {rev: $pi_rev, modified_files: $pi_dirty}"
        echo "preflight_window_sec: $WINDOW"
        echo "preflight_rates_hz:"
        while read -r topic rate; do echo "  \"$topic\": \"$rate\""; done <<<"$rates"
        echo "required_missing: [${missing[*]:-}]"
    } >"$dir/meta.yaml"
    cat >"$dir/README.md" <<README
# Yaw recording $LABEL ($(date +%F))

Made by Software/ros2/scripts/gripperx_yaw_record.sh. Bag: \`$bag/\` (mcap). Git revs, clocks
and the pre-flight topic rates are in \`meta.yaml\`. Fill in the operator protocol
(claude/docs/YAW_TEST_PROTOCOL_TEMPLATE.md) and keep it next to this file as \`protocol.md\`.
README

    cat >"$dir/qos_overrides.yaml" <<QOS
/joint_states:
  history: keep_last
  depth: 10
  reliability: reliable
  durability: volatile
QOS
    echo "Recording to $dir/$bag - Ctrl-C to stop."
    # Ctrl-C sends SIGINT to the whole foreground process group: recorder, tee and this script.
    # rosbag2 must get it (it finalises the bag); tee must survive it (else the recorder's last
    # lines - "Recording stopped", the loss count - never reach record.log); and this script must
    # survive it (else the end lines and the bag report are skipped). Hence the no-op trap here
    # and tee started with SIGINT ignored (an ignored signal is inherited through exec).
    trap ':' INT
    "$ROS2_BIN" bag record -s mcap -o "$dir/$bag" --qos-profile-overrides-path "$dir/qos_overrides.yaml" \
        --topics "${REQUIRED_TOPICS[@]}" "${EXTRA_TOPICS[@]}" 2>&1 | (trap '' INT; exec tee "$dir/record.log")
    local rc=${PIPESTATUS[0]}
    trap - INT
    local lost=unknown
    # rosbag2 prints the loss line only when loss > 0; "Recording stopped" shows it ran to the end.
    if [ -s "$dir/record.log" ] && grep -q 'Recording stopped' "$dir/record.log"; then
        lost="$(sed -n 's/.*Number of messages lost on the transport layer: *//p' "$dir/record.log" | tail -n1)"
        lost="${lost:-0}"
    fi
    {
        echo "end_laptop: \"$(date '+%F %T %z')\""
        echo "end_pi_epoch: $(ssh -o BatchMode=yes -o ConnectTimeout=5 "$SSH_ALIAS" 'date +%s.%N' 2>/dev/null || echo unreachable)"
        echo "recorder_exit: $rc"
        echo "transport_lost_messages: \"$lost\""
    } >>"$dir/meta.yaml"
    local latched_csv req_csv rate_args=() report_rc
    latched_csv="$(IFS=,; echo "${LATCHED_TOPICS[*]}")"; req_csv="$(IFS=,; echo "${REQUIRED_TOPICS[@]}" | tr ' ' ',')"
    while read -r topic rate; do rate_args+=("$topic=$rate"); done <<<"$rates"
    python3 -c "$REPORT_PY" "$dir/$bag" "$COUNT_FRACTION" "$latched_csv" "$TIMESYNC_WARN_SEC" "$req_csv" "${rate_args[@]}" >>"$dir/meta.yaml"
    report_rc=$?
    echo "Done. $dir"
    [ "$report_rc" -eq 0 ] || { echo "WARNING: bag is INCOMPLETE or could not be checked - see meta.yaml (exit 3)." >&2; exit 3; }
}

# Source guard: the offline test sources this file without running main.
if [ "${BASH_SOURCE[0]}" = "$0" ]; then main "$@"; fi
