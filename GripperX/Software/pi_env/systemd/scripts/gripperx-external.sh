#!/bin/bash
# /usr/local/bin/gripperx-external.sh
#
# Starts the Octopus external-goal link (FR-12 — external litter goals supplied
# by the Octopus litter-detection system over a rosbridge WebSocket) as a
# systemd service, so it comes up on boot and on a Mode R restart (HWR-40 short
# press — restart the ROS 2 stack, the Pi stays up) instead of only ever from a
# manual `ros2 launch`.
#
# ROLLOUT STAGE 3 — USER DECISION 2026-09-24, DURING THE FIRST REAL-ROBOT TEST.
# octopus_link.launch.py documents three stages: 1 = telemetry only (its
# defaults), 2 = `goal_ingress:=true`, disarmed and dry-run, 3 = additionally
# `dry_run:=false`. This service runs stage 3:
#   * the link node connects to the Octopus rosbridge, publishes the robot's
#     telemetry on /octopus/devices/gripperx/status and subscribes their datum,
#     transform-status AND goal topics (`goal_ingress_enabled: true`);
#   * the gateway validates every goal against the line calibration and the
#     geofence, publishes previews, and starts DISARMED (SR-15 rule 1 — no
#     persisted arming state);
#   * `dry_run:=false`: the second block on dispatch is OFF.
# THE ONLY REMAINING BLOCK IS ARMING: an explicit SetArming service call on the
# robot's own domain, which expires by itself (SR-15 rules 2/3/4). Once armed,
# the gateway sends Octopus goals to Nav2 and the robot drives and picks. Arming
# is therefore a motion approval under SR-1 and needs the user's go-ahead for
# that specific test.
#
# SUPERSEDED 2026-09-24: until then this file ran stage 1 and said that
# `dry_run:=false` "must never be reached by editing an autostart script". The
# user overrode that rule deliberately on 2026-09-24, knowing that FR-12 §10.1
# (the items owed before the real robot) is not yet met. To step back, set
# `dry_run:=true` (stage 2) or `goal_ingress:=false` (stage 1) below, reinstall
# this script (§0 of documentation/DEPLOYMENT.md — a `git pull` does NOT update
# what systemd runs) and restart this unit.
#
# WHAT THIS SERVICE NEEDS FROM THE REST OF THE STACK, and what happens when it
# is not there yet (the ordering rationale lives in gripperx-external.service):
#   /navigate_to_pose         gripperx-navigation.service (Nav2 bt_navigator)
#   /global_costmap/costmap   gripperx-navigation.service
#   map -> base_footprint TF  gripperx-mapping.service (slam_toolbox + EKF)
#   /odometry/filtered        gripperx-mapping.service (EKF)
#   /teleop/active_mode       gripperx-bringup.service (teleop_mux_node)
# None of them is a start condition. The gateway looks at `server_is_ready()`
# on every dispatch tick and never blocks on `wait_for_server`, so a missing
# Nav2 is reported (the NAV2_UNAVAILABLE auto-disarm trigger) and recovers by
# itself when discovery matches. Starting early therefore degrades to
# telemetry-only; it does not fail.
#
# WHERE AN OPERATOR LOOKS WHEN THIS DOES NOT COME UP (internal safety audit
# finding F-10: a refusal to start is SILENT to the Octopus, because their side
# has no failure channel — "no goals arriving" and "not running" look the same
# from there, so nobody may infer link health from the Octopus side):
#   systemctl status gripperx-external.service
#       status=78/EX_CONFIG -> a DELIBERATE refusal, caught by the pre-flight
#                              probe below. The unit stays `failed` and is NOT
#                              retried.
#       Result: exit-code / "Start request repeated too quickly" -> the launch
#                              tree kept ending on its own. Bounded retries ran
#                              out; the unit stays `failed`.
#   journalctl -u gripperx-external.service -b --no-pager
#       the nodes' own FATAL/ERROR lines, which is where the REASON is. Every
#       stop of this unit also logs systemd's own verdict (SERVICE_RESULT /
#       EXIT_STATUS) from ExecStopPost, so the journal distinguishes a requested
#       stop from a self-exit without anyone having been logged in at the time.
# There is no diagnostics path that works while the unit is down: the gateway's
# /diagnostics and /gripperx/external/status exist only while it runs, which is
# why the journal is the answer and why the refusal is given its own exit code.
#
# NOT A MOTION EVENT (SR-1 — no drive, steering or arm motion without explicit
# per-test user approval). This unit starts two pure-python rclpy nodes. They
# publish no command topic, hold no hardware interface, and their only route to
# an actuator is a Nav2 or /pick_plastic goal, which the gateway sends only while
# ARMED — and it always starts disarmed. Starting or restarting this unit moves
# nothing; arming it does.

source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/Software/ros2/install/setup.bash
# PlastiX domain convention — see gripperx-bringup.sh. Real GripperX-1 = 20.
# This is also the value the SR-8 probe below checks the configuration against.
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# UDP-only FastDDS transport (SHM off) — must be consistent across all gripperx
# services, otherwise localhost data paths break. Rationale: gripperx-bringup.sh.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
export HOME=/home/ubuntu

# Which config file the launch file loads: `real` selects octopus_link_real.yaml.
# MANDATORY — the launch argument defaults to `twin`, whose expected_domain_id is
# 220, so omitting it would make every start a FATAL SR-8 refusal on this machine.
LINK_ENV=real

# sysexits.h convention, so a refusal is distinguishable in `systemctl status`
# and can be given its own restart policy in the unit.
EX_CONFIG=78     # a deliberate refusal to start -> do NOT retry (RestartPreventExitStatus)

log() { echo "[external] $*"; }

# --- the SR-8 pre-flight probe (sim/real domain isolation) -------------------
# WHY A PROBE AT ALL, given that both nodes carry the guard themselves: because
# `ros2 launch` DESTROYS the evidence. Its return code is 0 unless the launch
# service itself raises — launch_service.py sets a non-zero code only from its
# own exception handler, never for a child that exited non-zero — so both nodes
# exiting FATAL with code 2 produces an idle launch service that shuts down and
# reports SUCCESS. Under Type=simple that reads as "the service ran and finished
# cleanly": the silent-health failure F-10 warns about, wearing systemd's
# clothes. This probe gives the one refusal we can name a status of its own,
# BEFORE launch is started and can swallow it.
#
# THIS IS NOT A SECOND COPY OF THE CHECK. It calls the package's own
# `domain_guard.check_domain` — the same function `enforce_domain` calls inside
# both nodes — against `expected_domain_id` read out of the same installed
# config file the nodes load. There is one rule in one place; this only asks it
# earlier, where the answer can still be turned into an exit status.
#
# IT FAILS OPEN BY DESIGN. Only a definite DomainMismatch refuses the start
# (exit 78). Anything that stops the probe from running at all — the module
# moved, the key renamed, no PyYAML — is reported as a WARN and the start
# continues, because a probe that cannot run must not become a new reason the
# link is down. The nodes still refuse in that case; the refusal then surfaces
# as a bounded restart loop instead of as exit 78, which is a loss of precision,
# not of safety.
#
# It deliberately checks SR-8 ONLY. The other by-design refusal — sim time on a
# non-simulation domain (SAFETY.md F-24) — is a one-line condition inside the
# gateway, and reproducing it here would be the duplicated P0-safety check this
# block just avoided. It is covered by the unit's bounded-retry path instead.
probe_sr8_domain() {
    python3 - "$LINK_ENV" <<'PY'
import os
import sys

try:
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from gripperx_external.domain_guard import DomainMismatch, check_domain

    path = os.path.join(
        get_package_share_directory("gripperx_external"),
        "config",
        "octopus_link_%s.yaml" % sys.argv[1],
    )
    with open(path, "r") as handle:
        config = yaml.safe_load(handle) or {}
    expected = [
        (node, body["ros__parameters"]["expected_domain_id"])
        for node, body in config.items()
        if isinstance(body, dict) and "expected_domain_id" in body.get("ros__parameters", {})
    ]
    if not expected:
        raise KeyError("no node in %s declares expected_domain_id" % path)
except Exception as exc:  # the probe machinery, never the verdict -- fail open
    print("probe could not run (%s: %s)" % (type(exc).__name__, exc), file=sys.stderr)
    sys.exit(3)

# From here on an exception IS the verdict and must not be swallowed.
for node, value in expected:
    try:
        check_domain(int(value))
    except DomainMismatch as exc:
        print("%s would refuse to start: %s" % (node, exc), file=sys.stderr)
        sys.exit(78)
print("%d node configs expect ROS_DOMAIN_ID=%s" % (len(expected), os.environ.get("ROS_DOMAIN_ID")))
PY
}

# --- the Octopus address, read out of the config and handed back to it ---------
# THIS IS A WORKAROUND FOR A DEFECT IN THE LAUNCH FILE, and it is here rather
# than in the config or the launch file because neither is this unit's to change.
#
# octopus_link.launch.py states that "only the two stage switches are overridable
# from the command line ... everything else comes from the config file", and then
# builds the link node's parameters as
#     parameters=[params_file, dict(overrides, url=url)]
# `url` is a LaunchConfiguration with default_value "ws://127.0.0.1:9090", and a
# later entry in that list WINS over an earlier one. So the launch argument's
# DEFAULT silently overrides `url:` in octopus_link_real.yaml, and a start that
# passes no `url:=` connects to localhost -- to the test/fake_octopus.py fixture's
# address -- no matter what the config says. The twin never noticed, because on a
# laptop 127.0.0.1:9090 IS where the fixture listens; on the robot it is nothing.
#
# So this script reads the address out of the SAME installed config file the nodes
# load and hands it straight back as `url:=`, which makes the override a no-op and
# restores the config as the single source. IT CONTAINS NO ADDRESS OF ITS OWN and
# it does not care what the address is -- whatever ends up in the YAML is what is
# used. The durable fix belongs in the launch file (stop overriding `url` unless
# the argument was given explicitly); until that is decided, this is the only
# place that can close the gap without editing a file this unit does not own.
read_link_url() {
    python3 - "$LINK_ENV" <<'PY'
import os
import sys

try:
    import yaml
    from ament_index_python.packages import get_package_share_directory

    path = os.path.join(
        get_package_share_directory("gripperx_external"),
        "config",
        "octopus_link_%s.yaml" % sys.argv[1],
    )
    with open(path, "r") as handle:
        config = yaml.safe_load(handle) or {}
    urls = sorted({
        body["ros__parameters"]["url"]
        for body in config.values()
        if isinstance(body, dict) and "url" in body.get("ros__parameters", {})
    })
    if len(urls) != 1:
        raise KeyError("expected exactly one url in %s, found %r" % (path, urls))
except Exception as exc:
    print("%s: %s" % (type(exc).__name__, exc), file=sys.stderr)
    sys.exit(3)
print(urls[0])
PY
}

URL_ARG=()
URL_OUT="$(read_link_url 2>&1)"
URL_RC=$?
# Last line only, and it has to look like a WebSocket URL: a stray warning on
# stderr (PyYAML deprecations have done this before) must not end up being passed
# as the address.
URL_LAST="$(printf '%s\n' "$URL_OUT" | tail -n 1)"
case "$URL_RC:$URL_LAST" in
    0:ws://*|0:wss://*)
        URL_ARG=(url:="$URL_LAST")
        log "Octopus url taken from octopus_link_${LINK_ENV}.yaml: $URL_LAST"
        ;;
    *)
        log "WARN: could not read a usable Octopus url out of octopus_link_${LINK_ENV}.yaml --"
        log "      $URL_OUT"
        log "      Starting WITHOUT a url:= argument, which means the launch file's"
        log "      DEFAULT applies (ws://127.0.0.1:9090, the test fixture's address) and"
        log "      the url in the config file is IGNORED -- see the comment above. The"
        log "      link will report connection failures against localhost; that is this"
        log "      warning, not a network problem on the Octopus side."
        ;;
esac

PROBE_OUT="$(probe_sr8_domain 2>&1)"
PROBE_RC=$?
case "$PROBE_RC" in
    0)
        log "SR-8 domain guard (pre-flight): $PROBE_OUT"
        ;;
    78)
        log "FATAL: refusing to start -- the SR-8 domain guard would reject this"
        log "       configuration, so both nodes would exit FATAL and 'ros2 launch'"
        log "       would still report success (see the probe comment above)."
        log "       $PROBE_OUT"
        log "       This is a CONFIGURATION error: either ROS_DOMAIN_ID above or"
        log "       expected_domain_id in octopus_link_${LINK_ENV}.yaml is wrong, or"
        log "       LINK_ENV names the wrong config. systemd will NOT retry this"
        log "       (RestartPreventExitStatus=${EX_CONFIG}); fix it and start the unit again."
        exit "$EX_CONFIG"
        ;;
    *)
        log "WARN: the SR-8 pre-flight probe could not run and is being IGNORED --"
        log "      $PROBE_OUT"
        log "      Starting anyway: the guard inside both nodes is unaffected. A"
        log "      refusal will then show up as a bounded restart loop rather than"
        log "      as exit ${EX_CONFIG}."
        ;;
esac

# --- run the link ----------------------------------------------------------
# `exec`, DELIBERATELY, AND NOT `ros2 launch ... &` + `wait` (which is what
# gripperx-mapping.sh does, because it has follow-up work to do after the
# launch). MEASURED on this laptop, not inherited from a habit: a NON-INTERACTIVE
# bash sets SIGINT and SIGQUIT to IGNORED in every asynchronous (`&`) child, and
# an ignored signal cannot be trapped or reset by the child, so
#   sleep 5 &        ->  SigIgn: 0000000000000006   (SIGINT + SIGQUIT)
#   the script itself ->  SigIgn: 0000000000000004   (SIGQUIT only)
# A backgrounded `ros2 launch` would therefore IGNORE the SIGINT that
# KillSignal=SIGINT sends it -- and SIG_IGN is inherited across fork and exec, so
# the two NODES launch spawns would ignore it as well. With KillMode=control-group
# the signal would reach every process in the cgroup and NONE of them would act on
# it: the stop would fall through to TimeoutStopSec and a SIGKILL, with no ordered
# teardown and no chance for the gateway to cancel a goal in flight (it installs
# its own SIGINT/SIGTERM handlers precisely so that cancel can be sent, and a
# SIGKILL is the one signal that defeats them).
# `exec` makes launch the unit's MainPID with systemd's own default signal
# dispositions, which is the only shape in which the stop contract works.
#
# The cost of `exec` is that this script cannot convert launch's exit status into
# anything afterwards. That is handled where it belongs — in the unit, by
# Restart=always plus a start-rate limit, because for THIS service a clean exit 0
# is itself a failure (launch reports 0 once its children are gone). See the
# reasoning block on Restart= in gripperx-external.service.
#
# Every argument is passed EXPLICITLY even where it merely restates a launch
# default, for the reason gripperx-navigation.sh gives for its own
# use_sim_time:=false: this file is installed under /usr/local/bin and does NOT
# deploy by `git pull`, so it can be older than the tree it belongs to. Stating
# the stage at the call site means the stage is visible in one diff of one file
# rather than inferred from two defaults somewhere else.
#   env:=real            mandatory, see LINK_ENV above
#   goal_ingress:=true   rollout stage 2+ (user decision 2026-09-24)
#   dry_run:=false       stage 3 (user decision 2026-09-24): arming is the only block
#   use_sim_time:=false  there is no /clock on the real robot (SAFETY.md F-24)
#   url:=<from the config>  only to neutralise the launch default; see above. The
#                       array is EMPTY when the address could not be read, and the
#                       warning above then says what that means.
exec ros2 launch gripperx_external octopus_link.launch.py \
  env:="$LINK_ENV" \
  goal_ingress:=true \
  dry_run:=false \
  use_sim_time:=false \
  "${URL_ARG[@]}"
