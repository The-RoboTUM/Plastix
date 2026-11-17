#!/bin/bash
# GripperX Desktop launcher
#
# Modes (first argument, default "real"):
#   real   RViz + teleop against the REAL robot, ROS_DOMAIN_ID=20. Also runs a
#          strictly read-only health check of the Pi first.
#   twin   Digital twin: Gazebo (sim_navigation.launch.py, GUI) + standalone
#          RViz + teleop, ROS_DOMAIN_ID=220. Runs Nav2 like the real robot
#          (where gripperx-navigation.service does), on live slam_toolbox
#          like gripperx-mapping.service, so an RViz "2D Goal Pose" is driven
#          in the twin too. teleop_mux starts in keyboard, as on the robot:
#          G in the teleop hands over to Nav2 (autonomous), K takes it back.
#   -h / --help   print this text.
#
# Input device (--input keyboard|web, default web, both modes): WHICH front-end
# drives, never how many. web = web_teleop_node, the browser UI
# (gripperx_teleop/docs/TELEOP_WEB_UI.md), run backgrounded with its log in
# $LOG_DIR; the URL is printed. keyboard = keyboard_teleop_node in its own
# terminal window, one --keyboard away. See "Browser teleop UI" below.
#
# real and twin are mutually exclusive - there is deliberately NO mode that
# runs both at once, and the script refuses to start one while the other is
# being teleoperated (see "Mutual exclusion" below).
#
# Domain convention (SR-8, PlastiX-wide): real GripperX robots 20-29, their
# digital twin the same id +200. GripperX-1 = 20 (real) / 220 (twin).
#
# Environment separation: nothing is exported in THIS script's own shell. Every
# RViz/Gazebo/teleop process is spawned as its own "bash -c '<prelude>; exec
# <cmd>'" subshell that sources its OWN domain's environment - the same guard
# Software/ros2/scripts/sim_env.sh uses against the unrelated ~/ros2_ws on
# ~/.bashrc, which otherwise leaks a stale AMENT_PREFIX_PATH / ROS_DOMAIN_ID into
# every new terminal. Because nothing is exported here, no spawned process can
# inherit a stale or foreign domain from this script.
#
# Collision guard: before starting anything that is not harmless to duplicate
# (teleop node, the twin's Gazebo/control node group), the script snapshots which
# of ITS OWN target processes already exist on that exact domain - matched via
# /proc/<pid>/environ, not process name, so a same-named real-domain and
# twin-domain process are never confused. A match means that step is SKIPPED with
# a clear message: a double-publish on /teleop/keyboard/cmd_vel, or a second
# ros2_control/Gazebo instance fighting the existing one, is worse than not
# starting. Only PIDs that appear AFTER this script's own launch action count as
# "ours"; teardown touches those exact PIDs only, each re-verified against
# /proc/<pid>/cmdline immediately before the kill, because a PID can be reused by
# an unrelated process in between. Nothing pre-existing is ever touched.
#
# Mutual exclusion, TWO rules, both about the same thing - never two live
# command sources, and never an ambiguous one:
#   (1) real <-> twin, ACROSS domains: the script refuses to start at all if a
#       teleop node is already running on the OTHER domain - twin refuses while
#       one runs on 20, real refuses while one runs on 220. It must never be
#       ambiguous which window drives the REAL robot: two teleop windows look
#       identical on screen and differ only in an invisible ROS_DOMAIN_ID, so a
#       keypress meant for the simulation would move real hardware (SR-1/SR-8).
#   (2) keyboard <-> web, WITHIN one domain: a running browser UI refuses a
#       keyboard start and a running keyboard teleop refuses a web start, both
#       directions, naming the PID. This is the harder failure of the two.
#       web_teleop_node SUBCLASSES KeyboardTeleopNode and inherits _publish
#       unchanged, so both front-ends publish the SAME /teleop/keyboard/cmd_vel
#       and the same /teleop/direct_steer. The mux forwards whichever arrived
#       last, and NEITHER operator's dead-man covers the other's traffic:
#       releasing every key on the page does not stop a robot the terminal is
#       driving, and neither does its emergency stop (SR-2/SR-3, and
#       gripperx_teleop/docs/TELEOP_WEB_UI.md in its own words). The node itself
#       only WARNS about a rival (red banner, ERROR log, deliberately no kill);
#       this launcher refuses, which is the cheaper place to catch it.
# Both rules match by /proc/<pid>/environ, not by process name alone, so a
# same-named process on another domain is never mistaken for one on ours - and
# rule (2) looks for the OTHER node name, not just its own.
#
# TTY: keyboard_teleop_node reads the terminal in raw mode (tty/termios), so it
# needs a real terminal, not a backgrounded pipe. Each keyboard teleop instance
# is therefore opened in its own terminal-emulator window (gnome-terminal,
# falling back to x-terminal-emulator); if neither exists, the script prints the
# exact command to run manually instead of starting it headless. web_teleop_node
# needs NO tty - its input arrives over HTTP - which is why --input web skips the
# terminal-emulator dance and is backgrounded like RViz and the mapping stack.
#
# Browser teleop UI (--input web): gripperx_teleop/docs/TELEOP_WEB_UI.md is the
# reference; what matters here is what the launcher must not get wrong.
#   * It is the SAME teleop. WebTeleopNode subclasses KeyboardTeleopNode and
#     overrides neither _publish nor press nor center - only _announce (no raw
#     tty escape codes when there is no tty) and the _observe display hook. So
#     every safety property, and every safety obligation, carries over
#     unchanged; see the steering gate paragraph below.
#   * It binds 127.0.0.1:8080 by DELIBERATE default. The page has no password
#     and no login - control is a first-come lease - so 0.0.0.0 would put a
#     live drive interface for a real robot on the network, where anyone who
#     can reach the port can drive it. This script therefore does not offer a
#     free-form host variable at all: the only way to leave loopback is the
#     explicit --web-expose flag, which prints a plain warning and is not
#     settable from the environment, so it cannot be turned on by a stale
#     export in someone's shell.
#   * Port collision is checked BEFORE launching. A second web_teleop_node on
#     the same port dies in its bind() with a traceback in a log file nobody is
#     watching; the launcher instead names the process holding the port and
#     points at --web-port.
#   * The UI has been run against the real robot, browser front-end included, so
#     a bare "real" start is not a first contact. The date of that run is not
#     recorded here (`TO-VERIFY`).
#
# RViz config for "real": gripperx_localization/rviz/localization.rviz, Fixed
# Frame "map", with Map(/map) + LaserScan(/scan) + RobotModel(/robot_description)
# + TF - the Pi runs mapping as a systemd service, so /map and the full chain
# map->odom->base_footprint->base_link exist on domain 20. It also shows the
# Octopus preview and line-calibration markers (/gripperx/external/*_markers)
# and loads the Publish Point tool (/clicked_point) for the two-post line
# calibration; FilteredOdom has its covariance display off. Price of this config:
# it carries displays that stay empty without Nav2/robot_localization/a camera
# (GlobalPlan /plan, FilteredOdom /odometry/filtered, DepthCloud+Camera+Image on
# /camera/*). Override: RVIZ_CONFIG_REAL.
# Note on stationary SLAM: slam_toolbox stamps map->odom with the last PROCESSED
# scan (minimum_travel_distance 0.3 m / heading 0.3 rad), so while the robot
# stands still that stamp freezes and ages. Harmless here - the transform keeps
# being published at 50 Hz and tf2 serves the single cached entry for any query
# time - and it advances again as soon as you drive.
#
# Teardown: gripperx_gazebo/README.md explicitly warns a plain Ctrl-C is not
# enough for the twin. See "Collision guard" above for how this script tracks
# and kills exactly what it started, nothing else, no ros2 daemon stop, no
# name-only pkill sweep. The survivor list in that README is stale (see "NFR-10"
# below); the surviving set is gz sim / clock_ready_gate / parameter_bridge /
# teleop_mux_node / robot_state_publisher / async_slam_toolbox_node, plus the
# Nav2 servers and their lifecycle_manager since the twin runs Nav2.
#
# Teardown from the teleop's own Q: Enter/Ctrl-C in THIS terminal is not the
# only way to trigger the teardown above any more. Both teleop front-ends
# (web_teleop_node's Shift+Q/quit button, keyboard_teleop_node's Q/Ctrl+C) call
# _signal_desk_quit() in gripperx_teleop/keyboard_teleop_node.py AFTER their own
# stop/center shutdown has published -- never on SIGTERM or a crash, only on a
# deliberate operator quit -- which touches a per-run flag file this script
# hands them through GRIPPERX_DESK_QUIT_FLAG (unset in a standalone run, so
# that case is untouched). The wait loop below polls for that file exactly
# like it watches for Enter, and tears the WHOLE session down the same way --
# RViz, the local mapping stack, everything this run started, not just the
# teleop that asked for it.
#
# NFR-10: swerve_controller is the ACTIVE AND ONLY drive path.
# real_robot.launch.py does not include control.launch.py, and
# spawn_robot.launch.py spawns swerve_controller instead of the old
# steering/wheel controller pair, so NONE of swerve_cmd_node,
# joint_command_bridge or sim_steer_bridge is started any more, on either side.
# The nodes are gone from the repo; only the orphaned config
# gripperx_control/config/swerve_cmd.yaml is left. That does NOT retire the
# checks below: a stale install tree, an old bringup that outlived a deploy, or a
# hand-started control.launch.py from an older checkout can all still put such a
# process on the robot, and a health check exists to see what IS running rather
# than what the current source could start.
#   * health check: swerve_cmd_node/joint_command_bridge are checked with
#     INVERTED polarity - if either runs, that is a THIRD publisher on
#     /hw/joint_commands (documented normal state is exactly 2: GripperXInterface
#     + its watchdog, D4/OP-18b/SR-10) and is reported as a fault.
#   * swerve_controller itself is a controller_manager PLUGIN, not a process: it
#     cannot be pgrep-ed, and its spawner is one-shot and long gone by the time
#     this check runs. Verifying it needs "ros2 control list_controllers" on the
#     Pi, which is deliberately NOT done here - DDS traffic during/after bringup
#     provokes controller_manager overruns.
#   * the twin's collision-guard and orphan lists follow from this. The reason for
#     every entry and every deliberate non-entry is at the lists themselves, in
#     start_twin() and in the teardown.
#
# Steering safety gate (real mode, MOVEMENT-RELEVANT - read this before
# removing it): merely STARTING the teleop commands the steering. keyboard_
# teleop_node comes up in manoeuvre state CORNERING (the startup state) and in
# that branch publishes angle*STEER_PATTERN on /teleop/direct_steer at
# publish_rate_hz (default 50.0) with angle initialised to 0.0;
# steer_servo_node applies /teleop/direct_steer as an OVERRIDE on top of
# /hw/joint_commands. So the wheels are driven to straight-ahead (0 deg)
# immediately, without any key press: on a robot left with actuator power ON and
# the steering holding an off-centre angle from the last drive, starting the
# launcher swings all four wheels by that much, unrequested.
# This is exactly the class of action the user's standing rule covers: no
# movement of drive, steering servos or arm without explicit per-test approval.
# The gate therefore states what will happen, reads the CURRENT angles
# from /hw/steer_states
# (Float64MultiArray, radians, joint order FL FR BL BR - the topic
# steer_servo_node itself publishes from servo read-back, chosen over TF
# because it is the servo truth and needs no mapping/odometry to be alive) and
# requires the word MOVE to be typed. Anything else, or a non-interactive
# stdin, aborts the teleop start. If the angles cannot be read the gate does
# NOT fall through - unknown angles are treated as worst case and it still
# asks. Bypass for the "I know the power is off" case:
# GRIPPERX_DESK_STEER_GATE=off, or the flag --no-steer-gate. The bypass
# asserts, it does not verify.
# THE GATE APPLIES TO --input web IDENTICALLY, and that is not a courtesy.
# web_teleop_node does not override _publish, and its __init__ calls
# super().__init__(node_name='web_teleop_node'), which creates the parent's
# create_timer(self._dt, self._publish). The tick therefore runs from the moment
# the node comes up - before any browser has connected, with no client and no
# key press - and in CORNERING it publishes angle*STEER_PATTERN on
# /teleop/direct_steer exactly as the terminal node does. Starting the browser
# UI is a movement command in precisely the same sense (SR-1). The gate sits in
# start_teleop() ABOVE the keyboard/web branch for that reason: there is no web
# path around it, and adding one would be adding a way to move the steering
# without approval.
# Deliberately NOT solved by patching keyboard_teleop_node.py: that is a repo
# file and a separate work package. This gate is launcher-level only.
#
# Live map in real mode (local mapping stack): the Pi-side mapping service is
# out of scope here (the bringup->systemd binding, OP-20, belongs to the
# shutdown-contract track, and the Pi bringup is deliberately hand-launched -
# this script starts, stops and enables NOTHING on the Pi). To still get a
# moving robot model and a map being drawn, real mode starts the mapping stack
# LOCALLY on this laptop on domain 20: gripperx_bringup/mapping.launch.py
# (rf2o_laser_odometry: /scan -> /odom + odom->base_footprint TF, plus
# async slam_toolbox with the repo config gripperx_bringup/config/
# slam_toolbox.yaml), followed by a lifecycle configure/activate dance -
# slam_toolbox comes up unconfigured and publishes nothing until it is activated.
# THIS IS NO LONGER THE SAME STACK THE Pi RUNS, and the difference is left in
# place deliberately rather than papered over. gripperx-mapping.sh on the Pi
# launches gripperx_localization/localization.launch.py (laser_scan_matcher + EKF
# + slam_toolbox), not mapping.launch.py, and mapping.launch.py's
# slam_toolbox.yaml carries the older, coarser parameters while rf2o and the EKF
# both publish odom->base_footprint. So this laptop-side fallback is a DIFFERENT
# odometry source with COARSER SLAM settings, and running it beside a mapping Pi
# is a TF fight, not merely a duplicate - which is why the detection below knows
# the Pi's node set and refuses to start on an unknown answer. Whether the laptop
# fallback should switch to localization.launch.py as well is a real question and
# NOT decided here - it is a repo file, another stack's tuning, and a user call.
# WEAK POINT, by design: rf2o consumes /scan over WiFi. Every dropped or late
# scan becomes an odometry error, and unlike on the Pi there is no way to hide
# the link. Expect worse odometry than the Pi-side variant; on a bad link the
# map will smear. The Pi-side service remains the better place for this.
# It SKIPS itself if the Pi is already publishing (gripperx-mapping.service
# active, or an rf2o/slam_toolbox node already in the DDS graph on domain 20).
# That check is not cosmetic: two publishers of the odom->base_footprint edge
# make the TF tree ambiguous, tf2 serves whichever arrived last, and the robot
# jumps around in RViz. Opt out with GRIPPERX_DESK_LOCAL_MAP=off or
# --no-local-map. The processes are tracked and torn down PID-exactly like
# everything else this script starts.
# AND IT SKIPS ITSELF WHEN IT CANNOT TELL. Both pieces of evidence can fail at
# once: a loaded Pi lets every ssh attempt time out (evidence "unknown"), and on
# a link that cannot see the robot the DDS fallback "ros2 node list" on domain 20
# returns a graph with no robot in it - indistinguishable from a Pi that is not
# mapping. Two failed observations are not a measurement that nothing is mapping,
# and acting on them would START a second rf2o + slam_toolbox ON THE REAL ROBOT'S
# DOMAIN, by guessing. Unknown is treated as worst case, exactly as the steering
# gate does: no ssh answer means no local mapping stack, and the run says so.
# --force-local-map (or GRIPPERX_DESK_LOCAL_MAP=force) is the deliberate override
# for the case where the operator KNOWS the Pi is not mapping; like
# --no-steer-gate it asserts, it does not verify.
# AND IT WAITS WHILE THE Pi STACK IS IN TRANSITION. "inactive" is only a
# steady-state answer if nothing is about to change it. Incident 2026-10-16: the
# run started 27 s into a Mode R restart (short press of the Pi's shutdown button
# = gripperx-button-daemon runs gripperx-stack-stop --mode=restart, the stack goes
# down and comes back, the Pi stays up). bringup and mapping read `inactive`, the
# laptop started its own rf2o + slam_toolbox, and 20 s later the Pi's mapping came
# back beside it: two /slam_toolbox, two odom->base_footprint sources. The health
# check therefore asks for transition evidence from systemd and the process table
# - never from journal text - and waits (bounded, GRIPPERX_DESK_PI_SETTLE_SEC,
# with progress output) while any of it is present, then asks again. Ctrl-C ends
# the run at once, with nothing started (see on_fatal_signal):
#   * a gripperx-stack-stop process (any mode). It is the ONLY signal that covers
#     the whole sequence: during its dwell every unit reads plain `inactive` and
#     no job is queued, but the process is alive from the press until the last
#     `systemctl start` has returned (those block through the ExecStartPre sleeps
#     of mapping and navigation). The same pattern matches the operator names
#     gripperx-stop/-restart/-shutdown that gripperx-stack-stop documents as
#     symlinks to itself (not installed on GripperX-1 as of 2026-10-16). A
#     sequence whose every mode is stop or halt (Mode H, long press: the Pi
#     powers off) brings nothing back, so it is announced and NOT waited for.
#   * a queued systemd job for a gripperx unit (a start job waiting behind
#     After= ordering leaves its unit reading `inactive`, e.g. at boot).
#   * a gripperx unit in activating/deactivating/reloading - which includes a
#     Restart=on-failure unit waiting out its RestartSec.
#   * gripperx-mapping active for less than PI_MAPPING_GRACE_SEC with its node
#     set not yet complete.
# If the wait times out with any of that still present, or mapping is active with
# an incomplete node set past the grace time, the evidence is "unsettled" and the
# local stack is refused exactly like "unknown" (--force-local-map overrides). A
# restart that begins AFTER this check is the collision watch's job, not this one.
#
# gz-transport partition (twin): ROS_DOMAIN_ID isolates DDS, but gz-transport
# IGNORES it entirely - it namespaces by GZ_PARTITION, which defaults to
# "<hostname>:<username>" and is therefore IDENTICAL for every track on this
# laptop, so a parallel Gazebo can capture this one's spawn (SR-8 gap).
# sim_env.sh sets GZ_IP but no GZ_PARTITION and is a repo file that is NOT edited
# from here, so the twin prelude sets GZ_PARTITION itself, AFTER sourcing
# sim_env.sh. Any explicit non-empty name is disjoint from that default, so
# gripperx_desk_twin_220 cannot collide with a track that sources sim_env.sh
# unchanged. Override with GZ_PARTITION_TWIN.
#
# Leader arm (real mode only, 2026-10-16): the hand-held SO101 leader's node,
# gripperx_leader/leader_arm_node, is started on domain 20 when /dev/gripperx_leader
# exists - before this, it was started by hand, and a J in the UI without it went
# nowhere. It gets the installed leader_calibration.yaml as calibration_file and is
# NOT started without it (the node's default '' means no angles, no wall, J refused);
# a reused node started without one is warned about. It is PASSIVE until an align request (J) arrives: on start it writes
# only torque-off. Missing device: a WARN, J will not work, everything else starts.
# Already running (a leader_arm_node on domain 20, or any process of this user
# holding the port): not started again, reused, and NOT stopped by teardown - it
# is not ours. The executable is exec'd directly, not via "ros2 run": ros2 run
# does not forward a SIGINT sent to its PID alone, so teardown would hit the
# wrapper and leave the node running. Teardown sends SIGINT and waits up to
# LEADER_STOP_WAIT_SEC: the node's shutdown() aborts an alignment and releases
# the virtual wall with a confirmed torque-off on every leader servo. SIGTERM runs
# the same path (rclpy shuts the context down, the finally block still runs), and
# so does SIGHUP - closing the desk terminal sends it to the whole foreground
# group, and leader_arm_node handles it like SIGINT since 2026-10-16 (a node
# built before that dies on it WITHOUT the torque-off). Of the signals this setup
# sends, only SIGKILL skips it, and then a servo held by the wall can stay
# torqued - teardown says so if it had to escalate. Opt out with --no-leader or
# GRIPPERX_DESK_LEADER=off.
#
# Domain ownership: 20 = real GripperX-1, 220 = its twin (SR-8 makes that
# mandatory, not conventional), 221 = the "second parallel twin session" the
# same convention reserves; gripperx_external hard-codes SIMULATION_DOMAIN_IDS
# = {220, 221} and treats every other domain as a real robot. No owner is named
# for a busy domain: the check reports the DOMAIN and the PIDs and leaves the
# attribution to the reader. It is read-only - it names PIDs and warns, never
# acts.

set -u

# WS is derived from the script's OWN location, the same technique sim_env.sh in
# this directory uses, and for the same reason: a hardcoded "$HOME/gripperx_ws"
# would silently point at the shared main tree even when this file runs out of a
# git worktree, sourcing a foreign install/setup.bash.
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
SSH_HOST="gripperx"

REAL_DOMAIN=20
TWIN_DOMAIN=220
RMW_IMPL=rmw_fastrtps_cpp

# Domains that other parallel sessions may occupy - only ever READ. The two ids
# the convention reserves for simulation (SR-8 / SIMULATION_DOMAIN_IDS), NOT the
# property of any named worktree; see "Domain ownership" in the header.
FOREIGN_DOMAINS=(220 221)

# gz-transport partition for the twin group; see the header. Must differ from
# the "<hostname>:<username>" default that the nav2 track keeps.
GZ_PARTITION_TWIN="${GZ_PARTITION_TWIN:-gripperx_desk_twin_220}"

# Steering safety gate (real mode). "off"/"0"/"no"/"skip" bypasses it; the flag
# --no-steer-gate sets the same thing. See the header for what is being gated.
STEER_GATE="${GRIPPERX_DESK_STEER_GATE:-on}"

# Local mapping stack in real mode. "off"/"0"/"no"/"skip" or --no-local-map;
# "force" or --force-local-map starts it even when the Pi could not be asked
# (see start_local_mapping - "on" deliberately does NOT).
LOCAL_MAP="${GRIPPERX_DESK_LOCAL_MAP:-on}"

# Leader arm node in real mode. "off"/"0"/"no"/"skip" or --no-leader. See the header.
LEADER="${GRIPPERX_DESK_LEADER:-on}"
LEADER_DEV="/dev/gripperx_leader"
# All six since 2026-11-13: the follower drives its gripper from leader id 6 (FR-17) and
# refuses leader mode while the stream lacks a joint.
LEADER_SERVO_IDS="[1,2,3,4,5,6]"
# calibration_file defaults to '' in the node and in leader_arm.yaml; without it the
# leader angles are unknown: no radians, no virtual wall, every J refused.
LEADER_CALIBRATION="$WS/install/gripperx_leader/share/gripperx_leader/config/leader_calibration.yaml"
# Bounded wait for the node's own torque-off on SIGINT before escalating.
LEADER_STOP_WAIT_SEC=10

# Which teleop front-end drives - "web" (default) or "keyboard". Applies to real
# and twin alike; it selects the input device, it never starts a second one.
# --input <device> overrides, --keyboard is the shorthand back.
#
# The browser UI is the SAME teleop, not a lighter one: WebTeleopNode subclasses
# KeyboardTeleopNode and overrides neither _publish nor press nor center, so the
# dead-man, the TransitionGuard and the steering gate are the inherited ones - the
# default says which front-end you see, never how much protection you get.
# "keyboard" stays available for the case the web UI cannot serve (no browser, or
# a terminal-only session over ssh); it is that escape hatch, not a safety
# measure, since the browser UI has been run against the real robot with the
# front-end included. The date of that run is not recorded - `TO-VERIFY`.
INPUT="${GRIPPERX_DESK_INPUT:-web}"

# The two teleop node names, in one place. Every guard in this script iterates
# over this list rather than hardcoding "keyboard_teleop_node", because the two
# front-ends publish the same topics and a guard that knows only one of them is
# a guard that lets the dangerous case through.
TELEOP_NODE_PATTERNS=("keyboard_teleop_node" "web_teleop_node")

WEB_PORT="${GRIPPERX_DESK_WEB_PORT:-8080}"
WEB_OPEN_BROWSER="false"          # --open-browser

# BIND ADDRESS OF THE BROWSER UI - read the header before touching this.
# 127.0.0.1 is a safety default, not a formality: the page has no password and
# control is a first-come lease, so any address that is reachable from the
# network is a live drive interface for a real robot, open to whoever gets
# there first. There is DELIBERATELY no GRIPPERX_DESK_WEB_HOST environment
# variable - an exported value survives in a shell long after the reason for it
# is gone, and this is the one setting that must never be on by accident. The
# only way off loopback is the --web-expose flag, typed on the command line,
# for that run, with a warning printed.
WEB_HOST="127.0.0.1"

# Set by pi_health_check: active | inactive | unsettled | unknown. "unknown" means
# the Pi could not be asked at all (ssh down) - then the DDS graph is the only
# evidence. "unsettled" means it could be asked, but its stack was still in (or
# stuck in) a transition from which its own mapping can come back - see "AND IT
# WAITS WHILE THE Pi STACK IS IN TRANSITION" in the header. Only "inactive" lets
# the local mapping stack start without --force-local-map.
PI_MAPPING_EVIDENCE="unknown"
# The transition reasons still present when the settle wait gave up; empty when
# the Pi settled (or was never in transition).
PI_TRANSITION_LEFT=""
PI_SETTLE_MAX_SEC="${GRIPPERX_DESK_PI_SETTLE_SEC:-180}"
PI_SETTLE_POLL_SEC="${GRIPPERX_DESK_PI_SETTLE_POLL_SEC:-5}"
# A non-number would make the bound test fail on every pass, i.e. wait for ever.
case "$PI_SETTLE_MAX_SEC" in ''|*[!0-9]*) PI_SETTLE_MAX_SEC=180 ;; esac
case "$PI_SETTLE_POLL_SEC" in ''|*[!0-9]*) PI_SETTLE_POLL_SEC=5 ;; esac
# Measured 2026-10-16: the Mode R sequence ran from the press (09:43:58) to the
# last unit start (09:45:08), ~70 s, which the 180 s default covers with margin.
# The grace below is how long an `active` mapping unit may still be missing
# nodes before that stops counting as "coming up" and counts as "stuck" (still
# refused, but no longer waited for).
PI_MAPPING_GRACE_SEC=60

RVIZ_CONFIG_REAL="${RVIZ_CONFIG_REAL:-$WS/install/gripperx_localization/share/gripperx_localization/rviz/localization.rviz}"
RVIZ_CONFIG_TWIN="${RVIZ_CONFIG_TWIN:-$WS/install/gripperx_localization/share/gripperx_localization/rviz/localization.rviz}"

LOG_DIR="/tmp/gripperx_desk"
mkdir -p "$LOG_DIR"

# Deliberate-quit signal path (see "Teardown from the teleop's own Q" above).
# Keyed on $$ (this script's own PID), not a fixed name: LOG_DIR is shared by
# every gripperx_desk.sh run on this laptop, and a fixed name would let a
# second concurrent run (real + a stray twin) read or clear each other's flag,
# or a stale file from a killed-and-restarted run be mistaken for a fresh quit.
# Removed at both ends: here before anything is started, and by teardown()
# once this run is done with it.
QUIT_FLAG="$LOG_DIR/quit_flag_$$"
rm -f "$QUIT_FLAG"

# PID of the teleop NODE (not the launch wrapper), set by whichever of
# start_teleop_keyboard/start_teleop_web actually got one running. Empty means
# "no teleop is up" -- steer-gate declined, duplicate-front-end refusal, or the
# node never appeared -- and the wait loop's death check below must stay quiet
# in that case: there is nothing of ours to have died.
TELEOP_NODE_PID=""

# Same guard sim_env.sh uses against the unrelated ~/ros2_ws on ~/.bashrc.
REAL_PRELUDE="unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH; \
source /opt/ros/jazzy/setup.bash; source '$WS/install/setup.bash'; \
export ROS_DOMAIN_ID=$REAL_DOMAIN; export RMW_IMPLEMENTATION=$RMW_IMPL;"

# Reuses the existing sim_env.sh instead of duplicating its domain/GZ_IP logic.
# GZ_PARTITION is appended AFTER the source, because sim_env.sh (a repo file,
# not edited from here) sets none and gz-transport ignores ROS_DOMAIN_ID - see
# the "gz-transport partition" paragraph in the header. Every twin process
# (gz sim, ros_gz_bridge/parameter_bridge, RViz, teleop) is spawned through this
# prelude, so they all land in the same partition and none of them can see or be
# seen by another track's Gazebo.
TWIN_PRELUDE="source '$WS/scripts/sim_env.sh' >/dev/null; \
export GZ_PARTITION='$GZ_PARTITION_TWIN';"

# The twin's sim launch (Gazebo + slam_toolbox + Nav2) additionally needs
# sim_env_nav2.sh: this laptop has no system Nav2, so it is layered from the
# locally extracted .rosdeps_local debs, and the launch files hand those libs to
# the Nav2 processes only (GRIPPERX_ROSDEPS_LIB - see sim_env_nav2.sh for why it
# must not be global). sim_env_nav2.sh sources sim_env.sh itself. RViz and the
# teleop keep the plain TWIN_PRELUDE: they need none of it.
TWIN_NAV_PRELUDE="source '$WS/scripts/sim_env_nav2.sh' >/dev/null; \
export GZ_PARTITION='$GZ_PARTITION_TWIN';"

TERM_CMD=""
if command -v gnome-terminal >/dev/null 2>&1; then
    TERM_CMD="gnome-terminal"
elif command -v x-terminal-emulator >/dev/null 2>&1; then
    TERM_CMD="x-terminal-emulator"
fi

declare -a TRACKED_PIDS=()
declare -a TRACKED_LABELS=()
declare -a TRACKED_PATTERNS=()   # substring expected in /proc/<pid>/cmdline at kill time
declare -a TRACKED_INT_WAITS=()  # seconds to wait after SIGINT before SIGTERM
declare -a TRACKED_KILL_NOTES=() # warned if the process needed SIGTERM/SIGKILL
declare -a TRACKED_STAGES=()     # teardown pass: node (teleop NODES) < launch (their wrappers) < command (leader) < "" (passive)

usage() {
    cat <<'EOF'
Usage: gripperx_desk.sh [real|twin|-h|--help] [flags]

  real   (default) Pi health check (read-only) + local mapping stack + RViz +
         teleop against the REAL robot, ROS_DOMAIN_ID=20, plus the leader arm
         node when /dev/gripperx_leader exists (see --no-leader).
  twin   Digital twin: Gazebo (GUI) + slam_toolbox + Nav2 + standalone RViz +
         teleop, ROS_DOMAIN_ID=220, in its own gz-transport partition.
         teleop_mux starts in keyboard mode, like the real robot: press G in
         the teleop to let Nav2 drive an RViz "2D Goal Pose", K to take over.

Input device (both modes): --input web (default) or --input keyboard.
  web       web_teleop_node, the browser UI, backgrounded, log in /tmp/
            gripperx_desk. Needs no terminal; the URL is printed.
  keyboard  keyboard_teleop_node in its own terminal window (needs a tty).
Same node either way - the browser UI subclasses the keyboard one and inherits
the dead-man, the guard and the steering gate unchanged.

Mutual exclusion, two rules:
  real <-> twin   the script refuses to start while ANY teleop node (keyboard
                  or web) runs on the other domain - it must never be ambiguous
                  which window drives the real robot.
  keyboard <-> web  within one domain, each refuses while the other runs, in
                  both directions, naming the PID. They publish the SAME
                  cmd_vel and the same /teleop/direct_steer, and neither
                  operator's dead-man covers the other's traffic.

MOVEMENT WARNING (real mode): starting the teleop by itself commands the
steering to straight-ahead - the node starts in state CORNERING and publishes
/teleop/direct_steer at 50 Hz with angle 0.0, which steer_servo_node applies as
an override. No key press is needed, and this is true of the browser UI too:
web_teleop_node subclasses the keyboard node and inherits the publish tick
unchanged, so it starts publishing before any browser has connected. real mode
therefore shows the current wheel angles and asks you to type MOVE before
either teleop is started.

Flags:
  --input <keyboard|web>  teleop front-end (default web)
  --web                   shorthand for --input web
  --keyboard              shorthand for --input keyboard (terminal teleop)
  --open-browser          open the UI in a browser (web only, default off)
  --web-port <n>          TCP port for the UI (default 8080)
  --web-expose            bind the UI to 0.0.0.0 instead of 127.0.0.1.
                          UNAUTHENTICATED DRIVE INTERFACE: the page has no
                          password, and anyone who can reach the port can drive
                          this robot. Only on a network you control.
  --no-steer-gate   skip the MOVE confirmation (only if actuator power is OFF)
  --no-local-map    do not start rf2o+slam_toolbox locally in real mode
  --no-leader       do not start the leader arm node (real mode). Without this
                    flag it is started when /dev/gripperx_leader exists; a missing
                    device is a WARN (J in the teleop will not work), and an
                    already running leader_arm_node is reused, never stopped.
  --force-local-map start it even if the Pi could not be asked whether it is
                    already mapping, or its stack had not settled (asserts,
                    does not verify)

Env overrides: RVIZ_CONFIG_REAL=<path>, RVIZ_CONFIG_TWIN=<path>,
               GRIPPERX_DESK_STEER_GATE=off, GRIPPERX_DESK_LOCAL_MAP=off,
               GRIPPERX_DESK_LEADER=off,
               GRIPPERX_DESK_INPUT=web, GRIPPERX_DESK_WEB_PORT=<n>,
               GRIPPERX_DESK_MAP_WATCH_INTERVAL=<s>|off, GZ_PARTITION_TWIN=<name>
               GRIPPERX_DESK_PI_SETTLE_SEC=<s> (real mode: how long to wait for a
                 Pi stack in transition, e.g. a Mode R restart; default 180)
               (there is no environment variable for the bind address on
                purpose - see --web-expose)
EOF
}

log() { echo "[gripperx_desk] $*"; }
warn() { echo "[gripperx_desk][WARN] $*" >&2; }
ok() { echo "[gripperx_desk][OK] $*"; }

# --- process helpers -------------------------------------------------------

domain_of_pid() {
    # "2>/dev/null" BEFORE the input redirection, deliberately: redirections are
    # applied left to right, and it is the "< /proc/.../environ" itself that
    # fails (EACCES) for processes of other users - that failure is reported by
    # the shell, not by tr, so a trailing 2>/dev/null comes too late to suppress
    # it.
    tr '\0' '\n' 2>/dev/null < "/proc/$1/environ" | sed -n 's/^ROS_DOMAIN_ID=//p'
}

# PIDs whose full cmdline matches $1 (pgrep -f) AND whose ROS_DOMAIN_ID env is $2.
pids_for_pattern_on_domain() {
    local pattern="$1" domain="$2" pid
    for pid in $(pgrep -f -- "$pattern" 2>/dev/null); do
        [ "$(domain_of_pid "$pid")" = "$domain" ] && echo "$pid"
    done
}

pid_cmdline_matches() {
    local pid="$1" pattern="$2"
    tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | grep -qF -- "$pattern"
}

track() { # label pid pattern [sigint_wait_sec [escalation_note [stage]]]
    TRACKED_LABELS+=("$1")
    TRACKED_PIDS+=("$2")
    TRACKED_PATTERNS+=("$3")
    TRACKED_INT_WAITS+=("${4:-3}")
    TRACKED_KILL_NOTES+=("${5:-}")
    TRACKED_STAGES+=("${6:-}")
}

# Is the teleop NODE (TELEOP_NODE_PID, set by whichever start_teleop_* branch
# succeeded) still alive? Used by the main wait loop to tell "the operator quit
# on purpose" (QUIT_FLAG appears) apart from "the process just died" (it did
# not) - the second case must NOT tear the rest of the session down, only warn,
# because a killed/crashed teleop is not the same signal as a deliberate one
# and RViz/the local map may still be exactly what the operator is looking at.
# Empty TELEOP_NODE_PID means no teleop ever came up (steer gate declined, a
# rival front-end refused it, ...) - nothing to have died, so this reports "ok".
teleop_process_alive() {
    [ -n "$TELEOP_NODE_PID" ] || return 0
    kill -0 "$TELEOP_NODE_PID" 2>/dev/null
}

# THE ros2 CLI DAEMON IS NOT A NODE, AND MUST NOT BE REPORTED AS ONE.
# Every "ros2 node list" / "ros2 topic list" / "ros2 lifecycle get" auto-starts a
# per-domain helper - python3 -c "from ros2cli.daemon.daemonize import main;
# main()" --name ros2-daemon --ros-domain-id <d> - which then idles for two hours.
# It carries ROS_DOMAIN_ID in its environ, so to a /proc-based domain match it is
# indistinguishable from a real node, and THIS SCRIPT SPAWNS THEM ITSELF: the
# health check's topic list, the local-mapping DDS probe and the lifecycle calls
# all do, so other_track_check() would report an empty domain as busy on the
# strength of a daemon left behind by this script's own previous run. A warning
# that fires when nothing is wrong is worse than no warning, because it teaches
# the reader to skip the one that matters.
# Matched on the CMDLINE, not on a name: the process is called "python3", which
# discriminates nothing, and "ros2-daemon" appears only as an argument value.
is_ros2_cli_daemon() { # pid
    tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null \
        | grep -qE 'ros2cli[.]daemon|--name ros2-daemon'
}

# Every PID whose ROS_DOMAIN_ID env is $1, DAEMONS INCLUDED - the caller decides
# what to do with them (see other_track_check, which separates the two). The
# pattern-matched guards elsewhere in this script are structurally immune to the
# trap above, and should stay that way: they match on a node NAME
# (keyboard_teleop_node, web_teleop_node, spawn_robot, parameter_bridge, ...),
# and a CLI daemon's argv contains none of those. Only this function matches "any
# process on the domain", which is why only this function needs the distinction.
# Unreadable /proc entries (other users, root) are silently skipped by
# domain_of_pid's 2>/dev/null - a foreign user's ROS process is not ours to
# report on anyway.
pids_on_domain() { # domain
    local d="$1" entry pid
    for entry in /proc/[0-9]*; do
        pid="${entry#/proc/}"
        [ "$(domain_of_pid "$pid")" = "$d" ] && echo "$pid"
    done
}

# Running Gazebo processes, regardless of domain and partition. pgrep -f matches
# full argv, and this script spawns its children as bash -c "<prelude> exec gz
# ..." wrappers whose argv contains the very pattern - the classic self-match
# trap. Every hit is therefore re-verified against /proc/<pid>/comm, which for a
# wrapper is bash/sh and for the real thing is not.
gazebo_pids() {
    local pid comm
    { pgrep -f -- "gz sim" 2>/dev/null; pgrep -f -- "gz-sim-server" 2>/dev/null; } \
        | sort -u | while read -r pid; do
        [ -n "$pid" ] || continue
        comm=$(cat "/proc/$pid/comm" 2>/dev/null)
        case "$comm" in
            bash|sh|dash|pgrep|sort|gripperx_desk.s*) continue ;;
        esac
        echo "$pid"
    done
}

# --- mutual exclusion ---------------------------------------------------------
# Two rules, see the "Mutual exclusion" paragraph in the header:
#   (1) real <-> twin, across domains  -> refuse_if_other_teleop()
#   (2) keyboard <-> web, within one domain -> refuse_if_conflicting_input()
# Both are read-only: nothing is killed, nothing is started, the run simply does
# not begin. Both match on /proc/<pid>/environ, and both look for BOTH teleop
# node names rather than only the one this run would start.

# "<node_pattern> PID <pid>" for every teleop node on domain $1, one per line.
teleop_nodes_on_domain() { # domain
    local d="$1" pat pid
    for pat in "${TELEOP_NODE_PATTERNS[@]}"; do
        for pid in $(pids_for_pattern_on_domain "$pat" "$d"); do
            echo "$pat PID $pid"
        done
    done
}

# The node name the requested input device would start, and the one it must not
# collide with. Kept as a pair of functions so the mapping exists exactly once.
node_pattern_for_input() { # input
    case "$1" in
        web) echo "web_teleop_node" ;;
        *)   echo "keyboard_teleop_node" ;;
    esac
}
rival_pattern_for_input() { # input
    case "$1" in
        web) echo "keyboard_teleop_node" ;;
        *)   echo "web_teleop_node" ;;
    esac
}
human_name_for_pattern() { # node_pattern
    case "$1" in
        web_teleop_node) echo "browser teleop UI (web_teleop_node)" ;;
        *)               echo "terminal keyboard teleop (keyboard_teleop_node)" ;;
    esac
}

refuse_if_other_teleop() { # other_domain other_label
    local other_domain="$1" other_label="$2" hits
    hits=$(teleop_nodes_on_domain "$other_domain")
    [ -z "$hits" ] && return 0
    warn "REFUSING to start: a teleop node is already running on the"
    warn "$other_label domain $other_domain:"
    while IFS= read -r h; do warn "    $h"; done <<<"$hits"
    warn "real and twin are mutually exclusive - it must never be ambiguous which"
    warn "window drives the REAL robot. Close that teleop first, then start this"
    warn "mode again. Nothing was started or killed."
    return 1
}

# Rule (2). The rival front-end on OUR OWN domain. This is the more dangerous of
# the two cases and the reason the check exists at all: both nodes publish
# /teleop/keyboard/cmd_vel and /teleop/direct_steer (web_teleop_node inherits
# _publish from KeyboardTeleopNode unchanged), the mux forwards whichever
# arrived last, and neither operator's dead-man or emergency stop covers the
# other's traffic. Refusing the START is the only place this can be caught
# cheaply; once both run, killing one blind could be killing the one an operator
# is holding a key on.
refuse_if_conflicting_input() { # domain input
    local domain="$1" input="$2" rival pid pids=()
    rival=$(rival_pattern_for_input "$input")
    for pid in $(pids_for_pattern_on_domain "$rival" "$domain"); do
        pids+=("$pid")
    done
    [ "${#pids[@]}" -eq 0 ] && return 0
    warn "REFUSING to start: the $(human_name_for_pattern "$rival") is already"
    warn "running on THIS domain ($domain) - PID(s): ${pids[*]}."
    warn "You asked for --input $input, i.e. the $(human_name_for_pattern "$(node_pattern_for_input "$input")")."
    warn "The two publish the SAME cmd_vel and the same /teleop/direct_steer, and"
    warn "neither one's dead-man covers the other's traffic: releasing every key on"
    warn "one front-end does NOT stop a robot the other is driving, and neither does"
    warn "its emergency stop. Stop PID ${pids[0]} first, or start the same device it"
    warn "is already using. Nothing was started or killed."
    return 1
}

# --- teardown ---------------------------------------------------------------

# Is SIGINT ignored by $1 (bit 2 of SigIgn in /proc/<pid>/status)?
sigint_ignored() {
    local mask
    mask=$(sed -n 's/^SigIgn:[[:space:]]*//p' "/proc/$1/status" 2>/dev/null)
    [ -n "$mask" ] && [ $(( 0x$mask & 2 )) -ne 0 ]
}

# SIGINT one tracked process, wait, escalate. A process that is already gone, or
# whose PID now belongs to something else, is reported and skipped - teardown
# must complete whatever state the nodes are in.
stop_tracked() { # index
    local i="$1" pid label pattern int_wait note n
    pid="${TRACKED_PIDS[$i]}"
    label="${TRACKED_LABELS[$i]}"
    pattern="${TRACKED_PATTERNS[$i]}"
    int_wait="${TRACKED_INT_WAITS[$i]}"
    note="${TRACKED_KILL_NOTES[$i]}"
    if ! kill -0 "$pid" 2>/dev/null; then
        log "  $label (PID $pid) already gone."
        return 0
    fi
    if ! pid_cmdline_matches "$pid" "$pattern"; then
        warn "  $label (PID $pid) no longer matches its expected command " \
             "(PID reused?) - NOT touching it."
        return 0
    fi
    # A launch wrapper started as "bash -c ...; exec ros2 launch" & from a
    # non-interactive shell has SIGINT IGNORED (bash does that to async jobs
    # without job control; ros2 launch keeps it - measured 2026-10-22). SIGINT is
    # then a no-op and its grace period dead time, so such a LAUNCH (never the
    # node or leader: those install their own handler once running) goes straight
    # to SIGTERM. Also covers a wrapper signalled before its exec (third audit of
    # 4dd8cad, finding 1): it used to lose the SIGINT and wait the grace out.
    if [ "${TRACKED_STAGES[$i]:-}" = launch ] && sigint_ignored "$pid"; then
        log "  $label (PID $pid) ignores SIGINT - SIGTERM at once."
        int_wait=0
    else
        kill -INT "$pid" 2>/dev/null
    fi
    for ((n = 0; n < int_wait * 2; n++)); do
        kill -0 "$pid" 2>/dev/null || break
        teardown_sleep 0.5
    done
    if kill -0 "$pid" 2>/dev/null; then
        [ -n "$note" ] && [ "$int_wait" -gt 0 ] && warn "  $label (PID $pid) still running ${int_wait} s after SIGINT - SIGTERM."
        kill -TERM "$pid" 2>/dev/null
        for ((n = 0; n < 4; n++)); do   # same 2 s bound, but leave as soon as it is gone
            kill -0 "$pid" 2>/dev/null || break
            teardown_sleep 0.5
        done
    fi
    if kill -0 "$pid" 2>/dev/null; then
        kill -KILL "$pid" 2>/dev/null
        log "  $label (PID $pid): SIGKILL."
        [ -n "$note" ] && warn "  $label: $note"
    else
        log "  $label (PID $pid): stopped."
    fi
}

# The waits inside teardown, with SIGINT/SIGTERM ignored in the sleep itself: a
# Ctrl-C at the terminal reaches every process of the foreground group, and a
# sleep killed by it would shorten a SIGINT grace period (the leader's torque-off)
# into an early SIGTERM/SIGKILL.
teardown_sleep() { # seconds
    ( trap '' INT TERM; exec sleep "$1" )
}

# Keyboard teleop whose launch is out but whose node has not been seen yet (it
# comes up in its own terminal window, so there is no PID to track at launch).
# teardown() resolves it before stopping anything - see there.
TELEOP_KBD_PENDING_DOMAIN=""
TELEOP_KBD_PENDING_SINCE=0
TELEOP_KBD_PENDING_LABEL=""
TELEOP_KBD_BASELINE=""
TELEOP_KBD_WAIT_SEC=15

# Track the keyboard teleop node (and its launch) if it has appeared on its domain
# since the launch. Returns 0 (and sets TELEOP_NODE_PID) when the node was found.
# Must run in the caller's shell, not in $(...): track() appends to arrays. PIDs that already existed
# before the launch (TELEOP_KBD_BASELINE) are never claimed.
resolve_kbd_teleop() {
    local domain="$TELEOP_KBD_PENDING_DOMAIN" label="$TELEOP_KBD_PENDING_LABEL" pid node_pid="" launch_pid=""
    for pid in $(pids_for_pattern_on_domain "keyboard_teleop_node" "$domain"); do
        [[ " $TELEOP_KBD_BASELINE " == *" $pid "* ]] && continue
        node_pid="$pid"; break
    done
    [ -n "$node_pid" ] || return 1
    for pid in $(pids_for_pattern_on_domain "ros2 launch gripperx_teleop laptop_teleop.launch.py" "$domain"); do
        [[ " $TELEOP_KBD_BASELINE " == *" $pid "* ]] && continue
        launch_pid="$pid"; break
    done
    track "$label teleop node" "$node_pid" "keyboard_teleop_node" "" "" node
    [ -n "$launch_pid" ] && track "$label teleop launch" "$launch_pid" "gripperx_teleop laptop_teleop.launch.py" "" "" launch
    TELEOP_NODE_PID="$node_pid"
    TELEOP_KBD_PENDING_DOMAIN=""
}

TEARDOWN_DONE=0
# Set by teardown's own traps when INT/TERM/PIPE arrives WHILE it runs.
TEARDOWN_SIGNAL=""
teardown() {
    [ "$TEARDOWN_DONE" = 1 ] && return
    # Not "spent" on an empty run: a call with nothing tracked yet must not leave
    # whatever is tracked LATER without an owner for the EXIT trap.
    if [ "${#TRACKED_PIDS[@]}" -eq 0 ] && [ "${TWIN_LAUNCHED:-0}" = 0 ] \
       && [ -z "$TELEOP_KBD_PENDING_DOMAIN" ]; then
        return   # nothing was ever started (e.g. -h/--help, or all steps skipped) - stay quiet
    fi
    # ONE TEARDOWN RUNS TO THE END. From here on INT/TERM/PIPE only record that
    # they arrived; the run exits with that status AFTER the last process is
    # handled. Before 2026-10-16 (b3958a7) a Ctrl-C during the ordinary teardown
    # (Enter / Q / EOF) reached on_fatal_signal, whose teardown call returned at
    # once (already running) and whose exit then abandoned the processes not yet
    # stopped - the leader mid its torque-off grace, mapping and RViz untouched.
    trap 'TEARDOWN_SIGNAL=130' INT
    trap 'TEARDOWN_SIGNAL=143' TERM
    trap 'TEARDOWN_SIGNAL=141' PIPE
    TEARDOWN_DONE=1   # after the traps: no window in which a signal finds it "done"
    echo
    # The watch goes first: it must not comment on processes that are on their way
    # out, and everything it could still say is said once more at the end of teardown.
    if [ -n "${MAP_WATCH_PID:-}" ] && kill -0 "$MAP_WATCH_PID" 2>/dev/null; then
        kill -TERM "$MAP_WATCH_PID" 2>/dev/null
    fi
    log "Tearing down everything this script started..."
    [ -n "$TELEOP_NODE_PID" ] || [ -n "$TELEOP_KBD_PENDING_DOMAIN" ] \
        && log "  Stopping the teleop may straighten the steering (a teleop that publishes its stop on a signal sends straight-ahead)."
    # COMMAND SOURCES FIRST, everything passive (local mapping, RViz) after them.
    # Until 2026-10-16 the order was start order, so the teleop - the one process
    # here that can drive the real robot - stayed alive and publishing through the
    # SIGINT waits of the whole mapping stack and RViz. Within the command sources:
    #   1. the teleop NODES (stage node: web and keyboard, incl. the pending-claim
    #      one) - they are what publishes, so they get their signal first;
    #   2. their launch wrappers (stage launch);
    #   3. the leader arm node (stage command).
    # Node before launch since 2026-10-22: a launch started as "bash -c ...; exec
    # ros2 launch" & ignores SIGINT and, on SIGTERM, dies WITHOUT forwarding it, so
    # signalling the launch first left the node orphaned and publishing for the
    # whole wait, until this loop reached the node's own entry. Within each pass
    # the start order is kept. NO stop command is published from here, on purpose:
    #   * before the SIGINT it would be pointless - the live teleop republishes its
    #     own twist every tick and teleop_mux forwards whichever arrived last;
    #   * after it, a ros2 CLI publisher needs longer to come up than teleop_mux's
    #     cmd_timeout_sec (0.5 s), which already sends zero on silence.
    # STEERING: the teleop nodes are expected to publish their own full stop when
    # signalled (/arm/stop, leader disarm, keyboard mode, zero twist AND straight
    # steering [0,0,0,0]) - branch Theo-teleop-signal-stop, c5dff3f, NOT yet in
    # the default branch and still under audit when this was written; before it a
    # signalled teleop never sent the centring (rclpy took the context down before
    # main() could). Straightening is a STEERING MOVE, up to 90 deg out of a
    # crab/spin pose. The user accepted on 2026-10-22 that teardown may therefore
    # move the steering; this script still sends no centring itself, so on a robot
    # whose teleop lacks that change nothing is straightened.
    # A keyboard teleop launched but not yet seen (Ctrl-C inside its startup
    # poll) would otherwise come up AFTER this teardown in its own window, a live
    # command source nobody owns. Bounded by what is left of its startup window.
    if [ -n "$TELEOP_KBD_PENDING_DOMAIN" ]; then
        local left=$(( TELEOP_KBD_WAIT_SEC - (SECONDS - TELEOP_KBD_PENDING_SINCE) )) found=0
        [ "$left" -lt 1 ] && left=1
        log "  keyboard teleop launch still pending - waiting up to ${left} s for its node, so it is not left running."
        while [ "$left" -gt 0 ]; do
            resolve_kbd_teleop && { found=1; break; }
            teardown_sleep 1
            left=$((left - 1))
        done
        if [ "$found" = 0 ]; then
            warn "  keyboard teleop node never appeared on domain $TELEOP_KBD_PENDING_DOMAIN - check its terminal window."
        fi
    fi
    local i
    local stage
    for stage in node launch command; do
        for i in "${!TRACKED_PIDS[@]}"; do
            [ "${TRACKED_STAGES[$i]:-}" = "$stage" ] && stop_tracked "$i"
        done
    done
    for i in "${!TRACKED_PIDS[@]}"; do
        case "${TRACKED_STAGES[$i]:-}" in node|launch|command) ;; *) stop_tracked "$i" ;; esac
    done

    # Known orphan-prone twin nodes (see gripperx_gazebo/README.md): only
    # touch NEW ones that appeared after our own launch, never anything that
    # predates this script run. ORPHAN_BASELINE_TWIN is populated in
    # start_twin() before the launch, empty if twin was never (successfully)
    # started this run.
    if [ "${TWIN_LAUNCHED:-0}" = 1 ]; then
        local p pattern2
        # Same NFR-10 / Harmonic correction as the collision guard in
        # start_twin() - the two lists must stay identical, otherwise something
        # is baselined but never cleaned up, or cleaned up but never baselined.
        # The two launch-file names of the collision guard are not repeated
        # here: the launch wrapper is tracked and stopped above.
        for pattern2 in "gz sim" "async_slam_toolbox_node" \
                        "ground_truth_odom_bridge" "clock_ready_gate" \
                        "robot_state_publisher" "teleop_mux_node" "spawn_robot" \
                        "ros_gz_bridge" "parameter_bridge" "scan_range_filter" \
                        "planner_server" "controller_server" "behavior_server" \
                        "bt_navigator" "velocity_smoother" "lifecycle_manager" \
                        "nav2_map_server" "nav2_amcl" "ekf_node" \
                        "laser_scan_matcher" "localization_input_node" \
                        "rviz2_mapping" "rviz2_navigation"; do
            for p in $(pids_for_pattern_on_domain "$pattern2" "$TWIN_DOMAIN"); do
                if [[ " ${TWIN_BASELINE_PIDS} " == *" $p "* ]]; then
                    continue   # pre-existing, not ours - never touch
                fi
                warn "  orphan cleanup: killing leftover '$pattern2' (PID $p, new since our launch)"
                kill -KILL "$p" 2>/dev/null
            done
        done
    fi
    if [ -n "${MAP_WATCH_FLAG:-}" ] && [ -f "$MAP_WATCH_FLAG" ]; then
        rm -f "$MAP_WATCH_FLAG"
        warn "A MAPPING COLLISION was seen during this run, and the stack that is going"
        warn "  down now was part of it. The Pi's own /slam_toolbox may have been left"
        warn "  'unconfigured' by it: its configure can have been answered by the node this"
        warn "  script started. Then there is no /map publisher and no map->odom left on"
        warn "  domain $REAL_DOMAIN, and the next RViz opens empty. Check it, do not assume:"
        warn "    ssh $SSH_HOST 'source /opt/ros/jazzy/setup.bash; ROS_DOMAIN_ID=$REAL_DOMAIN ros2 lifecycle get /slam_toolbox'"
        warn "  Repair, if it does not say 'active':"
        warn "    ssh $SSH_HOST 'sudo systemctl restart gripperx-mapping'"
    fi
    # Consumed or not, this run is done with it - a stale file would be read by
    # a LATER gripperx_desk.sh run only if PIDs wrapped AND that run reused this
    # exact filename, which the $$ in its name already rules out; this is
    # cleanliness, not a safety measure.
    rm -f "$QUIT_FLAG" 2>/dev/null
    log "Teardown complete. Terminal windows (if any) may remain open (idle shell) - close them manually."
    if [ -n "$TEARDOWN_SIGNAL" ]; then
        trap - EXIT
        exit "$TEARDOWN_SIGNAL"
    fi
}
# PIPE is in the list deliberately. A shell killed by an UNTRAPPED fatal signal
# does not run its EXIT trap, and SIGPIPE is exactly that case: piping this
# script into something that exits early ("gripperx_desk.sh real | head") kills
# it the moment it writes to the closed pipe, leaving every process it started
# (RViz, the local mapping stack, Gazebo) running.
# INT, TERM and PIPE END THE RUN after the teardown. Until 2026-10-16 the trap was
# a bare "teardown", which returns: a Ctrl-C before anything was tracked (the Pi
# health check, e.g. its settle wait) tore down nothing and the script carried on
# starting RViz, the teleop and the leader - which then nothing tore down. The
# three are ignored while the handler runs, so a second Ctrl-C (or a write to the
# closed pipe from teardown's own log lines) cannot cut the teardown short; it is
# bounded anyway (SIGINT wait -> SIGTERM -> SIGKILL per process). A probe ssh in
# flight is killed first - it runs in the background exactly so that this trap
# fires at once instead of after up to 45 s (see pi_probe).
# What it reaches: every launch is tracked the moment it is backgrounded, so a
# Ctrl-C inside a start-up poll still stops it; the keyboard teleop, whose node
# starts inside a terminal emulator and has no PID at launch, is marked pending
# and claimed by teardown (bounded by its 15 s start-up window). How fast: the
# probe and the settle/retry sleeps are interruptible; the start-up polls are
# foreground sleeps of 0.5-3 s, so a signal sent to this PID alone can lag by
# that much (a terminal Ctrl-C ends the sleep as well), and the ros2 CLI calls
# under `timeout 20` (DDS node/topic list, lifecycle calls) are NOT interruptible:
# timeout runs in its own process group, so the trap waits for them, up to 20 s.
# A signal that arrives
# while teardown runs does not cut it short - see teardown().
on_fatal_signal() { # exit_status
    trap '' INT TERM PIPE
    if [ -n "${PI_PROBE_PID:-}" ]; then
        kill -TERM "$PI_PROBE_PID" 2>/dev/null
    fi
    echo
    log "interrupted - stopping."
    teardown
    exit "$1"
}
install_traps() {
    trap 'on_fatal_signal 130' INT
    trap 'on_fatal_signal 143' TERM
    trap 'on_fatal_signal 141' PIPE
    trap teardown EXIT
}
install_traps

# --- Pi health check (real mode only) ---------------------------------------

# A foreground sleep holds back a trap until it ends; a waited-for background one
# does not (same reason as the background probe in pi_probe).
interruptible_sleep() { # seconds
    sleep "$1" &
    wait "$!"
}

# One read-only look at the Pi, with retries. Sets PI_PROBE_OUT to the KEY: value
# lines of the payload; returns 1 (after saying so) when every attempt failed.
pi_probe() {
    # The retry budget is deliberately wide: on a fully loaded Pi (all four
    # services active) the first two attempts can time out and the third succeed.
    # A slow handshake is not a down robot, and declaring the robot unreachable
    # while it is merely busy teaches the operator to stop believing this check.
    # The outer "timeout" is not redundant with ConnectTimeout: ConnectTimeout
    # bounds the TCP/handshake phase only, and a session that hangs AFTER
    # authentication (the loaded case) would otherwise stall here for ever.
    # BACKGROUND + wait, not $(...): bash runs a trap only after the foreground
    # command returns, and "timeout" puts ssh in its own process group, so a
    # Ctrl-C at the terminal reached neither - the run sat out the ssh (up to 45 s)
    # before it could stop. The wait builtin returns as soon as a trapped signal
    # arrives, and on_fatal_signal kills PI_PROBE_PID. stdin is /dev/null: ssh
    # forwards stdin to the remote command, and this one reads none.
    local out attempt=0 max=5 rc tmp="$LOG_DIR/pi_probe_$$.out"
    PI_PROBE_OUT=""
    while :; do
        attempt=$((attempt + 1))
        timeout 45 ssh -o ConnectTimeout=10 -o BatchMode=yes "$SSH_HOST" '
            echo "UPTIME_LINE: $(uptime)"
            echo "SVC_BRINGUP: $(systemctl is-active gripperx-bringup 2>/dev/null)"
            echo "SVC_AGENT: $(systemctl is-active gripperx-agent 2>/dev/null)"
            echo "SVC_MAPPING: $(systemctl is-active gripperx-mapping 2>/dev/null)"
            echo "SVC_NAV: $(systemctl is-active gripperx-navigation 2>/dev/null)"
            # The [x] brackets are NOT cosmetic: pgrep -f matches full argv, and
            # the argv of the remote bash running THIS very script contains every
            # pattern below - without the brackets each count came back one too
            # high and "node missing" could never trigger. The regex "arm_action_serve[r]"
            # does not match the literal text "arm_action_serve[r]" in our own argv.
            echo "BRINGUP_PROC: $(pgrep -c -f "real_robot[.]launch[.]py")"
            echo "N_ARM: $(pgrep -c -f "arm_action_serve[r]")"
            echo "N_STEER: $(pgrep -c -f "steer_servo_nod[e]")"
            echo "N_SWERVE: $(pgrep -c -f "swerve_cmd_nod[e]")"
            echo "N_JCB: $(pgrep -c -f "joint_command_bridg[e]")"
            echo "N_MUX: $(pgrep -c -f "teleop_mux_nod[e]")"
            echo "N_LIDPWR: $(pgrep -c -f "lidar_power_nod[e]")"
            echo "N_RSP: $(pgrep -c -f "robot_state_publishe[r]")"
            echo "N_RC: $(pgrep -c -f "ros2_control_nod[e]")"
            echo "N_LIDAR: $(pgrep -c -f "ldlidar_stl_ros2_nod[e]")"
            echo "N_SCANFILT: $(pgrep -c -f "scan_range_filte[r]")"
            # WHAT THE Pi RUNS FOR MAPPING.
            # (No apostrophes in this block: it is a single-quoted ssh payload.)
            # gripperx-mapping.service launches gripperx_localization/
            # localization.launch.py with enable_slam:=true
            # enable_laser_odometry:=true: ros2_laser_scan_matcher (node name
            # laser_odometry_node) + ekf_node + odom_divergence_monitor +
            # async_slam_toolbox_node. It is NOT gripperx_bringup/mapping.launch.py
            # and rf2o is GONE from the robot, so requiring rf2o here would keep
            # PI_MAPPING_EVIDENCE from ever reaching "active" - the flag that stops
            # this laptop starting a SECOND mapping stack on domain 20. rf2o is
            # still asked for, with the opposite meaning: it and the EKF both
            # publish odom->base_footprint, so an rf2o on the Pi is a TF fight, not
            # a mapping stack.
            echo "N_RF2O: $(pgrep -c -f "rf2o_laser_odometry_nod[e]")"
            echo "N_LSM: $(pgrep -c -f "laser_scan_matche[r]")"
            echo "N_EKF: $(pgrep -c -f "ekf_nod[e]")"
            echo "N_SLAM: $(pgrep -c -f "async_slam_toolbox_nod[e]")"
            echo "AGENT_CT: $(docker ps --filter name=mros_agent --format "{{.Status}}" 2>/dev/null)"
            # TRANSITION EVIDENCE - see "AND IT WAITS WHILE THE Pi STACK IS IN
            # TRANSITION" in the header. systemd and the process table only, no
            # journal text. The stop sequence runs as root under sudo from the
            # button daemon, but its argv is still visible here; the [p] bracket
            # is the same self-match guard as above.
            echo "SVC_EXTERNAL: $(systemctl is-active gripperx-external 2>/dev/null)"
            # The pattern also matches the three operator names gripperx-stop /
            # gripperx-restart / gripperx-shutdown, which gripperx-stack-stop
            # documents as symlinks to itself (dispatch on $0; NOT installed on
            # GripperX-1 as of 2026-10-16). Every alternative carries its own
            # bracket: one unguarded alternative would match our own argv.
            echo "STACKSTOP_N: $(pgrep -c -f "bin/gripperx-(stack-sto[p]|restar[t]|sto[p]|shutdow[n])( |\$)")"
            echo "STACKSTOP_CMDS: $(pgrep -a -f "bin/gripperx-(stack-sto[p]|restar[t]|sto[p]|shutdow[n])( |\$)" | cut -d " " -f 2- | tr "\n" "|")"
            echo "JOBS: $(systemctl list-jobs --no-legend 2>/dev/null | grep -oE "gripperx[-a-z]*[.](service|target) +[a-z]+ +[a-z]+" | tr -s " " | tr " \n" ":,")"
            echo "MAP_ACTIVE_ENTER_US: $(systemctl show -p ActiveEnterTimestampMonotonic --value gripperx-mapping 2>/dev/null)"
            echo "UPTIME_S: $(cut -d. -f1 /proc/uptime)"
        ' >"$tmp" 2>&1 </dev/null &
        PI_PROBE_PID=$!
        wait "$PI_PROBE_PID"
        rc=$?
        PI_PROBE_PID=""
        out=$(cat "$tmp" 2>/dev/null)
        rm -f "$tmp"
        if [ "$rc" -eq 0 ]; then
            break
        fi
        if [ "$attempt" -ge "$max" ]; then
            warn "Pi unreachable after $max attempts via ssh $SSH_HOST. Continuing anyway -"
            warn "real-mode RViz/teleop will come up but likely show nothing/fail to connect."
            warn "  NOTE: a loaded Pi answers late, it is not necessarily down. Two of five"
            warn "  attempts timing out was normal on 2026-08-25. Try 'ssh $SSH_HOST uptime'"
            warn "  by hand before concluding anything."
            return 1
        fi
        warn "ssh attempt $attempt/$max failed (timeout or refused), retrying in 5s..."
        interruptible_sleep 5
    done
    PI_PROBE_OUT="$out"
    return 0
}

probe_field() { # probe_output KEY
    sed -n "s/^$2: //p" <<<"$1" | head -n1
}

# Seconds gripperx-mapping has been `active`, from the probe's two monotonic
# readings; empty when either is missing or not a number (never active, old
# systemd, garbled answer) - the caller then does not claim an age at all.
mapping_active_age() { # probe_output
    local us up
    us=$(probe_field "$1" MAP_ACTIVE_ENTER_US)
    up=$(probe_field "$1" UPTIME_S)
    case "$us" in ''|*[!0-9]*) return 0 ;; esac
    case "$up" in ''|*[!0-9]*) return 0 ;; esac
    [ "$us" -gt 0 ] || return 0
    echo $(( up - us / 1000000 ))
}

# The Pi's mapping node set is complete: slam_toolbox plus one odometry source of
# the current stack (same rule as PI_MAPPING_EVIDENCE="active" below).
mapping_nodes_complete() { # probe_output
    local n_slam n_lsm n_ekf
    n_slam=$(probe_field "$1" N_SLAM)
    n_lsm=$(probe_field "$1" N_LSM)
    n_ekf=$(probe_field "$1" N_EKF)
    [ "${n_slam:-0}" -ge 1 ] 2>/dev/null \
        && { [ "${n_lsm:-0}" -ge 1 ] 2>/dev/null || [ "${n_ekf:-0}" -ge 1 ] 2>/dev/null; }
}

# The modes of every running stop sequence, unique, space-separated: the last
# --mode= on its command line (an explicit --mode wins over the name, as in
# gripperx-stack-stop itself), else the mode its operator name dispatches to, else
# "?" (unreadable - treated like a restart by the callers: worst case).
stackstop_modes() { # probe_output
    local cmds line m lines=() modes=()
    cmds=$(probe_field "$1" STACKSTOP_CMDS)
    IFS='|' read -r -a lines <<<"$cmds"
    for line in "${lines[@]}"; do
        [ -n "${line// /}" ] || continue
        m=$(grep -oE -- '--mode=[a-z]+' <<<"$line" | tail -n1)
        m="${m#--mode=}"
        if [ -z "$m" ]; then
            case " $line " in
                *bin/gripperx-restart\ *)  m=restart ;;
                *bin/gripperx-stop\ *)     m=stop ;;
                *bin/gripperx-shutdown\ *) m=halt ;;
                *)                         m="?" ;;
            esac
        fi
        modes+=("$m")
    done
    [ "${#modes[@]}" -gt 0 ] || { echo "?"; return 0; }
    printf '%s\n' "${modes[@]}" | sort -u | tr '\n' ' ' | sed 's/ $//'
}

# Non-empty (the modes) when stop sequences are running and NONE of them brings
# the stack back: only --mode=stop (stack stays down; also bringup's own ExecStop,
# --mode=stop --from-unit) and --mode=halt (Mode H, long press: the Pi powers off).
# Mode R shows "restart" (plus that ExecStop's "stop"), and that is waited for.
pi_stopping_only() { # probe_output
    local n modes m
    n=$(probe_field "$1" STACKSTOP_N)
    [ "${n:-0}" -ge 1 ] 2>/dev/null || return 0
    modes=$(stackstop_modes "$1")
    for m in $modes; do
        case "$m" in stop|halt) ;; *) return 0 ;; esac
    done
    echo "$modes"
}

# Prints the reasons ("; "-separated) why the Pi stack is in transition right now,
# nothing if it is not. Pure: reads only the probe output, so the offline test
# harness can drive it. See the header for why each signal is on the list.
pi_transition_reasons() { # probe_output
    local out="$1" r=() n modes jobs key unit st age
    n=$(probe_field "$out" STACKSTOP_N)
    if [ "${n:-0}" -ge 1 ] 2>/dev/null && [ -z "$(pi_stopping_only "$out")" ]; then
        modes=$(stackstop_modes "$out")
        r+=("stop sequence running (gripperx-stack-stop, mode: $modes)")
    fi
    jobs=$(probe_field "$out" JOBS)
    jobs="${jobs%,}"
    [ -n "${jobs//[[:space:],]/}" ] && r+=("systemd jobs queued: ${jobs//,/, }")
    for key in AGENT:agent BRINGUP:bringup MAPPING:mapping NAV:navigation EXTERNAL:external; do
        unit="gripperx-${key#*:}"
        st=$(probe_field "$out" "SVC_${key%%:*}")
        case "$st" in
            activating|deactivating|reloading|refreshing) r+=("$unit $st") ;;
        esac
    done
    if [ "$(probe_field "$out" SVC_MAPPING)" = "active" ] && ! mapping_nodes_complete "$out"; then
        age=$(mapping_active_age "$out")
        if [ -n "$age" ] && [ "$age" -lt "$PI_MAPPING_GRACE_SEC" ]; then
            r+=("gripperx-mapping active for ${age}s, its nodes not all up yet")
        fi
    fi
    [ "${#r[@]}" -eq 0 ] && return 0
    local IFS=';'
    printf '%s\n' "${r[*]}" | sed 's/;/; /g'
}

pi_health_check() {
    log "== Pi health check (read-only, ssh $SSH_HOST) =="
    local out reasons elapsed stopping settle_start=$SECONDS announced=0 stop_announced=0
    PI_TRANSITION_LEFT=""
    while :; do
        pi_probe || return 1
        out="$PI_PROBE_OUT"
        stopping=$(pi_stopping_only "$out")
        if [ -n "$stopping" ] && [ "$stop_announced" = 0 ]; then
            warn "Pi stack is being STOPPED (gripperx-stack-stop, mode: $stopping) - nothing will"
            warn "  bring it back by itself (stop: stack stays down; halt: the Pi powers off),"
            warn "  so this run does NOT wait for a return. Units still shutting down are waited"
            warn "  for; after that the usual checks decide, and an unreachable Pi is 'unknown'."
            stop_announced=1
        fi
        reasons=$(pi_transition_reasons "$out")
        if [ -z "$reasons" ]; then
            [ "$announced" = 1 ] && ok "Pi stack settled after $((SECONDS - settle_start)) s - re-evaluating."
            break
        fi
        elapsed=$((SECONDS - settle_start))
        if [ "$announced" = 0 ]; then
            warn "Pi stack is IN TRANSITION: $reasons"
            warn "  Its state now says nothing about its state in a minute - e.g. a Mode R"
            warn "  restart (short press of the shutdown button) brings mapping back by itself."
            warn "  Waiting up to ${PI_SETTLE_MAX_SEC} s before deciding anything; nothing has been"
            warn "  started yet. Ctrl-C aborts. (GRIPPERX_DESK_PI_SETTLE_SEC sets the bound.)"
            announced=1
        fi
        if [ "$elapsed" -ge "$PI_SETTLE_MAX_SEC" ]; then
            PI_TRANSITION_LEFT="$reasons"
            warn "Pi stack still in transition after ${elapsed} s: $reasons"
            warn "  Giving up the wait. Mapping evidence will be 'unsettled': no local"
            warn "  mapping stack is started on it (see the local mapping line below)."
            break
        fi
        log "  waiting for the Pi stack (${elapsed}/${PI_SETTLE_MAX_SEC} s): $reasons"
        interruptible_sleep "$PI_SETTLE_POLL_SEC"
    done

    local uptime_line bringup_proc n_arm n_steer n_swerve n_jcb n_rsp n_rc n_lidar agent_ct
    local svc_mapping n_rf2o n_slam svc_bringup n_mux n_lidpwr n_scanfilt n_lsm n_ekf
    uptime_line=$(sed -n 's/^UPTIME_LINE: //p' <<<"$out")
    bringup_proc=$(sed -n 's/^BRINGUP_PROC: //p' <<<"$out")
    n_arm=$(sed -n 's/^N_ARM: //p' <<<"$out")
    n_steer=$(sed -n 's/^N_STEER: //p' <<<"$out")
    n_swerve=$(sed -n 's/^N_SWERVE: //p' <<<"$out")
    n_jcb=$(sed -n 's/^N_JCB: //p' <<<"$out")
    n_rsp=$(sed -n 's/^N_RSP: //p' <<<"$out")
    n_rc=$(sed -n 's/^N_RC: //p' <<<"$out")
    n_lidar=$(sed -n 's/^N_LIDAR: //p' <<<"$out")
    n_scanfilt=$(sed -n 's/^N_SCANFILT: //p' <<<"$out")
    agent_ct=$(sed -n 's/^AGENT_CT: //p' <<<"$out")
    svc_mapping=$(sed -n 's/^SVC_MAPPING: //p' <<<"$out")
    n_rf2o=$(sed -n 's/^N_RF2O: //p' <<<"$out")
    n_lsm=$(sed -n 's/^N_LSM: //p' <<<"$out")
    n_ekf=$(sed -n 's/^N_EKF: //p' <<<"$out")
    n_slam=$(sed -n 's/^N_SLAM: //p' <<<"$out")
    svc_bringup=$(sed -n 's/^SVC_BRINGUP: //p' <<<"$out")
    n_mux=$(sed -n 's/^N_MUX: //p' <<<"$out")
    n_lidpwr=$(sed -n 's/^N_LIDPWR: //p' <<<"$out")

    ok "Pi reachable. $uptime_line"
    # Process AND service state, because they can disagree: a hand-launched
    # "ros2 launch real_robot.launch.py" keeps the nodes alive while
    # gripperx-bringup.service is inactive. That combination is dangerous, not
    # cosmetic - gripperx-mapping.service has Requires=gripperx-bringup.service,
    # so restarting mapping would then pull up a SECOND bringup that fights the
    # hand-launched one over /dev/steering_servo and /dev/arm_servo.
    if [ "${bringup_proc:-0}" -ge 1 ] 2>/dev/null; then
        if [ "$svc_bringup" = "active" ]; then
            ok "bringup: gripperx-bringup.service active, real_robot.launch.py running."
        else
            warn "bringup: real_robot.launch.py IS running, but gripperx-bringup.service is"
            warn "  ${svc_bringup:-unknown} - this is a hand-launched instance, not the service."
            warn "  Do NOT restart gripperx-mapping in this state (Requires= would start a"
            warn "  second bringup and both would fight over the servo serial ports)."
        fi
    else
        warn "bringup (real_robot.launch.py) process NOT found (service: ${svc_bringup:-unknown})"
        warn "  - teleop will have nothing to talk to."
    fi
    # Required set per NFR-10 (see the header): no swerve_cmd_node and no
    # joint_command_bridge, which are not started any more, but teleop_mux_node,
    # without which the teleop cannot reach the drive. swerve_controller is
    # deliberately absent - it is a controller_manager plugin inside
    # ros2_control_node, not a process.
    local missing=()
    [ "${n_rsp:-0}" -ge 1 ] 2>/dev/null || missing+=("robot_state_publisher")
    [ "${n_rc:-0}" -ge 1 ] 2>/dev/null || missing+=("ros2_control_node")
    [ "${n_lidar:-0}" -ge 1 ] 2>/dev/null || missing+=("ldlidar_stl_ros2_node")
    # Not cosmetic: the LiDAR driver publishes /scan_RAW, and scan_range_filter is
    # what republishes /scan with the self-returns (arm, gripper, all below 0.10 m)
    # removed. If it is not running there is NO /scan at all - not unfiltered data,
    # none - so slam_toolbox, both costmaps and RViz go quiet together.
    [ "${n_scanfilt:-0}" -ge 1 ] 2>/dev/null || missing+=("scan_range_filter")
    [ "${n_steer:-0}" -ge 1 ] 2>/dev/null || missing+=("steer_servo_node")
    [ "${n_mux:-0}" -ge 1 ] 2>/dev/null || missing+=("teleop_mux_node")
    [ "${n_arm:-0}" -ge 1 ] 2>/dev/null || missing+=("arm_action_server")
    if [ "${#missing[@]}" -eq 0 ]; then
        ok "Key nodes present: robot_state_publisher, ros2_control_node, ldlidar, scan_range_filter, steer_servo_node, teleop_mux_node, arm_action_server"
        ok "  (swerve_controller is a plugin inside ros2_control_node, not a process - not checkable here; its spawner is one-shot and long gone.)"
    else
        warn "Key node(s) not found: ${missing[*]}"
    fi
    # lidar_power_node is reported separately, not as a missing key node: it is
    # known to be orphaned/crashed by SIGTERM (open item in the handover), so a
    # zero here is a known weakness, not necessarily a broken bringup.
    if [ "${n_lidpwr:-0}" -ge 1 ] 2>/dev/null; then
        ok "lidar_power_node running (LiDAR power relay)."
    else
        warn "lidar_power_node not found - known SIGTERM orphan/crash issue; LiDAR power relay may be unmanaged."
    fi
    # Opposite polarity, and this one IS a fault: /hw/joint_commands must have
    # exactly 2 publishers (GripperXInterface + watchdog, D4/OP-18b/SR-10). A
    # leftover swerve_cmd_node or joint_command_bridge makes a third one and two
    # sources fight over the hardware boundary.
    local legacy=()
    [ "${n_swerve:-0}" -ge 1 ] 2>/dev/null && legacy+=("swerve_cmd_node x$n_swerve")
    [ "${n_jcb:-0}" -ge 1 ] 2>/dev/null && legacy+=("joint_command_bridge x$n_jcb")
    if [ "${#legacy[@]}" -gt 0 ]; then
        warn "PRE-NFR-10 NODE(S) RUNNING: ${legacy[*]}"
        warn "  These are out of the active path since NFR-10 and are NOT started by"
        warn "  real_robot.launch.py any more. Running, they are a THIRD publisher on"
        warn "  /hw/joint_commands (normal state is exactly 2) and fight swerve_controller"
        warn "  over the hardware boundary. Someone hand-started control.launch.py, or an"
        warn "  old bringup survived. Investigate before driving. Nothing touched from here."
    fi
    if [ -n "${agent_ct:-}" ]; then
        ok "micro-ROS agent container: $agent_ct"
    else
        warn "micro-ROS agent container (mros_agent) not found via docker ps."
    fi
    # "The Pi is mapping" = the service is up, slam_toolbox is running, and SOME
    # odometry source of the current stack is running with it. Either of
    # laser_scan_matcher and ekf_node counts: the launch file starts the EKF in
    # four mutually exclusive variants under different arguments, and this script
    # has no business predicting which one systemd picked - it only needs to know
    # that the odom edge on domain $REAL_DOMAIN is already owned by somebody.
    # "inactive" is claimed only for a SETTLED Pi: a transition the wait above gave
    # up on, or a mapping unit that is active without its full node set, is a Pi
    # whose own mapping can (re)appear beside ours - "unsettled", refused like
    # "unknown". Until 2026-10-16 the active-but-incomplete case counted as
    # "inactive" and let the local stack start.
    if [ "$svc_mapping" = "active" ] && mapping_nodes_complete "$out"; then
        PI_MAPPING_EVIDENCE="active"
    elif [ -n "$PI_TRANSITION_LEFT" ] || [ "$svc_mapping" = "active" ]; then
        PI_MAPPING_EVIDENCE="unsettled"
    else
        PI_MAPPING_EVIDENCE="inactive"
    fi
    if [ "${n_rf2o:-0}" -ge 1 ] 2>/dev/null; then
        warn "rf2o_laser_odometry is running ON THE Pi (x$n_rf2o). Since a52379d the mapping"
        warn "  service starts the EKF instead, and rf2o and the EKF BOTH publish"
        warn "  odom->base_footprint - two publishers of one TF edge, tf2 serves whichever"
        warn "  arrived last. Someone hand-started mapping.launch.py. Investigate."
    fi
    mapping_check "$svc_mapping" "${n_lsm:-0}" "${n_slam:-0}" "${n_ekf:-0}"
    warn "Known-benign noise WHILE ACTUATOR POWER IS OFF: steer_servo_node respawn loop /"
    warn "  arm_action_server retry spam - not a fault. If those are ABSENT the servo bus is"
    warn "  answering, i.e. actuator power is ON and the steering can move (handover"
    warn "  2026-08-19: power was left ON). This script cannot read the power state itself."
    return 0
}

# --- mapping / SLAM status (part of the read-only health check) --------------
# Answers the one question the user actually has before opening RViz: will the
# live map appear? Two independent levels of evidence - the Pi-side service and
# process state (already fetched by the ssh block above, no DDS traffic), and
# the topic level as seen from THIS laptop on domain 20, which is exactly what
# RViz will see. STRICTLY READ-ONLY: if mapping is down, the command to start
# it is printed, never executed - starting Pi services is the user's call.
mapping_check() { # svc_state n_laser_scan_matcher n_slam n_ekf
    local svc="$1" n_lsm="$2" n_slam="$3" n_ekf="${4:-0}"
    if [ "$svc" = "active" ] && [ "$n_slam" -ge 1 ] 2>/dev/null \
       && { [ "$n_lsm" -ge 1 ] 2>/dev/null || [ "$n_ekf" -ge 1 ] 2>/dev/null; }; then
        ok "gripperx-mapping.service active (laser_scan_matcher x$n_lsm, ekf_node x$n_ekf, async_slam_toolbox x$n_slam)."
    elif [ "$svc" = "active" ]; then
        warn "gripperx-mapping.service is active but nodes are missing (laser_scan_matcher=$n_lsm,"
        warn "  ekf_node=$n_ekf, slam_toolbox=$n_slam). Expected set since a52379d:"
        warn "  localization.launch.py -> laser_odometry_node + ekf_filter_node + slam_toolbox."
    else
        warn "gripperx-mapping.service: ${svc:-unknown} - NO live map and NO /odom."
        warn "RViz will open on Fixed Frame 'map' and stay empty. Start it yourself:"
        warn "    ssh $SSH_HOST 'sudo systemctl start gripperx-mapping'"
    fi

    local topics
    topics=$(timeout 20 bash -c "$REAL_PRELUDE exec ros2 topic list" 2>/dev/null)
    if [ -z "$topics" ]; then
        warn "Could not read the topic list on domain $REAL_DOMAIN (timeout) - skipping the topic check."
        return 0
    fi
    local t missing=()
    for t in /map /scan /robot_description; do
        grep -qx -- "$t" <<<"$topics" || missing+=("$t")
    done
    if [ "${#missing[@]}" -eq 0 ]; then
        ok "Topics for the live view present on domain $REAL_DOMAIN: /map /scan /robot_description."
    else
        warn "Missing on domain $REAL_DOMAIN: ${missing[*]} - the corresponding RViz display stays empty."
    fi
    # /odom is reported SEPARATELY and NOT as a required topic, because who
    # publishes it is unverified: rf2o published /odom and rf2o is gone; the EKF's
    # own output topic is /odometry/filtered, and one remapping in
    # localization.launch.py points /odometry/filtered at /wheel/odom for one of the
    # four EKF variants. Rather than guess which topic carries odometry on this
    # robot, both are reported and neither is called a fault.
    for t in /odom /odometry/filtered; do
        if grep -qx -- "$t" <<<"$topics"; then
            ok "  odometry topic present on domain $REAL_DOMAIN: $t"
        else
            log "  odometry topic absent on domain $REAL_DOMAIN: $t (informational - the producer"
            log "    of /odom changed with a52379d and is not asserted here)"
        fi
    done

    # What this launcher will do about it. The Pi-side service is never started
    # from here (see the header: no Pi state changes at all).
    case "${LOCAL_MAP,,}" in
        off|0|no|skip)
            log "local mapping: disabled by request - no /map will be produced from this laptop." ;;
        *)
            if [ "$PI_MAPPING_EVIDENCE" = "active" ]; then
                log "local mapping: will be SKIPPED - the Pi already publishes /map and odom->base_footprint."
            elif [ "$PI_MAPPING_EVIDENCE" = "unknown" ] && [ "${LOCAL_MAP,,}" != "force" ]; then
                log "local mapping: will be SKIPPED - the Pi could not be asked, and this script does"
                log "  not start a mapping stack on domain $REAL_DOMAIN on a guess. --force-local-map overrides."
            elif [ "$PI_MAPPING_EVIDENCE" = "unsettled" ] && [ "${LOCAL_MAP,,}" != "force" ]; then
                log "local mapping: will be SKIPPED - the Pi stack has not settled, so its mapping can"
                log "  still come up beside ours on domain $REAL_DOMAIN. --force-local-map overrides."
            else
                log "local mapping: will be STARTED on this laptop (rf2o + slam_toolbox, domain $REAL_DOMAIN)."
                log "  Laser odometry then runs over WiFi - the known weak point of this variant."
            fi ;;
    esac
}

# --- other-track collision awareness (read-only) ------------------------------
# ROS_DOMAIN_ID keeps the tracks apart on DDS, but nothing keeps them apart on
# this laptop's CPU, GPU or - for Gazebo - on gz-transport, which ignores
# ROS_DOMAIN_ID completely. This reports, it never acts: no PID is signalled, no
# domain is avoided, no start is refused here. Refusing is the job of
# refuse_if_other_teleop() and of the twin collision guard, which are about
# ambiguity of control, not about mere co-residence.
other_track_check() { # domain_we_are_about_to_use
    local ours="$1" d pid pids comm found=0
    log "== other-track check (read-only, nothing is killed) =="
    # No owner names - see "Domain ownership" in the header. Attributing a PID to
    # a named track is a guess; the PID itself is not.
    for d in "${FOREIGN_DOMAINS[@]}"; do
        # Two buckets, and only one of them is news. See is_ros2_cli_daemon():
        # a ros2 CLI daemon carries the domain in its environ but is not another
        # session at work - it is the leftover of somebody's (often our own)
        # "ros2 topic list", and counting it as activity makes this check cry wolf
        # on an empty domain.
        local real_pids="" daemon_pids=""
        for pid in $(pids_on_domain "$d"); do
            if is_ros2_cli_daemon "$pid"; then
                daemon_pids="$daemon_pids $pid"
            else
                real_pids="$real_pids $pid"
            fi
        done
        real_pids="${real_pids# }"
        daemon_pids="${daemon_pids# }"
        if [ -n "$real_pids" ]; then
            found=1
            if [ "$d" = "$ours" ]; then
                warn "Domain $d is ALREADY IN USE and it is the domain this run needs:"
                warn "  PID(s): $real_pids"
                warn "  Same domain means one shared DDS graph: node names, /tf and /clock"
                warn "  collide. Coordinate with that session before continuing."
            else
                log "  Domain $d busy (another session) - PID(s): $real_pids. Not our domain, informational only."
            fi
        fi
        # Deliberately a plain log line and deliberately worded as bookkeeping:
        # it must not read like "another track is here".
        [ -n "$daemon_pids" ] && \
            log "  Domain $d: idle ros2 CLI daemon(s) only (PID(s): $daemon_pids) - not a session, no action needed."
    done
    pids=$(gazebo_pids | tr '\n' ' ')
    pids="${pids% }"
    if [ -n "$pids" ]; then
        found=1
        warn "A Gazebo is already running (PID(s): $pids) - regardless of ROS_DOMAIN_ID."
        warn "  gz-transport namespaces by GZ_PARTITION, not by domain. A foreign Gazebo on"
        warn "  the default partition has captured a spawn before (SR-8). This script's twin"
        warn "  runs in partition '$GZ_PARTITION_TWIN', so it is isolated from it - but the"
        warn "  two still share the GPU. Nothing was touched."
    fi
    [ "$found" = 0 ] && ok "No other-session processes on domain ${FOREIGN_DOMAINS[*]}, no Gazebo running."
    return 0
}

# --- steering safety gate (real mode) ----------------------------------------
# See the "Steering safety gate" paragraph in the header for the full rationale.
# Short version: starting keyboard_teleop_node IS a movement command. It comes up
# in CORNERING and immediately publishes angle*STEER_PATTERN (angle=0.0) on
# /teleop/direct_steer at 50 Hz, which steer_servo_node applies as an override -
# all four wheels are driven to straight-ahead without any key press. The user's
# standing rule (per test, never blanket) requires explicit approval for exactly
# that, so it is asked for here, with the current angles on screen.

# Current steering angles in DEGREES, one line, joint order FL FR BL BR.
# Source: /hw/steer_states (std_msgs/Float64MultiArray, RADIANS), published by
# steer_servo_node from servo read-back. Preferred over TF because it is the
# servo's own measurement and needs neither robot_state_publisher nor odometry
# to be alive - TF would additionally go stale silently, this topic simply stops.
read_steer_states_deg() {
    local raw vals
    raw=$(timeout 15 bash -c "$REAL_PRELUDE exec ros2 topic echo --once /hw/steer_states" 2>/dev/null)
    [ -n "$raw" ] || return 1
    # Range from the "data:" line to the message separator. "data_offset:" inside
    # the layout block does NOT match "^data:" and is therefore not picked up.
    # Both YAML styles are handled (block "- 0.5" and flow "[0.5, -0.5]").
    vals=$(sed -n '/^data:/,/^---/p' <<<"$raw" \
           | grep -oE '[-+]?[0-9]+\.?[0-9]*([eE][-+]?[0-9]+)?' | tr '\n' ' ')
    [ -n "${vals// /}" ] || return 1
    awk '{for(i=1;i<=NF;i++) printf "%s%+.1f", (i>1?" ":""), $i*57.2957795131; print ""}' <<<"$vals"
}

# "ros2 lifecycle get" prints "<state> [<id>]", e.g. "active [3]",
# "inactive [2]", "unconfigured [1]". Only the bare state word is returned, and
# every caller compares it with "=", NEVER with a glob: "inactive" CONTAINS the
# substring "active", so [[ $state == *active* ]] is true for a node that is
# merely configured. The activation loop then breaks before it ever calls
# "lifecycle set activate" and reports success, while slam_toolbox sits in
# "inactive [2]" and publishes no map at all.
lifecycle_state_of() { # node
    timeout 10 bash -c "$REAL_PRELUDE exec ros2 lifecycle get '$1'" 2>/dev/null \
        | awk 'NR==1{print $1}'
}

steer_safety_gate() {
    echo
    log "== STEERING SAFETY GATE (real mode) =="
    log "  Starting the teleop is itself a movement command: keyboard_teleop_node"
    log "  comes up in state CORNERING and publishes /teleop/direct_steer at 50 Hz"
    log "  with angle 0.0, which steer_servo_node applies as an override - all four"
    log "  wheels are commanded to STRAIGHT AHEAD (0 deg) at once, no key press."

    # The angles are read and shown BEFORE the bypass is honoured, on purpose.
    # --no-steer-gate is an assertion that the actuator power is off; a live
    # reading on /hw/steer_states is the one piece of evidence that can expose
    # that assertion as wrong, so suppressing it in exactly the case where the
    # user has stopped asking would hide the only warning that matters. The
    # bypass skips the CONFIRMATION, never the measurement.
    local degs="" maxabs="" readable=0
    if degs=$(read_steer_states_deg) && [ -n "$degs" ]; then
        readable=1
        ok "  Current angles /hw/steer_states (FL FR BL BR): $degs deg"
        maxabs=$(awk '{m=0;for(i=1;i<=NF;i++){v=$i<0?-$i:$i; if(v>m)m=v} printf "%.1f", m}' <<<"$degs")
        warn "  Largest wheel swing if you continue: about $maxabs deg."
    else
        warn "  Could NOT read /hw/steer_states on domain $REAL_DOMAIN within 15 s."
        warn "  Either the Pi is unreachable, or steer_servo_node is not publishing."
        warn "  The current wheel angles are therefore UNKNOWN. This gate does not fall"
        warn "  through on a failed read - unknown is treated as worst case (the handover"
        warn "  of 2026-08-19 records the steering parked at about +-33 deg with power ON)."
    fi

    case "${STEER_GATE,,}" in
        off|0|no|skip)
            warn "  Confirmation BYPASSED (GRIPPERX_DESK_STEER_GATE=$STEER_GATE / --no-steer-gate)."
            warn "  That is an assertion that the actuator power is OFF, not a measurement."
            if [ "$readable" = 1 ]; then
                warn "  Note that the angles above WERE readable, i.e. steer_servo_node is"
                warn "  publishing servo read-back on domain $REAL_DOMAIN. Compare them against your"
                warn "  assumption before continuing - if the power is on, those wheels move now."
            fi
            return 0
            ;;
    esac

    if [ ! -t 0 ]; then
        warn "  Refusing: stdin is not a terminal, so the confirmation cannot be typed."
        warn "  Run this from an interactive shell, or set GRIPPERX_DESK_STEER_GATE=off"
        warn "  if you know the actuator power is OFF. Teleop NOT started."
        return 1
    fi

    local answer=""
    read -r -p "[gripperx_desk] Type MOVE to allow this steering motion (anything else aborts): " answer || answer=""
    if [ "$answer" != "MOVE" ]; then
        warn "  Not confirmed - teleop NOT started. Nothing was commanded, nothing moved."
        warn "  Everything else this run started (RViz, local mapping) keeps running."
        return 1
    fi
    ok "  Confirmed for this start. Approval is per test, it does not carry over."
    return 0
}

# --- local mapping stack (real mode) -----------------------------------------
# Starts gripperx_bringup/mapping.launch.py (rf2o_laser_odometry + async
# slam_toolbox with the repo config) on THIS laptop, on the real domain, plus the
# lifecycle configure/activate dance that gripperx-mapping.sh performs - a
# lifecycle node comes up "unconfigured" and publishes nothing until activated.
# Nothing is started, stopped or enabled on the Pi.
# KNOWN WEAK POINT: rf2o consumes /scan over WiFi, so dropped or late scans become
# odometry error and the map smears. The Pi-side gripperx-mapping.service, where
# /scan never crosses the link, remains the better place for this.
LOCAL_MAP_STATE="not started"
# 1 only if THIS run actually launched the local stack. Every skip path leaves it
# at 0, and the collision watch below must not run in those cases: without a local
# stack there is nothing of ours to collide, and the Pi's own nodes are the normal,
# correct picture rather than a finding.
LOCAL_MAP_OURS=0

start_local_mapping() {
    case "${LOCAL_MAP,,}" in
        off|0|no|skip)
            LOCAL_MAP_STATE="disabled by request"
            log "local mapping: disabled (GRIPPERX_DESK_LOCAL_MAP=$LOCAL_MAP / --no-local-map)."
            return 0
            ;;
    esac
    log "== local mapping: rf2o + slam_toolbox on domain $REAL_DOMAIN =="

    # Detect FIRST, start second. Two publishers of the odom->base_footprint edge
    # are a real failure mode, not a cosmetic one: the TF tree becomes ambiguous,
    # tf2 serves whichever transform arrived last, and the robot jumps in RViz.
    if [ "$PI_MAPPING_EVIDENCE" = "active" ]; then
        LOCAL_MAP_STATE="skipped (Pi already mapping)"
        ok "local mapping: SKIPPED - gripperx-mapping is active on the Pi (rf2o + slam_toolbox)."
        ok "  The Pi already owns /map and odom->base_footprint. Two publishers of that"
        ok "  edge would make the TF tree ambiguous - use the Pi's map."
        return 0
    fi
    # UNSETTLED IS NOT "NO" EITHER. The Pi answered, but its stack was in a
    # transition the health check gave up waiting for (a restart, a start job, a
    # unit crash-looping through Restart=on-failure), or mapping is active with an
    # incomplete node set. Either way its own slam_toolbox + odometry can appear
    # beside ours at any moment - the 2026-10-16 collision. Refused before the DDS
    # probe: an empty graph now would prove nothing about a minute from now.
    if [ "$PI_MAPPING_EVIDENCE" = "unsettled" ] && [ "${LOCAL_MAP,,}" != "force" ]; then
        LOCAL_MAP_STATE="skipped (Pi stack unsettled - refused to guess)"
        warn "local mapping: NOT started - the Pi stack had not settled:"
        warn "  ${PI_TRANSITION_LEFT:-gripperx-mapping is active but its node set is incomplete}"
        warn "  A Pi whose mapping is coming back (or stuck half-up) is not a Pi that is not"
        warn "  mapping: a local stack would become a second publisher of odom->base_footprint"
        warn "  and a second /slam_toolbox on domain $REAL_DOMAIN."
        warn "  Check it yourself:  ssh $SSH_HOST 'systemctl is-active gripperx-mapping; systemctl list-jobs'"
        warn "  Then re-run this script once it has settled, or, if you KNOW the Pi will not"
        warn "  map, with --force-local-map (that asserts it; it does not verify it)."
        return 1
    fi
    local nodes hits
    nodes=$(timeout 20 bash -c "$REAL_PRELUDE exec ros2 node list" 2>/dev/null)
    # Node NAMES as they appear in "ros2 node list", covering both stacks: the
    # Pi's current one (laser_odometry_node + ekf_filter_node + slam_toolbox,
    # a52379d) and the older rf2o one, which this laptop still starts itself and
    # which another laptop session might therefore have running.
    hits=$(grep -E 'rf2o_laser_odometry|laser_odometry_node|ekf_filter_node|slam_toolbox' \
           <<<"${nodes:-}" | tr '\n' ' ')
    if [ -n "${hits// /}" ]; then
        LOCAL_MAP_STATE="skipped (mapping node already in DDS graph)"
        warn "local mapping: SKIPPED - already in the DDS graph on domain $REAL_DOMAIN: $hits"
        warn "  Could be the Pi (ssh may have been unreachable, so the service state is"
        warn "  ${PI_MAPPING_EVIDENCE}) or another laptop session. Not starting a second one."
        return 0
    fi
    # UNKNOWN IS NOT "NO". Reaching this line with evidence "unknown" means BOTH
    # observations came back empty-handed: ssh could not be completed (a loaded Pi
    # can let every attempt time out while it is healthy and mapping), and the DDS
    # graph above showed no rf2o/slam_toolbox, which on a link that cannot see the
    # robot at all looks exactly like a robot that is not mapping. Starting would put a
    # SECOND publisher of odom->base_footprint on the REAL robot's domain on the
    # strength of two failures to observe. Refuse instead, and say what to do.
    if [ "$PI_MAPPING_EVIDENCE" = "unknown" ] && [ "${LOCAL_MAP,,}" != "force" ]; then
        LOCAL_MAP_STATE="skipped (Pi state unknown - refused to guess)"
        warn "local mapping: NOT started - the Pi could not be asked (ssh failed) and the DDS"
        warn "  graph on domain $REAL_DOMAIN showed no rf2o/slam_toolbox either. Those two"
        warn "  together are 'we cannot see the robot', NOT 'the robot is not mapping', and"
        warn "  the difference matters: if gripperx-mapping IS running on the Pi, a local"
        warn "  stack becomes a second publisher of odom->base_footprint, tf2 serves whichever"
        warn "  arrived last and the robot jumps around in RViz."
        warn "  Check it yourself:  ssh $SSH_HOST 'systemctl is-active gripperx-mapping'"
        warn "  If it is really down and you want the laptop-side map, re-run with"
        warn "  --force-local-map (that asserts the Pi is not mapping; it does not verify it)."
        return 1
    fi

    local logfile="$LOG_DIR/local_mapping_${REAL_DOMAIN}.log"
    bash -c "$REAL_PRELUDE exec ros2 launch gripperx_bringup mapping.launch.py" >"$logfile" 2>&1 &
    local lpid=$!
    # Tracked AT LAUNCH, here and at every other launch: a Ctrl-C in the startup
    # wait below must still find it. The pattern is in the wrapper's own argv
    # (bash -c "... exec <pattern> ..."), so the PID-reuse check holds before
    # and after the exec; a launch that dies is reported "already gone".
    track "local mapping launch" "$lpid" "gripperx_bringup mapping.launch.py"
    sleep 3
    if ! kill -0 "$lpid" 2>/dev/null; then
        LOCAL_MAP_STATE="FAILED to start"
        warn "local mapping: launch exited immediately - see $logfile"
        return 1
    fi
    LOCAL_MAP_OURS=1
    ok "local mapping: launch started (PID $lpid, log: $logfile)"

    local waited=0 rf2o_pid="" slam_pid=""
    while [ "$waited" -lt 30 ]; do
        [ -n "$rf2o_pid" ] || rf2o_pid=$(pids_for_pattern_on_domain "rf2o_laser_odometry_node" "$REAL_DOMAIN" | head -n1)
        [ -n "$slam_pid" ] || slam_pid=$(pids_for_pattern_on_domain "async_slam_toolbox_node" "$REAL_DOMAIN" | head -n1)
        [ -n "$rf2o_pid" ] && [ -n "$slam_pid" ] && break
        sleep 1
        waited=$((waited + 1))
    done
    if [ -n "$rf2o_pid" ]; then
        track "local rf2o_laser_odometry" "$rf2o_pid" "rf2o_laser_odometry_node"
        ok "local mapping: rf2o_laser_odometry up (PID $rf2o_pid) - /scan -> /odom + odom->base_footprint."
    else
        warn "local mapping: rf2o_laser_odometry did not appear within 30 s - see $logfile"
    fi
    if [ -n "$slam_pid" ]; then
        track "local slam_toolbox" "$slam_pid" "async_slam_toolbox_node"
    else
        LOCAL_MAP_STATE="rf2o only, slam_toolbox missing"
        warn "local mapping: async_slam_toolbox_node did not appear within 30 s - see $logfile"
        return 1
    fi

    # Lifecycle dance, same order as gripperx-mapping.sh, but BOUNDED: that
    # script runs under systemd and may loop forever, this one must never hang
    # an interactive session.
    local tries state=""
    for tries in $(seq 1 15); do
        state=$(lifecycle_state_of /slam_toolbox)
        case "$state" in unconfigured|inactive|active) break ;; esac
        sleep 2
    done
    case "$state" in
        unconfigured|inactive|active) ;;
        *)
            LOCAL_MAP_STATE="slam_toolbox not answering lifecycle (${state:-no reply})"
            warn "local mapping: /slam_toolbox does not answer lifecycle queries (${state:-no reply}) - see $logfile"
            return 1
            ;;
    esac
    if [ "$state" = "unconfigured" ]; then
        for tries in $(seq 1 10); do
            timeout 15 bash -c "$REAL_PRELUDE exec ros2 lifecycle set /slam_toolbox configure" 2>&1 \
                | grep -q successful && break
            sleep 1
        done
        state=$(lifecycle_state_of /slam_toolbox)
    fi
    if [ "$state" = "inactive" ]; then
        for tries in $(seq 1 10); do
            timeout 15 bash -c "$REAL_PRELUDE exec ros2 lifecycle set /slam_toolbox activate" 2>&1 \
                | grep -q successful && break
            sleep 1
        done
    fi
    state=$(lifecycle_state_of /slam_toolbox)
    if [ "$state" = "active" ]; then
        LOCAL_MAP_STATE="active (rf2o PID ${rf2o_pid:-?}, slam_toolbox PID $slam_pid)"
        ok "local mapping: slam_toolbox is ACTIVE - the map is being built."
        warn "local mapping: no /scan means no map. rf2o pulls the scan over WiFi; that link"
        warn "  is the known weak point of running mapping here instead of on the Pi."
    else
        LOCAL_MAP_STATE="slam_toolbox stuck in '${state:-unknown}'"
        warn "local mapping: slam_toolbox did not reach 'active' (state: ${state:-unknown}) - see $logfile"
        warn "  It is running but publishes no map in this state."
        return 1
    fi
    return 0
}

# --- mapping collision watch (real mode) -------------------------------------
# THE GUARD IN start_local_mapping CHECKS ONCE, AT START, so it cannot see a Pi-side
# mapping stack that is started afterwards. From that moment domain 20 carries two
# publishers of odom->base_footprint and two nodes named /slam_toolbox, and three
# things follow: RViz jumps, which is what an ambiguous TF tree looks like; the Pi's
# own lifecycle configure can land on OUR slam_toolbox, because lifecycle calls
# address a node by name, leaving the Pi's node "unconfigured" while its service
# reports "building map"; and stopping this script then removes the only /map
# publisher and the only map->odom transform on the domain, so the next RViz comes
# up empty and drops every message for a transform that no longer exists.
# This watch re-reads the graph while the run
# is alive, so the collision is named when it appears rather than reconstructed from logs
# afterwards. It only ever REPORTS: killing a mapping stack under a running session is a
# decision for the operator, and both candidate stacks may be feeding something.
MAP_WATCH_PID=""
MAP_WATCH_INTERVAL="${GRIPPERX_DESK_MAP_WATCH_INTERVAL:-15}"
MAP_WATCH_FLAG="$LOG_DIR/map_collision_seen"

start_map_collision_watch() {
    [ "$LOCAL_MAP_OURS" = 1 ] || return 0
    case "${MAP_WATCH_INTERVAL,,}" in
        off|0|no)
            log "mapping collision watch: disabled (GRIPPERX_DESK_MAP_WATCH_INTERVAL=$MAP_WATCH_INTERVAL)."
            return 0
            ;;
    esac
    rm -f "$MAP_WATCH_FLAG"
    (
        local_seen=0
        while sleep "$MAP_WATCH_INTERVAL"; do
            nodes=$(timeout 20 bash -c "$REAL_PRELUDE exec ros2 node list" 2>/dev/null)
            # An empty answer is a failed observation, not an all-clear - same rule the
            # start-time guard follows. Say nothing and look again next cycle.
            [ -n "${nodes//[[:space:]]/}" ] || continue
            slam_dupes=$(grep -cx '/slam_toolbox' <<<"$nodes")
            pi_nodes=$(grep -Ex '/(laser_odometry_node|ekf_filter_node)' <<<"$nodes" | tr '\n' ' ')
            if [ "${slam_dupes:-0}" -le 1 ] && [ -z "${pi_nodes// /}" ]; then
                local_seen=0
                continue
            fi
            : > "$MAP_WATCH_FLAG"
            if [ "$local_seen" = 0 ]; then
                warn "MAPPING COLLISION on domain $REAL_DOMAIN - the Pi started mapping while"
                warn "  this script's own stack is running. Evidence: ${slam_dupes} node(s) named"
                warn "  /slam_toolbox${pi_nodes:+, plus Pi mapping nodes: ${pi_nodes% }}."
                warn "  Two publishers of odom->base_footprint: tf2 serves whichever arrived"
                warn "  last and the robot jumps in RViz. Worse, lifecycle calls address a node"
                warn "  by NAME, so the Pi's configure/activate can land on OUR slam_toolbox and"
                warn "  leave the Pi's own node unconfigured - with no symptom until this script"
                warn "  exits and takes the only working map publisher with it."
                warn "  Resolve it - keep the Pi's stack, drop ours:"
                warn "    stop this script (Ctrl-C), then re-run it without a local map:"
                warn "      $0 real --no-local-map"
                warn "    and on the robot, repair the node the collision may have left behind:"
                warn "      ssh $SSH_HOST 'sudo systemctl restart gripperx-mapping'"
            else
                warn "MAPPING COLLISION still present (${slam_dupes} x /slam_toolbox${pi_nodes:+, ${pi_nodes% }})."
            fi
            local_seen=1
        done
    ) &
    MAP_WATCH_PID=$!
    log "mapping collision watch: running (PID $MAP_WATCH_PID, every ${MAP_WATCH_INTERVAL}s)."
}

# --- RViz --------------------------------------------------------------------

start_rviz() { # prelude config domain label
    local prelude="$1" config="$2" domain="$3" label="$4"
    local logfile="$LOG_DIR/rviz_${domain}.log"
    bash -c "$prelude exec rviz2 -d '$config'" >"$logfile" 2>&1 &
    local pid=$!
    track "$label RViz" "$pid" "rviz2"   # at launch - see start_local_mapping
    sleep 1
    if ! kill -0 "$pid" 2>/dev/null; then
        warn "$label RViz (domain $domain) exited immediately - see $logfile"
        return 1
    fi
    ok "$label RViz started (PID $pid, config: $config, log: $logfile)"
}

# --- teleop -------------------------------------------------------------------
# One entry point, two front-ends. Everything that is about SAFETY rather than
# about the input device lives here, above the branch: the duplicate check, the
# rival-front-end check and the steering gate. Neither branch can skip them, and
# a future third front-end would inherit them by construction.

start_teleop() { # prelude domain label mode
    local prelude="$1" domain="$2" label="$3" mode="$4"
    local node_pattern rival_pattern existing rivals
    node_pattern=$(node_pattern_for_input "$INPUT")
    rival_pattern=$(rival_pattern_for_input "$INPUT")

    existing=$(pids_for_pattern_on_domain "$node_pattern" "$domain")
    if [ -n "$existing" ]; then
        warn "$label teleop: $node_pattern already running on domain $domain (PID(s): $existing)."
        warn "$label teleop: NOT starting a second instance (would double-publish on the same topic). Skipped."
        return 1
    fi

    # Rule (2) again, and not redundantly: main() refused before ANYTHING was
    # started, but between that check and this line the health check, the local
    # mapping stack and RViz have run - tens of seconds in which somebody else's
    # front-end can appear on this domain. There the answer is to abort the run;
    # here RViz and the mapping stack are already up and are harmless, so only
    # the teleop is skipped. Both messages name the PID.
    rivals=$(pids_for_pattern_on_domain "$rival_pattern" "$domain")
    if [ -n "$rivals" ]; then
        warn "$label teleop: a $(human_name_for_pattern "$rival_pattern") appeared on domain"
        warn "$label teleop: $domain since this run started (PID(s): $rivals)."
        warn "$label teleop: NOT starting - both publish the same cmd_vel and the same"
        warn "$label teleop: /teleop/direct_steer, and neither dead-man covers the other's"
        warn "$label teleop: traffic. Skipped. Nothing was killed."
        return 1
    fi

    # Gate AFTER the duplicate check on purpose: if no teleop is going to be
    # started, no steering command is going to be issued either, and asking for
    # movement approval for something that will not happen only trains people to
    # type MOVE reflexively. Gate BEFORE the branch below, so that neither the
    # browser UI nor the terminal fallback (where the command is only printed for
    # the user to run by hand) can reach a publish tick without it - a hand-run
    # moves the wheels just the same, and web_teleop_node inherits the very same
    # tick from KeyboardTeleopNode.
    if [ "$mode" = "real" ]; then
        steer_safety_gate || return 1
    fi

    case "$INPUT" in
        web) start_teleop_web "$prelude" "$domain" "$label" ;;
        *)   start_teleop_keyboard "$prelude" "$domain" "$label" "$mode" ;;
    esac
}

# --- keyboard front-end (needs a real terminal) -------------------------------

start_teleop_keyboard() { # prelude domain label mode
    local prelude="$1" domain="$2" label="$3" mode="$4"
    # laptop_teleop.launch.py passes config/keyboard_teleop.yaml as its first
    # parameters entry, and that file is keyed on `/**`. That is what supplies a, b
    # and wheel_radius, which keyboard_teleop_node declares WITHOUT defaults
    # (geometry single source of truth) - without that file the node raises at
    # startup instead of running.
    # GRIPPERX_DESK_QUIT_FLAG: exported into the SAME subshell as the launch, so
    # keyboard_teleop_node inherits it (ros2 launch passes its own environment
    # to the nodes it spawns) and _signal_desk_quit() there can find it. See the
    # "Teardown from the teleop's own Q" paragraph in the header.
    local cmd="$prelude export GRIPPERX_DESK_QUIT_FLAG='$QUIT_FLAG'; exec ros2 launch gripperx_teleop laptop_teleop.launch.py"
    if [ -z "$TERM_CMD" ]; then
        warn "$label teleop: no terminal emulator found (checked gnome-terminal, x-terminal-emulator)."
        warn "$label teleop: run this manually in your own terminal window instead:"
        warn "    bash -c \"$cmd\""
        warn "$label teleop: or use --input web, which needs no terminal at all."
        return 1
    fi

    # The trailing "Press Enter to close" prompt is for the ordinary case: the
    # node exited (crash, SIGTERM from teardown, plain window close) and the
    # operator should get a moment to read $cmd's own output before the window
    # vanishes. A Q/Ctrl+C quit in THIS window is different - the operator is
    # already looking at it, and it is what is DRIVING the desk-wide teardown
    # that is about to run (see the wait loop in main, below); the flag file
    # is the one thing on this laptop that knows which case just happened.
    # No PID to track yet (the node starts inside the terminal emulator): mark
    # the launch pending, so a teardown during the poll below still claims it.
    TELEOP_KBD_BASELINE=$(pids_for_pattern_on_domain "keyboard_teleop_node" "$domain"; \
                          pids_for_pattern_on_domain "ros2 launch gripperx_teleop laptop_teleop.launch.py" "$domain")
    TELEOP_KBD_BASELINE=$(tr '\n' ' ' <<<"$TELEOP_KBD_BASELINE")
    TELEOP_KBD_PENDING_LABEL="$label"
    TELEOP_KBD_PENDING_SINCE=$SECONDS
    TELEOP_KBD_PENDING_DOMAIN="$domain"
    "$TERM_CMD" --title="GripperX teleop ($mode, domain $domain)" -- bash -c \
        "$cmd; echo; echo '[gripperx_desk] teleop ended.'; \
         if [ -f '$QUIT_FLAG' ]; then \
             echo '[gripperx_desk] quit requested from this window - the whole desk session is stopping, closing.'; \
         else \
             read -r -p 'Press Enter to close this window... '; \
         fi" \
        >/dev/null 2>&1 &

    local waited=0 node_pid=""
    while [ "$waited" -lt "$TELEOP_KBD_WAIT_SEC" ]; do
        if resolve_kbd_teleop; then
            node_pid="$TELEOP_NODE_PID"
            break
        fi
        sleep 1
        waited=$((waited + 1))
    done
    if [ -z "$node_pid" ]; then
        # Stays PENDING on purpose: if the node does come up late, teardown
        # still claims it (bounded by the same window).
        warn "$label teleop: no keyboard_teleop_node appeared within ${TELEOP_KBD_WAIT_SEC}s - check the terminal window for errors."
        return 1
    fi
    ok "$label teleop started in its own terminal window (node PID $node_pid, domain $domain)."
}

# --- web front-end (needs no terminal) ----------------------------------------

# Who, if anyone, is listening on TCP port $1. Prints a human description and
# returns 0 if the port is taken. Two independent methods, because neither is
# reliable alone: ss names the PID but only for processes this user owns, and a
# connect test sees any listener but can name none.
web_port_in_use() { # port
    local port="$1" pids="" pid out=""
    if command -v ss >/dev/null 2>&1; then
        pids=$(ss -ltnp 2>/dev/null \
               | awk -v p=":$port" '$4 ~ (p "$") {print}' \
               | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u | tr '\n' ' ')
    fi
    if [ -n "${pids// /}" ]; then
        for pid in $pids; do
            out="$out PID $pid ($(cat "/proc/$pid/comm" 2>/dev/null || echo '?'), ROS_DOMAIN_ID=$(domain_of_pid "$pid" | head -n1))"
        done
        echo "${out# }"
        return 0
    fi
    if timeout 2 bash -c "exec 3<>/dev/tcp/127.0.0.1/$port" 2>/dev/null; then
        echo "an unnamed process (a connection to 127.0.0.1:$port succeeded)"
        return 0
    fi
    return 1
}

start_teleop_web() { # prelude domain label
    local prelude="$1" domain="$2" label="$3"

    # BEFORE launching, not after: a second web_teleop_node on a taken port dies
    # inside bind() with a traceback in a log file nobody is watching, and the only
    # symptom this script would otherwise show is "no node appeared within 20 s".
    local holder
    if holder=$(web_port_in_use "$WEB_PORT"); then
        warn "$label teleop (web): TCP port $WEB_PORT is already taken by $holder."
        warn "$label teleop (web): web_teleop_node would fail in bind(). Use --web-port <n>,"
        warn "$label teleop (web): or stop that process first. NOT started, nothing killed."
        return 1
    fi

    local logfile="$LOG_DIR/web_teleop_${domain}.log"
    # open_browser is wired through as a launch argument rather than opened from
    # here, so that the browser opens the URL the NODE actually bound, not one
    # this script guessed.
    local args="web_host:=$WEB_HOST web_port:=$WEB_PORT open_browser:=$WEB_OPEN_BROWSER"
    # GRIPPERX_DESK_QUIT_FLAG: same mechanism as the keyboard front-end (see
    # the header) - web_teleop_node inherits it from ros2 launch and writes it
    # only on a deliberate Shift+Q/quit-button operator quit.
    bash -c "$prelude export GRIPPERX_DESK_QUIT_FLAG='$QUIT_FLAG'; exec ros2 launch gripperx_teleop web_teleop.launch.py $args" \
        >"$logfile" 2>&1 &
    local lpid=$!
    track "$label teleop launch (web)" "$lpid" "gripperx_teleop web_teleop.launch.py" "" "" launch
    sleep 2
    if ! kill -0 "$lpid" 2>/dev/null; then
        warn "$label teleop (web): launch exited immediately - see $logfile"
        return 1
    fi

    local waited=0 node_pid=""
    while [ "$waited" -lt 20 ]; do
        node_pid=$(pids_for_pattern_on_domain "web_teleop_node" "$domain" | head -n1)
        [ -n "$node_pid" ] && break
        sleep 1
        waited=$((waited + 1))
    done
    if [ -z "$node_pid" ]; then
        warn "$label teleop (web): no web_teleop_node appeared within 20 s - see $logfile"
        warn "$label teleop (web): the launch process is tracked and will be torn down with the rest."
        return 1
    fi
    track "$label teleop node (web)" "$node_pid" "web_teleop_node" "" "" node
    TELEOP_NODE_PID="$node_pid"

    # A live process is not a served page. Wait for the socket, so the URL below
    # is one that works when it is printed rather than one that will work soon.
    local served=0
    for waited in 1 2 3 4 5 6 7 8 9 10; do
        if web_port_in_use "$WEB_PORT" >/dev/null; then served=1; break; fi
        sleep 1
    done

    echo
    log "======================================================================"
    if [ "$served" = 1 ]; then
        ok  "  BROWSER TELEOP UI:   http://127.0.0.1:$WEB_PORT/"
    else
        warn "  BROWSER TELEOP UI:   http://127.0.0.1:$WEB_PORT/  (not answering yet - see $logfile)"
    fi
    log  "  node PID $node_pid, launch PID $lpid, domain $domain, log $logfile"
    if [ "$WEB_HOST" != "127.0.0.1" ]; then
        warn "  BOUND TO $WEB_HOST - reachable from the network, WITH NO PASSWORD."
        warn "  Anyone who can reach this port can drive the robot."
    else
        log  "  Bound to 127.0.0.1 only - not reachable from the network."
    fi
    log "======================================================================"
    echo
}

# --- leader arm node (real mode) -------------------------------------------------

# "PID (cmdline)" of every process of THIS user with $1 open, one per line. Other
# users' /proc/<pid>/fd is unreadable, so a foreign holder stays invisible here.
# One find over all fds instead of a readlink per fd: that loop took ~15 s on a
# laptop with ~2650 open fds.
port_holders() { # device
    local dev="$1" real pid
    real=$(readlink -f -- "$dev" 2>/dev/null) || return 0
    for pid in $(find /proc/[0-9]*/fd -maxdepth 1 \( -lname "$dev" -o -lname "$real" \) \
                     2>/dev/null | cut -d/ -f3 | sort -u -n); do
        echo "$pid $(tr '\0' ' ' 2>/dev/null < "/proc/$pid/cmdline")"
    done
}

start_leader() { # prelude domain
    local prelude="$1" domain="$2"
    case "$LEADER" in
        off|0|no|skip)
            log "leader arm: disabled (GRIPPERX_DESK_LEADER=$LEADER / --no-leader) - J in the teleop will not work."
            return 0 ;;
    esac
    if [ ! -e "$LEADER_DEV" ]; then
        warn "leader arm: $LEADER_DEV not found - the leader arm is not plugged in (or the"
        warn "  udev rule gripperx_leader/udev/99-gripperx-leader.rules is not installed)."
        warn "  leader_arm_node NOT started: J (leader mode) in the teleop will not work."
        warn "  Everything else starts as usual."
        return 0
    fi
    # Already there: on our domain by name, or holding the port at all. Either way
    # a second node would fight it for the bus. Reused, never tracked, never stopped.
    local existing holders pid line others=() uncal=()
    existing=$(pids_for_pattern_on_domain "leader_arm_node" "$domain" | tr '\n' ' ')
    if [ -n "${existing// /}" ]; then
        log "leader arm: leader_arm_node already running on domain $domain (PID(s): ${existing% })"
        log "  - reused, NOT started again, and NOT stopped at teardown (not ours):"
        # The match is by command line; shown so a false one (an editor, less) is visible.
        for pid in $existing; do
            log "    PID $pid: $(tr '\0' ' ' 2>/dev/null < "/proc/$pid/cmdline")"
        done
        for pid in $existing; do
            pid_cmdline_matches "$pid" "calibration_file:=" || uncal+=("$pid")
        done
        if [ "${#uncal[@]}" -gt 0 ]; then
            warn "leader arm: the reused node (PID(s) ${uncal[*]}) was started WITHOUT calibration_file:="
            warn "  - leader angles unknown: J alignment is refused and the virtual wall is"
            warn "  unavailable with it. Stop it (Ctrl-C in its terminal) and rerun this script."
        fi
        return 0
    fi
    # Only when no node was found by name: the fd scan is the slower check.
    holders=$(port_holders "$LEADER_DEV")
    if [ -n "$holders" ]; then
        while IFS= read -r line; do
            pid="${line%% *}"
            if pid_cmdline_matches "$pid" "leader_arm_node"; then
                warn "leader arm: leader_arm_node PID $pid holds $LEADER_DEV but runs on domain"
                warn "  '$(domain_of_pid "$pid")', not $domain - J from this teleop will NOT reach it."
                warn "  Reused anyway (a second node cannot open the port), NOT stopped at teardown."
                pid_cmdline_matches "$pid" "calibration_file:=" || \
                    warn "  It was also started WITHOUT calibration_file:= - J and the wall will not work with it."
                return 0
            fi
            others+=("$line")
        done <<< "$holders"
        warn "leader arm: $LEADER_DEV is held by another process - leader_arm_node NOT started:"
        for line in "${others[@]}"; do warn "    PID $line"; done
        warn "  (leader_eeprom / record_limits / a serial terminal?) Nothing was killed."
        warn "  J (leader mode) in the teleop will not work until it is free and the node runs."
        return 0
    fi
    local exe="$WS/install/gripperx_leader/lib/gripperx_leader/leader_arm_node"
    if [ ! -x "$exe" ]; then
        warn "leader arm: $exe is missing - gripperx_leader is not built in this workspace."
        warn "  leader_arm_node NOT started: J (leader mode) in the teleop will not work."
        return 0
    fi
    if [ ! -f "$LEADER_CALIBRATION" ]; then
        warn "leader arm: calibration file missing: $LEADER_CALIBRATION"
        warn "  leader_arm_node NOT started - an uncalibrated leader has no angles, no wall,"
        warn "  and refuses every J. Rebuild gripperx_leader (the file is installed from"
        warn "  src/gripperx_leader/config/leader_calibration.yaml)."
        return 0
    fi

    local logfile="$LOG_DIR/leader_${domain}.log"
    # exec of the installed executable, not "ros2 run" - see the header (SIGINT).
    bash -c "$prelude exec '$exe' --ros-args -p port:=$LEADER_DEV -p 'servo_ids:=$LEADER_SERVO_IDS' -p 'calibration_file:=$LEADER_CALIBRATION'" \
        >"$logfile" 2>&1 &
    local lpid=$! waited=0
    # At launch - see start_local_mapping. The startup poll below can take 15 s.
    track "leader arm node" "$lpid" "leader_arm_node" "$LEADER_STOP_WAIT_SEC" \
        "it was KILLED, so its shutdown (torque off on every leader servo) did not run - a servo held by the virtual wall may still be torqued. Unplug the leader's power/USB or restart the node and stop it with Ctrl-C." command
    # Ready = the node's own "DISARMED ... torque off" line; it dies on a failed
    # startup check (bus, ping, calibration). 15 s covers the bus scan.
    while [ "$waited" -lt 30 ]; do
        kill -0 "$lpid" 2>/dev/null || break
        grep -q 'leader streaming DISARMED' "$logfile" 2>/dev/null && break
        sleep 0.5
        waited=$((waited + 1))
    done
    if ! kill -0 "$lpid" 2>/dev/null; then
        warn "leader arm: leader_arm_node exited during startup - see $logfile:"
        tail -n 5 "$logfile" 2>/dev/null | sed 's/^/    /' >&2
        warn "  J (leader mode) in the teleop will not work."
        return 0
    fi
    if grep -q 'leader streaming DISARMED' "$logfile" 2>/dev/null; then
        ok "leader arm: leader_arm_node started (PID $lpid, domain $domain, $LEADER_DEV, log $logfile)."
    else
        warn "leader arm: leader_arm_node (PID $lpid) is running but has not reported ready"
        warn "  within 15 s - see $logfile. Tracked; teardown stops it."
    fi
}

# --- groups -------------------------------------------------------------------

start_real() {
    log "== real: local mapping + RViz + teleop, ROS_DOMAIN_ID=$REAL_DOMAIN =="
    # Mapping first: RViz opens on Fixed Frame "map", so /map and the TF chain
    # should already be on their way when it starts. Neither of these two can
    # move anything - the gate sits in front of the teleop, which can.
    start_local_mapping
    # Only arms itself if the line above actually started a local stack.
    start_map_collision_watch
    start_rviz "$REAL_PRELUDE" "$RVIZ_CONFIG_REAL" "$REAL_DOMAIN" "real"
    start_teleop "$REAL_PRELUDE" "$REAL_DOMAIN" "real" "real"
    # After the teleop, so its J and Space are there by the time the leader can
    # act on them. Passive until an align request: moves nothing on start.
    start_leader "$REAL_PRELUDE" "$REAL_DOMAIN"
    log "real: local mapping state: $LOCAL_MAP_STATE"
}

TWIN_LAUNCHED=0
TWIN_BASELINE_PIDS=""

start_twin() {
    log "== twin: Gazebo + SLAM + Nav2 + RViz + teleop, ROS_DOMAIN_ID=$TWIN_DOMAIN =="
    # Per NFR-10 / Gazebo Harmonic - see the header. Deliberately absent:
    # sim_steer_bridge / swerve_cmd_node / joint_command_bridge (not started on
    # either side), ros2_control_node (in the sim the controller_manager lives
    # inside the gz_ros2_control plugin, i.e. inside the gz sim process, so no such
    # process exists to collide with) and gzserver (Gazebo Classic). Present:
    # clock_ready_gate and teleop_mux_node, which the twin does start, and
    # scan_range_filter, which simulate_robot.launch.py starts in the twin as well
    # (/scan_raw -> /scan), where a second instance would republish the same topic.
    # All of these survive a plain Ctrl-C.
    # Nav2 (sim_navigation.launch.py -> gripperx_planning/navigation.launch.py):
    # the five servers and their lifecycle_manager, the only Nav2 processes the
    # twin's localization:=slam mode starts. Also listed, although this script's
    # launch does not start them, because another session's copy on domain 220
    # would still fight ours for map->odom / odom->base_footprint or answer our
    # goals: sim_mapping.launch.py (the pre-Nav2 twin launch), the AMCL
    # alternative (nav2_map_server, nav2_amcl) and the odom_source:=ekf chain
    # (ekf_node, laser_scan_matcher, localization_input_node).
    local patterns=("sim_mapping.launch.py" "sim_navigation.launch.py" \
                     "gz sim" "async_slam_toolbox_node" \
                     "ground_truth_odom_bridge" "clock_ready_gate" \
                     "robot_state_publisher" "teleop_mux_node" "spawn_robot" \
                     "ros_gz_bridge" "parameter_bridge" "scan_range_filter" \
                     "planner_server" "controller_server" "behavior_server" \
                     "bt_navigator" "velocity_smoother" "lifecycle_manager" \
                     "nav2_map_server" "nav2_amcl" "ekf_node" \
                     "laser_scan_matcher" "localization_input_node" \
                     "rviz2_mapping" "rviz2_navigation")

    # Pre-flight, read-only. spawn_robot.launch.py (via sim_navigation ->
    # simulation -> simulate_robot) starts gripperx_gazebo's clock_ready_gate. If
    # that executable is missing, launch does not degrade - it aborts the ENTIRE
    # launch description and tears Gazebo down again, and the only message is
    # "executable 'clock_ready_gate' not found on the libexec directory", which
    # reads like a code bug rather than a stale install tree - typically
    # install/gripperx_gazebo/lib/gripperx_gazebo missing the executable while the
    # source src/gripperx_gazebo/scripts/clock_ready_gate is present. Warn, name the
    # fix, and continue - this script never builds anything and never writes to the
    # shared workspace.
    local gate_exe="$WS/install/gripperx_gazebo/lib/gripperx_gazebo/clock_ready_gate"
    if [ ! -x "$gate_exe" ]; then
        warn "twin: clock_ready_gate is missing from the install tree:"
        warn "  $gate_exe"
        warn "twin: sim_navigation.launch.py will ABORT on it and take Gazebo down with it."
        warn "twin: the laptop install of gripperx_gazebo is stale. Rebuild it yourself:"
        warn "    (cd '$WS' && colcon build --packages-select gripperx_gazebo)"
        warn "twin: continuing so you see the launch's own error; expect Gazebo not to stay up."
    fi
    # Same idea for Nav2: this laptop has no system Nav2, it comes from
    # .rosdeps_local (gitignored; in a git worktree a symlink to the primary
    # checkout's copy). Without it the Nav2 executables are "not found" and the
    # launch aborts with Gazebo inside it.
    if [ ! -d "$WS/.rosdeps_local/opt/ros/jazzy" ] && [ ! -d /opt/ros/jazzy/share/nav2_bt_navigator ]; then
        warn "twin: Nav2 not found - neither $WS/.rosdeps_local nor a system install."
        warn "twin: sim_navigation.launch.py will abort on it. Fetch it with"
        warn "    $WS/scripts/fetch_missing_ros_debs.sh"
        warn "  or, in a git worktree, symlink the primary checkout's .rosdeps_local."
    fi
    local p pid collisions=()
    TWIN_BASELINE_PIDS=""
    for p in "${patterns[@]}"; do
        for pid in $(pids_for_pattern_on_domain "$p" "$TWIN_DOMAIN"); do
            TWIN_BASELINE_PIDS="$TWIN_BASELINE_PIDS $pid"
            collisions+=("$p(PID $pid)")
        done
    done
    if [ "${#collisions[@]}" -gt 0 ]; then
        warn "twin: refusing to launch Gazebo/sim_navigation - already running on domain $TWIN_DOMAIN:"
        warn "  ${collisions[*]}"
        warn "twin: this looks like a leftover/other session's sim stack. NOT touching it, NOT starting a duplicate."
        warn "twin: Gazebo skipped. Standalone RViz + teleop for the twin will still be attempted."
        TWIN_LAUNCHED=0
    else
        local logfile="$LOG_DIR/gazebo_twin.log"
        # localization:=slam + odom_source:=ground_truth are sim_navigation's
        # defaults, spelled out because they are the choice: live slam_toolbox
        # is what the robot's gripperx-mapping.service runs (no saved map,
        # unlike AMCL), and ground-truth odometry is what the pre-Nav2 twin
        # (sim_mapping) used. The robot's odometry is the EKF chain instead
        # (odom_source:=ekf here); that switch is an open decision, not made here.
        # initial_mode:=keyboard overrides sim_navigation's autonomous default
        # so teleop_mux starts where the robot's does (teleop_mux.yaml).
        bash -c "$TWIN_NAV_PRELUDE exec ros2 launch gripperx_gazebo sim_navigation.launch.py headless:=false use_rviz:=false localization:=slam odom_source:=ground_truth initial_mode:=keyboard" \
            >"$logfile" 2>&1 &
        local gzpid=$!
        track "twin gazebo/sim_navigation" "$gzpid" "gripperx_gazebo"   # at launch
        sleep 2
        if kill -0 "$gzpid" 2>/dev/null; then
            TWIN_LAUNCHED=1
            ok "twin Gazebo launch started (PID $gzpid, log: $logfile) - GUI window may take a few seconds."
        else
            warn "twin Gazebo launch exited immediately - see $logfile"
            TWIN_LAUNCHED=0
        fi
    fi
    start_rviz "$TWIN_PRELUDE" "$RVIZ_CONFIG_TWIN" "$TWIN_DOMAIN" "twin"
    start_teleop "$TWIN_PRELUDE" "$TWIN_DOMAIN" "twin" "twin"
}

# --- main ----------------------------------------------------------------

# Sourced instead of executed (the offline test harness, scripts/test/
# test_gripperx_desk.sh): every function is defined, nothing below is run.
# "return" succeeds only in a sourced file or a function; comparing BASH_SOURCE
# with $0 instead misfired for "bash < file" and bash -c "$(cat file)".
if (return 0 2>/dev/null); then
    trap - INT TERM PIPE EXIT
    return 0
fi

MODE="real"
case "${1:-}" in
    -h|--help) usage; exit 0 ;;
    real|twin) MODE="$1"; shift ;;
    "") ;;
    -*) ;;   # no mode given, only flags - keep the default
    *) echo "Unknown mode: $1" >&2; usage; exit 1 ;;
esac

while [ "$#" -gt 0 ]; do
    case "$1" in
        -h|--help)      usage; exit 0 ;;
        --no-steer-gate) STEER_GATE="off" ;;
        --no-local-map)  LOCAL_MAP="off" ;;
        --force-local-map) LOCAL_MAP="force" ;;
        --no-leader)     LEADER="off" ;;
        --input)        shift; INPUT="${1:-}" ;;
        --input=*)      INPUT="${1#--input=}" ;;
        --web)          INPUT="web" ;;
        --keyboard)     INPUT="keyboard" ;;
        --open-browser) WEB_OPEN_BROWSER="true" ;;
        --web-port)     shift; WEB_PORT="${1:-}" ;;
        --web-port=*)   WEB_PORT="${1#--web-port=}" ;;
        # The opt-in for leaving loopback. Flag only, never an env var - see the
        # WEB_HOST definition and the header.
        --web-expose)   WEB_HOST="0.0.0.0" ;;
        *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
    esac
    shift
done

case "$INPUT" in
    keyboard|web) ;;
    *) echo "Unknown input device: '$INPUT' (expected 'keyboard' or 'web')" >&2; usage; exit 1 ;;
esac
case "$WEB_PORT" in
    ''|*[!0-9]*) echo "Invalid --web-port: '$WEB_PORT' (expected a number)" >&2; exit 1 ;;
esac

log "Mode: $MODE, input device: $INPUT"
# Only the keyboard front-end needs a terminal emulator; the browser UI is
# backgrounded, which is the whole point of it.
if [ "$INPUT" = keyboard ] && [ -z "$TERM_CMD" ]; then
    warn "No terminal emulator found - teleop will only print manual instructions."
    warn "  (--input web needs none: it is served over HTTP, not read from a tty.)"
fi
if [ "$INPUT" = web ] && [ "$WEB_HOST" != "127.0.0.1" ]; then
    warn "--web-expose: the UI will bind $WEB_HOST:$WEB_PORT, NOT loopback."
    warn "  This page has NO PASSWORD and control is first-come. Anyone who can reach"
    warn "  that port can drive this robot. Only do this on a network you control."
fi
case "$MODE" in
    real)
        refuse_if_other_teleop "$TWIN_DOMAIN" "twin" || exit 1
        refuse_if_conflicting_input "$REAL_DOMAIN" "$INPUT" || exit 1
        other_track_check "$REAL_DOMAIN"
        pi_health_check
        start_real
        ;;
    twin)
        refuse_if_other_teleop "$REAL_DOMAIN" "real" || exit 1
        refuse_if_conflicting_input "$TWIN_DOMAIN" "$INPUT" || exit 1
        other_track_check "$TWIN_DOMAIN"
        start_twin
        ;;
esac

log "Setup done. Press Ctrl-C or Enter here, or Q in the teleop (browser tab or"
log "  keyboard-teleop terminal window), to stop everything this script started."
printf '> '

# Was a blocking "read -r -p '> ' _" until the teleop's own Q gained the power
# to end the whole session too (see "Teardown from the teleop's own Q" in the
# header) - a single blocking read cannot also notice a flag file appearing.
# -t 1 polls instead, at the cost of the prompt only being drawn once (a
# redrawn "> " every second would be worse than losing it).
#   * a typed line (Enter, or anything else + Enter) -> stop, exactly as before
#   * EOF/closed stdin (rc=1, e.g. this script's own stdin is not a tty) ->
#     stop, exactly as the old "|| true" fallthrough did
#   * timeout (bash gives read -t an exit status > 128) -> keep polling
TELEOP_DEATH_WARNED=0
while :; do
    # $? MUST be captured on the line right after read - "if read...; then...fi"
    # with a false condition and no else resets $? to 0 by itself (bash's own
    # rule for the if compound), which silently threw away the timeout code and
    # broke out on the FIRST tick every time (caught by the offline harness,
    # not by eye). rc=0 (a line was typed) and rc=1 (EOF/closed stdin) both
    # stop, exactly like the old blocking "read ... || true" did for both
    # cases; only a timeout (rc>128) falls through to the checks below.
    IFS= read -r -t 1 _line
    rc=$?
    if [ "$rc" -le 128 ]; then
        break
    fi

    if [ -n "$QUIT_FLAG" ] && [ -f "$QUIT_FLAG" ]; then
        echo
        log "teleop quit requested - stopping everything this script started."
        break
    fi

    # The tracked teleop process died WITHOUT the flag: crashed, or killed by
    # something other than its own Q/quit button. That is not the same signal
    # as a deliberate quit - warn once and keep RViz/the local map running
    # rather than guess. teleop_process_alive() itself returns "ok" (no
    # warning) when no teleop is tracked at all, e.g. the steer gate was
    # declined and nothing ever started.
    if [ "$TELEOP_DEATH_WARNED" = 0 ] && ! teleop_process_alive; then
        warn "the teleop process is no longer running, and no quit was requested -"
        warn "  RViz/the local mapping stack etc. are left running. Press Enter here to"
        warn "  stop them too, or start teleop again yourself."
        TELEOP_DEATH_WARNED=1
    fi
done
teardown
