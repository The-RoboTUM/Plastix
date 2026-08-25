#!/bin/bash
# GripperX Desktop launcher
#
# Modes (first argument, default "real"):
#   real   RViz + teleop against the REAL robot, ROS_DOMAIN_ID=20. Also runs a
#          strictly read-only health check of the Pi first.
#   twin   Digital twin: Gazebo (sim_mapping.launch.py, GUI) + standalone
#          RViz + teleop, ROS_DOMAIN_ID=220. Mirrors the "Current
#          3-terminal set" in Software/ros2/src/gripperx_gazebo/README.md.
#   -h / --help   print this text.
#
# Input device (--input keyboard|web, default keyboard, both modes): WHICH
# front-end drives, never how many. keyboard = keyboard_teleop_node in its own
# terminal window, the long-standing behaviour and still the default. web =
# web_teleop_node, the browser UI (gripperx_teleop/docs/TELEOP_WEB_UI.md), run
# backgrounded with its log in $LOG_DIR. See "Browser teleop UI" below.
#
# real and twin are mutually exclusive - there is deliberately NO mode that
# runs both at once, and the script refuses to start one while the other is
# being teleoperated (see "Mutual exclusion" below).
#
# Domain convention (SR-8, PlastiX-wide): real GripperX robots 20-29, their
# digital twin the same id +200. GripperX-1 = 20 (real) / 220 (twin).
#
# Environment separation: nothing is exported in THIS script's own shell.
# Every RViz/Gazebo/teleop process is spawned as its own "bash -c '<prelude>;
# exec <cmd>'" subshell that sources its OWN domain's environment (mirroring
# Software/ros2/scripts/sim_env.sh's guard against the unrelated ~/ros2_ws
# on ~/.bashrc, which would otherwise leak a stale AMENT_PREFIX_PATH /
# ROS_DOMAIN_ID into every new terminal). Because nothing is exported here,
# no spawned process can inherit a stale or foreign domain from this script.
#
# Collision guard: before starting anything that isn't harmless to duplicate
# (teleop node, the twin's Gazebo/control node group), the script snapshots
# which of ITS OWN target processes already exist on that exact domain
# (matched via /proc/<pid>/environ, not just process name, so a same-named
# real-domain and twin-domain process are never confused). If a match is
# already running, that step is SKIPPED with a clear message instead of
# starting a duplicate (double-publish on /teleop/keyboard/cmd_vel, or a
# second ros2_control/Gazebo instance fighting the existing one, are worse
# than not starting). Only PIDs that appear AFTER this script's own launch
# action are ever recorded as "ours"; teardown only ever touches those exact
# PIDs, each re-verified against /proc/<pid>/cmdline immediately before the
# kill (guards against a PID being reused by an unrelated process between
# tracking and teardown). Nothing pre-existing is ever touched.
#
# Mutual exclusion, TWO rules, both about the same thing - never two live
# command sources, and never an ambiguous one:
#   (1) real <-> twin, ACROSS domains: the script refuses to start at all if a
#       teleop node is already running on the OTHER domain - twin refuses while
#       one runs on 20, real refuses while one runs on 220. Rationale: it must
#       never be ambiguous which window drives the REAL robot. Two teleop
#       windows look identical on screen and differ only in an invisible
#       ROS_DOMAIN_ID; a keypress meant for the simulation would then move real
#       hardware (SR-1/SR-8).
#   (2) keyboard <-> web, WITHIN one domain: a running browser UI refuses a
#       keyboard start and a running keyboard teleop refuses a web start, both
#       directions, naming the PID. This is the harder failure of the two.
#       web_teleop_node SUBCLASSES KeyboardTeleopNode and inherits _publish
#       unchanged, so the two front-ends publish the SAME /teleop/keyboard/
#       cmd_vel and the same /teleop/direct_steer. The mux forwards whichever
#       arrived last, and - the part that makes it dangerous rather than merely
#       untidy - NEITHER operator's dead-man covers the other's traffic:
#       releasing every key on the page does not stop a robot the terminal is
#       driving, and neither does its emergency stop (SR-2/SR-3, and
#       gripperx_teleop/docs/TELEOP_WEB_UI.md says the same in its own words). The node itself
#       only WARNS about a rival (red banner, ERROR log, deliberately no kill);
#       this launcher refuses, which is the cheaper place to catch it.
# Both rules match by /proc/<pid>/environ, not by process name alone, so a
# same-named process on another domain is never mistaken for one on ours - and
# rule (2) looks for the OTHER node name, not just its own.
#
# TTY: keyboard_teleop_node reads the terminal in raw mode (tty/termios), so
# it needs a real terminal, not a backgrounded pipe. Each keyboard teleop
# instance is therefore opened in its own terminal-emulator window
# (gnome-terminal, falling back to x-terminal-emulator); if neither exists, the
# script prints the exact command to run manually instead of starting it
# headless. web_teleop_node needs NO tty - its input arrives over HTTP - which
# is precisely why --input web skips the whole terminal-emulator dance and is
# backgrounded like RViz and the mapping stack.
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
#     points at --web-port. Observed: an unrelated session's UI on
#     127.0.0.1:8080 while this was being written.
#   * HANDOVER (2026-08-25) records the UI as verified on the twin (220) and on
#     a desk rig (42, no motors) and NEVER RUN AGAINST THE REAL ROBOT, with the
#     /joint_states-vs-commanded sign convention still unmeasured. "real" plus
#     "--input web" is therefore that first contact, and the launcher says so
#     out loud rather than letting it happen quietly.
#
# RViz config for "real": SUPERSEDED 2026-08-18. The Pi now runs mapping as a
# systemd service (gripperx-mapping: rf2o_laser_odometry + async slam_toolbox),
# so /map, /odom and the full TF chain map->odom->base_footprint->base_link DO
# exist on domain 20. The old default gripperx_description/rviz/display.rviz
# (Fixed Frame "base_link", no LaserScan, no Map) could therefore never show
# the robot moving through the world - it pinned the view to the robot itself.
# New default: gripperx_localization/rviz/localization.rviz, Fixed Frame "map",
# with Map(/map) + LaserScan(/scan) + RobotModel(/robot_description) + TF.
# Chosen over ~/.rviz2/gripperx_mapping.rviz because that config has NO
# RobotModel and no TF display at all (only Map/LaserScan/costmaps//plan), so
# the one thing actually asked for - a robot model that moves - would be
# missing. Price of this choice: localization.rviz carries displays that stay
# empty without Nav2/robot_localization/a camera (GlobalPlan /plan,
# FilteredOdom /odometry/filtered, DepthCloud+Camera+Image on /camera/*).
# Verified live on domain 20 (2026-08-18): RViz builds the map swatch
# (163x164 cells, matching /map) and logs no "Fixed Frame does not exist" and
# no missing transform for lidar_link/base_link. Override: RVIZ_CONFIG_REAL.
# Note on stationary SLAM: slam_toolbox stamps map->odom with the last
# PROCESSED scan (minimum_travel_distance 0.3 m / heading 0.3 rad), so while
# the robot stands still that stamp freezes and ages. Harmless here - the
# transform keeps being published at 50 Hz and tf2 serves the single cached
# entry for any query time - and it advances again as soon as you drive.
#
# Teardown: gripperx_gazebo/README.md explicitly warns a plain Ctrl-C is not
# enough for the twin. See "Collision guard" above for how this script tracks
# and kills exactly what it started, nothing else, no ros2 daemon stop, no
# name-only pkill sweep. The README still names swerve_cmd_node and
# joint_command_bridge as the usual survivors - that is stale, see "NFR-10"
# below; the surviving set is now gz sim / clock_ready_gate / parameter_bridge /
# teleop_mux_node / robot_state_publisher / async_slam_toolbox_node.
#
# NFR-10 (hardware-accepted 2026-08-19): swerve_controller is the ACTIVE AND ONLY
# drive path. real_robot.launch.py no longer includes control.launch.py, and
# spawn_robot.launch.py spawns swerve_controller instead of the old
# steering/wheel controller pair, so NONE of swerve_cmd_node,
# joint_command_bridge or sim_steer_bridge is started any more - on either side.
# The files were REMOVED from the repo in the deletion round (eb05e25, "deletion
# round: retire the pre-NFR-10 controller node chain"); only the orphaned config
# gripperx_control/config/swerve_cmd.yaml is left. That does NOT retire the
# checks below: a stale install tree, an old bringup that outlived a deploy, or
# a hand-started control.launch.py from an older checkout can all still put such
# a process on the robot, and the point of a health check is to see what IS
# running rather than what the current source could start.
#   * health check: swerve_cmd_node/joint_command_bridge were listed as REQUIRED
#     nodes, so every healthy robot reported two missing nodes. They are now
#     checked with the opposite polarity - if either is running, that is a
#     THIRD publisher on /hw/joint_commands (the documented normal state is
#     exactly 2: GripperXInterface + its watchdog, D4/OP-18b/SR-10) and is
#     reported as a fault. teleop_mux_node and lidar_power_node were missing
#     from the required list and were added.
#   * swerve_controller itself is a controller_manager PLUGIN, not a process:
#     it cannot be pgrep-ed. Its spawner is one-shot and is long gone by the
#     time this check runs. Verifying it needs "ros2 control list_controllers"
#     on the Pi, which is deliberately NOT done here (DDS traffic during/after
#     bringup provokes controller_manager overruns).
#   * twin collision-guard/orphan lists: dropped sim_steer_bridge,
#     swerve_cmd_node, joint_command_bridge (never started any more) and
#     ros2_control_node (in the sim the controller_manager runs INSIDE the
#     gz_ros2_control plugin, i.e. inside the gz sim process - there is no
#     separate ros2_control_node to orphan); dropped gzserver (Gazebo Classic,
#     this stack is Harmonic). Added clock_ready_gate and teleop_mux_node,
#     which the twin does start and which do survive a plain Ctrl-C.
#
# Steering safety gate (real mode, MOVEMENT-RELEVANT - read this before
# removing it): merely STARTING the teleop commands the steering. keyboard_
# teleop_node comes up in manoeuvre state CORNERING (the startup state) and in
# that branch publishes angle*STEER_PATTERN on /teleop/direct_steer at
# publish_rate_hz (default 50.0) with angle initialised to 0.0;
# steer_servo_node applies /teleop/direct_steer as an OVERRIDE on top of
# /hw/joint_commands. So the wheels are driven to straight-ahead (0 deg)
# immediately, without any key press. On 2026-08-19 the robot was left with
# actuator power ON and the steering holding ~ +-33 deg from the last drive -
# starting the launcher would have swung all four wheels by that much,
# unrequested.
# This is exactly the class of action the user's standing rule covers: no
# movement of drive, steering servos or arm without explicit per-test approval
# (rule of 2026-07-06, reaffirmed after the incident). The gate therefore
# states what will happen, reads the CURRENT angles from /hw/steer_states
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
# place deliberately rather than papered over. Since a52379d (2026-08-24)
# gripperx-mapping.sh launches gripperx_localization/localization.launch.py
# (laser_scan_matcher + EKF + slam_toolbox with the 2026-08-21 tuning), not
# mapping.launch.py; the commit's own words are that mapping.launch.py's
# slam_toolbox.yaml carries the older, coarser parameters and that rf2o and the
# EKF both publish odom->base_footprint. So this laptop-side fallback is now a
# DIFFERENT odometry source with COARSER SLAM settings, and running it beside a
# mapping Pi is a TF fight, not merely a duplicate. That is exactly why the
# detection below had to be corrected to recognise the Pi's new node set, and why
# it now refuses to start on an unknown answer. Whether the laptop fallback
# should switch to localization.launch.py as well is a real question and NOT
# decided here - it is a repo file, another stack's tuning, and a user call.
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
# AND IT SKIPS ITSELF WHEN IT CANNOT TELL - added 2026-08-25 after the path was
# exercised for the first time. Both pieces of evidence can fail at once, and
# they did: on a loaded Pi all five ssh attempts timed out (evidence "unknown"),
# and the DDS fallback "ros2 node list" on domain 20 returned a graph with no
# robot in it at all - the laptop could not see the Pi's nodes, only a local one.
# Two failed observations are not a measurement that nothing is mapping, but the
# old code read them that way and went on to START a second rf2o + slam_toolbox
# ON THE REAL ROBOT'S DOMAIN - precisely the duplicate-publisher case the
# paragraph above exists to prevent, reached by guessing. The steering gate two
# paragraphs down already answers this class of question the other way round
# ("unknown is treated as worst case"), and this now does the same: no ssh
# answer means no local mapping stack, and the run says so. --force-local-map
# (or GRIPPERX_DESK_LOCAL_MAP=force) is the deliberate override for the case
# where the operator KNOWS the Pi is not mapping; like --no-steer-gate it
# asserts, it does not verify.
#
# gz-transport partition (twin): ROS_DOMAIN_ID isolates DDS, but gz-transport
# IGNORES it entirely - it namespaces by GZ_PARTITION, which defaults to
# "<hostname>:<username>" and is therefore IDENTICAL for every track on this
# laptop. A parallel Gazebo has already captured a spawn once because of this
# (SR-8 gap, recorded in the handover). sim_env.sh sets GZ_IP but no
# GZ_PARTITION and is a repo file that is NOT edited from here, so the twin
# prelude sets GZ_PARTITION itself, AFTER sourcing sim_env.sh. The name
# gripperx_desk_twin_220 cannot collide with the nav2 track (~/gripperx_ws_nav2,
# domain 220), because that track sources sim_env.sh unchanged and thus stays
# on the "<hostname>:<username>" default - any explicit non-empty name is
# disjoint from it. Override with GZ_PARTITION_TWIN.
#
# Domain ownership: 20 = real GripperX-1, 220 = its twin (SR-8 makes that
# mandatory, not conventional), 221 = the "second parallel twin session" the
# same convention reserves; gripperx_external hard-codes SIMULATION_DOMAIN_IDS
# = {220, 221} and treats every other domain as a real robot.
# CORRECTED 2026-08-25: this paragraph used to attribute 220 to a nav2 track in
# ~/gripperx_ws_nav2 and 221 to an octopus track in ~/gripperx_ws_octopus. Both
# worktrees were removed and both branches deleted on 2026-08-25, so naming an
# owner is now a guess. The check reports the DOMAIN and the PIDs and leaves the
# attribution to the reader. It stays read-only either way: it names PIDs and
# warns, it never acts.

set -u

# Repo copy of this script (Software/ros2/scripts/gripperx_desk.sh): WS is
# derived from the script's OWN location, same technique sim_env.sh in this
# same directory already uses ("$(dirname "${BASH_SOURCE[0]}")/.."), and for
# the same reason - a hardcoded "$HOME/gripperx_ws" would silently point at
# the shared main tree even when this file is run out of a git worktree
# (e.g. ~/gripperx_ws_desk), sourcing a foreign install/setup.bash. The
# original, unversioned ~/gripperx_desk.sh keeps the old hardcoded
# "$HOME/gripperx_ws/Software/ros2" on purpose - it only ever runs from the
# user's actual $HOME, never from a worktree copy.
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
SSH_HOST="gripperx"

REAL_DOMAIN=20
TWIN_DOMAIN=220
RMW_IMPL=rmw_fastrtps_cpp

# Domains that other parallel sessions may occupy - only ever READ, never used
# here. See "Domain ownership" in the header: these are the two ids the domain
# convention reserves for simulation (SR-8 / SIMULATION_DOMAIN_IDS), NOT the
# property of any named worktree. The two worktrees this list used to name were
# deleted on 2026-08-25.
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

# Which teleop front-end drives - "keyboard" (default, unchanged behaviour) or
# "web". Applies to real and twin alike; it selects the input device, it never
# starts a second one. --input <device> overrides.
INPUT="${GRIPPERX_DESK_INPUT:-keyboard}"

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

# Set by pi_health_check: active | inactive | unknown. "unknown" means the Pi
# could not be asked at all (ssh down) - then the DDS graph is the only evidence.
PI_MAPPING_EVIDENCE="unknown"

RVIZ_CONFIG_REAL="${RVIZ_CONFIG_REAL:-$WS/install/gripperx_localization/share/gripperx_localization/rviz/localization.rviz}"
RVIZ_CONFIG_TWIN="${RVIZ_CONFIG_TWIN:-$WS/install/gripperx_localization/share/gripperx_localization/rviz/localization.rviz}"

LOG_DIR="/tmp/gripperx_desk"
mkdir -p "$LOG_DIR"

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

TERM_CMD=""
if command -v gnome-terminal >/dev/null 2>&1; then
    TERM_CMD="gnome-terminal"
elif command -v x-terminal-emulator >/dev/null 2>&1; then
    TERM_CMD="x-terminal-emulator"
fi

declare -a TRACKED_PIDS=()
declare -a TRACKED_LABELS=()
declare -a TRACKED_PATTERNS=()   # substring expected in /proc/<pid>/cmdline at kill time

usage() {
    cat <<'EOF'
Usage: gripperx_desk.sh [real|twin|-h|--help] [flags]

  real   (default) Pi health check (read-only) + local mapping stack + RViz +
         teleop against the REAL robot, ROS_DOMAIN_ID=20.
  twin   Digital twin: Gazebo (GUI) + standalone RViz + teleop,
         ROS_DOMAIN_ID=220, in its own gz-transport partition.

Input device (both modes): --input keyboard (default) or --input web.
  keyboard  keyboard_teleop_node in its own terminal window (needs a tty).
  web       web_teleop_node, the browser UI, backgrounded, log in /tmp/
            gripperx_desk. Needs no terminal; the URL is printed.

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
  --input <keyboard|web>  teleop front-end (default keyboard)
  --web                   shorthand for --input web
  --open-browser          open the UI in a browser (web only, default off)
  --web-port <n>          TCP port for the UI (default 8080)
  --web-expose            bind the UI to 0.0.0.0 instead of 127.0.0.1.
                          UNAUTHENTICATED DRIVE INTERFACE: the page has no
                          password, and anyone who can reach the port can drive
                          this robot. Only on a network you control.
  --no-steer-gate   skip the MOVE confirmation (only if actuator power is OFF)
  --no-local-map    do not start rf2o+slam_toolbox locally in real mode
  --force-local-map start it even if the Pi could not be asked whether it is
                    already mapping (asserts, does not verify)

Env overrides: RVIZ_CONFIG_REAL=<path>, RVIZ_CONFIG_TWIN=<path>,
               GRIPPERX_DESK_STEER_GATE=off, GRIPPERX_DESK_LOCAL_MAP=off,
               GRIPPERX_DESK_INPUT=web, GRIPPERX_DESK_WEB_PORT=<n>,
               GZ_PARTITION_TWIN=<name>
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
    # the shell, not by tr, so a trailing 2>/dev/null came too late to suppress
    # it. Only showed up once this was called for every PID in /proc rather than
    # for pgrep hits, which are all our own.
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

track() { # label pid pattern
    TRACKED_LABELS+=("$1")
    TRACKED_PIDS+=("$2")
    TRACKED_PATTERNS+=("$3")
}

# THE ros2 CLI DAEMON IS NOT A NODE, AND MUST NOT BE REPORTED AS ONE.
# Every "ros2 node list" / "ros2 topic list" / "ros2 lifecycle get" auto-starts a
# per-domain helper - python3 -c "from ros2cli.daemon.daemonize import main;
# main()" --name ros2-daemon --ros-domain-id <d> - which then idles for two hours.
# It carries ROS_DOMAIN_ID in its environ, so to a /proc-based domain match it is
# indistinguishable from a real node, and THIS SCRIPT SPAWNS THEM ITSELF: the
# health check's topic list, the local-mapping DDS probe and the lifecycle calls
# all do. Found 2026-08-25: other_track_check() reported domains 220 and 221 as
# busy with named tracks while both were in fact empty - the hits were daemons,
# one of them left behind by this script's own previous run. A warning that fires
# when nothing is wrong is worse than no warning, because it teaches the reader
# to skip the one that matters.
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

TEARDOWN_DONE=0
teardown() {
    [ "$TEARDOWN_DONE" = 1 ] && return
    TEARDOWN_DONE=1
    if [ "${#TRACKED_PIDS[@]}" -eq 0 ] && [ "${TWIN_LAUNCHED:-0}" = 0 ]; then
        return   # nothing was ever started (e.g. -h/--help, or all steps skipped) - stay quiet
    fi
    echo
    log "Tearing down everything this script started..."
    local i pid label pattern
    for i in "${!TRACKED_PIDS[@]}"; do
        pid="${TRACKED_PIDS[$i]}"
        label="${TRACKED_LABELS[$i]}"
        pattern="${TRACKED_PATTERNS[$i]}"
        if ! kill -0 "$pid" 2>/dev/null; then
            log "  $label (PID $pid) already gone."
            continue
        fi
        if ! pid_cmdline_matches "$pid" "$pattern"; then
            warn "  $label (PID $pid) no longer matches its expected command " \
                 "(PID reused?) - NOT touching it."
            continue
        fi
        kill -INT "$pid" 2>/dev/null
        for _ in 1 2 3 4 5 6; do
            kill -0 "$pid" 2>/dev/null || break
            sleep 0.5
        done
        if kill -0 "$pid" 2>/dev/null; then
            kill -TERM "$pid" 2>/dev/null
            sleep 2
        fi
        if kill -0 "$pid" 2>/dev/null; then
            kill -KILL "$pid" 2>/dev/null
            log "  $label (PID $pid): SIGKILL."
        else
            log "  $label (PID $pid): stopped."
        fi
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
        for pattern2 in "gz sim" "async_slam_toolbox_node" \
                        "ground_truth_odom_bridge" "clock_ready_gate" \
                        "robot_state_publisher" "teleop_mux_node" "spawn_robot" \
                        "ros_gz_bridge" "parameter_bridge" "scan_range_filter" \
                        "rviz2_mapping"; do
            for p in $(pids_for_pattern_on_domain "$pattern2" "$TWIN_DOMAIN"); do
                if [[ " ${TWIN_BASELINE_PIDS} " == *" $p "* ]]; then
                    continue   # pre-existing, not ours - never touch
                fi
                warn "  orphan cleanup: killing leftover '$pattern2' (PID $p, new since our launch)"
                kill -KILL "$p" 2>/dev/null
            done
        done
    fi
    log "Teardown complete. Terminal windows (if any) may remain open (idle shell) - close them manually."
}
# PIPE is in the list deliberately. A shell killed by an UNTRAPPED fatal signal
# does not run its EXIT trap, and SIGPIPE is exactly that case: piping this
# script into something that exits early ("gripperx_desk.sh real | head") kills
# it the moment it writes to the closed pipe, and every process it had started
# (RViz, the local mapping stack, Gazebo) was left running. Observed while
# testing this script on 2026-08-20.
trap teardown INT TERM PIPE EXIT

# --- Pi health check (real mode only) ---------------------------------------

pi_health_check() {
    log "== Pi health check (read-only, ssh $SSH_HOST) =="
    # RETRY BUDGET WIDENED 2026-08-25, from 3 attempts / ConnectTimeout 5.
    # Measured that day on a fully loaded Pi (all four services active, load ~3.0,
    # goal_gateway_node 36.5 % + ros2_control_node 19.6 % + laser_scan_matcher
    # 16.3 % + steer_servo_node 15.1 %): attempts 1 AND 2 timed out and attempt 3
    # succeeded. That is the whole old budget spent on a Pi that was perfectly
    # healthy - one more slow handshake and this check would have declared the
    # robot unreachable and told the user to expect an empty RViz. A slow
    # handshake is not a down robot, and the cost of being wrong in that direction
    # is that the operator stops believing the health check.
    # The outer "timeout" is not redundant with ConnectTimeout: ConnectTimeout
    # bounds the TCP/handshake phase only, and a session that hangs AFTER
    # authentication (the loaded case) would otherwise stall here for ever.
    local out attempt=0 max=5
    while :; do
        attempt=$((attempt + 1))
        out=$(timeout 45 ssh -o ConnectTimeout=10 -o BatchMode=yes "$SSH_HOST" '
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
            # THE MAPPING STACK ON THE Pi CHANGED - a52379d, 2026-08-24.
            # (No apostrophes in this block: it is a single-quoted ssh payload.)
            # gripperx-mapping.service no longer launches gripperx_bringup/
            # mapping.launch.py (rf2o + slam_toolbox). It launches
            # gripperx_localization/localization.launch.py with enable_slam:=true
            # enable_laser_odometry:=true, i.e. ros2_laser_scan_matcher (node name
            # laser_odometry_node) + an ekf_node + odom_divergence_monitor +
            # async_slam_toolbox_node. rf2o is GONE from the robot. Asking for it
            # is why the check reported "service active but nodes are missing
            # (rf2o=0)" on a healthy, mapping robot on 2026-08-25 - and, worse,
            # why PI_MAPPING_EVIDENCE could never reach "active", which is the
            # flag that stops this laptop starting a SECOND mapping stack on
            # domain 20. rf2o is still asked for, but with the opposite meaning:
            # per a52379d rf2o and the EKF both publish odom->base_footprint, so
            # an rf2o on the Pi today is a TF fight, not a mapping stack.
            echo "N_RF2O: $(pgrep -c -f "rf2o_laser_odometry_nod[e]")"
            echo "N_LSM: $(pgrep -c -f "laser_scan_matche[r]")"
            echo "N_EKF: $(pgrep -c -f "ekf_nod[e]")"
            echo "N_SLAM: $(pgrep -c -f "async_slam_toolbox_nod[e]")"
            echo "AGENT_CT: $(docker ps --filter name=mros_agent --format "{{.Status}}" 2>/dev/null)"
        ' 2>&1)
        if [ $? -eq 0 ]; then
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
        sleep 5
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
    # Required set corrected for NFR-10 (see the header): swerve_cmd_node and
    # joint_command_bridge were in here and are NOT started any more, so this
    # check reported two missing nodes on every healthy robot. teleop_mux_node
    # was missing although the teleop cannot reach the drive without it.
    # swerve_controller is deliberately absent from this list - it is a
    # controller_manager plugin inside ros2_control_node, not a process.
    local missing=()
    [ "${n_rsp:-0}" -ge 1 ] 2>/dev/null || missing+=("robot_state_publisher")
    [ "${n_rc:-0}" -ge 1 ] 2>/dev/null || missing+=("ros2_control_node")
    [ "${n_lidar:-0}" -ge 1 ] 2>/dev/null || missing+=("ldlidar_stl_ros2_node")
    # ADDED 2026-08-25, and it is not cosmetic. Since the scan-filter change the
    # LiDAR driver publishes /scan_RAW, and scan_range_filter is what republishes
    # /scan with the self-returns (arm, gripper, all below 0.10 m) removed. If it
    # is not running there is NO /scan at all - not unfiltered data, none - so
    # slam_toolbox, both costmaps and RViz go quiet together. The launch file
    # calls that a deliberately loud failure; without this line the health check
    # was deaf to it and would have reported a healthy robot.
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
    if [ "$svc_mapping" = "active" ] && [ "${n_slam:-0}" -ge 1 ] 2>/dev/null \
       && { [ "${n_lsm:-0}" -ge 1 ] 2>/dev/null || [ "${n_ekf:-0}" -ge 1 ] 2>/dev/null; }; then
        PI_MAPPING_EVIDENCE="active"
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
    # publishes it changed with a52379d and this script has not verified the new
    # answer. rf2o published /odom (odom_topic: '/odom') and rf2o is gone; the
    # EKF's own output topic is /odometry/filtered, and one of the remappings in
    # localization.launch.py points /odometry/filtered at /wheel/odom for one of
    # the four EKF variants. Rather than guess which topic carries odometry on
    # this robot today, both are reported and neither is called a fault.
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
    # No owner names any more - see "Domain ownership" in the header. The
    # worktrees this used to name were deleted on 2026-08-25 and attributing a
    # PID to a track that no longer exists is worse than reporting the PID.
    for d in "${FOREIGN_DOMAINS[@]}"; do
        # Two buckets, and only one of them is news. See is_ros2_cli_daemon():
        # a ros2 CLI daemon carries the domain in its environ but is not another
        # session at work - it is the leftover of somebody's (often our own)
        # "ros2 topic list". Counting it as activity is what made this check cry
        # wolf on 220/221 when both were empty.
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
# standing rule (2026-07-06, per test, never blanket) requires explicit approval
# for exactly that, so it is asked for here, with the current angles on screen.

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
# every caller compares it with "=", NEVER with a glob. Reason, found in
# testing: "inactive" CONTAINS the substring "active", so [[ $state == *active* ]]
# is true for a node that is merely configured. That made the activation loop
# break before it ever called "lifecycle set activate" and then report success -
# slam_toolbox sat in "inactive [2]" and published no map at all, while the
# launcher said "ACTIVE - the map is being built".
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
# Nothing is started, stopped or enabled on the Pi; see the header (OP-20 is not
# this script's business).
#
# KNOWN WEAK POINT: rf2o consumes /scan over WiFi. Dropped or late scans become
# odometry error and the map smears. The Pi-side gripperx-mapping.service, where
# /scan never crosses the link, remains the better place for this.
LOCAL_MAP_STATE="not started"

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
    # observations came back empty-handed: ssh could not be completed (a loaded
    # Pi answers late - all five attempts timed out on 2026-08-25 while the robot
    # was perfectly healthy and mapping), and the DDS graph above showed no
    # rf2o/slam_toolbox, which on a link that cannot see the robot at all looks
    # exactly the same as a robot that is not mapping. Starting here would put a
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
    sleep 3
    if ! kill -0 "$lpid" 2>/dev/null; then
        LOCAL_MAP_STATE="FAILED to start"
        warn "local mapping: launch exited immediately - see $logfile"
        return 1
    fi
    track "local mapping launch" "$lpid" "gripperx_bringup mapping.launch.py"
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

# --- RViz --------------------------------------------------------------------

start_rviz() { # prelude config domain label
    local prelude="$1" config="$2" domain="$3" label="$4"
    local logfile="$LOG_DIR/rviz_${domain}.log"
    bash -c "$prelude exec rviz2 -d '$config'" >"$logfile" 2>&1 &
    local pid=$!
    sleep 1
    if ! kill -0 "$pid" 2>/dev/null; then
        warn "$label RViz (domain $domain) exited immediately - see $logfile"
        return 1
    fi
    track "$label RViz" "$pid" "rviz2"
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
    # parameters entry, and that file is keyed on `/**` since 2026-08-25. That is
    # what supplies a, b and wheel_radius, which keyboard_teleop_node declares
    # WITHOUT defaults (geometry single source of truth) - so this plain launch
    # still starts. Re-verified 2026-08-25 on a scratch domain; without the file
    # the node would raise at startup instead of running.
    local cmd="$prelude exec ros2 launch gripperx_teleop laptop_teleop.launch.py"
    if [ -z "$TERM_CMD" ]; then
        warn "$label teleop: no terminal emulator found (checked gnome-terminal, x-terminal-emulator)."
        warn "$label teleop: run this manually in your own terminal window instead:"
        warn "    bash -c \"$cmd\""
        warn "$label teleop: or use --input web, which needs no terminal at all."
        return 1
    fi

    "$TERM_CMD" --title="GripperX teleop ($mode, domain $domain)" -- bash -c \
        "$cmd; echo; echo '[gripperx_desk] teleop ended.'; read -r -p 'Press Enter to close this window... '" \
        >/dev/null 2>&1 &

    local waited=0 node_pid launch_pid
    while [ "$waited" -lt 15 ]; do
        node_pid=$(pids_for_pattern_on_domain "keyboard_teleop_node" "$domain" | head -n1)
        [ -n "$node_pid" ] && break
        sleep 1
        waited=$((waited + 1))
    done
    if [ -z "$node_pid" ]; then
        warn "$label teleop: no keyboard_teleop_node appeared within 15s - check the terminal window for errors."
        return 1
    fi
    launch_pid=$(pids_for_pattern_on_domain "ros2 launch gripperx_teleop laptop_teleop.launch.py" "$domain" | head -n1)
    track "$label teleop node" "$node_pid" "keyboard_teleop_node"
    [ -n "$launch_pid" ] && track "$label teleop launch" "$launch_pid" "gripperx_teleop laptop_teleop.launch.py"
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
    # inside bind() with a traceback in a log file nobody is watching, and the
    # only symptom this script would otherwise show is "no node appeared within
    # 20 s". Observed for real while this was written - an unrelated session held
    # 127.0.0.1:8080.
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
    bash -c "$prelude exec ros2 launch gripperx_teleop web_teleop.launch.py $args" \
        >"$logfile" 2>&1 &
    local lpid=$!
    sleep 2
    if ! kill -0 "$lpid" 2>/dev/null; then
        warn "$label teleop (web): launch exited immediately - see $logfile"
        return 1
    fi
    track "$label teleop launch (web)" "$lpid" "gripperx_teleop web_teleop.launch.py"

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
    track "$label teleop node (web)" "$node_pid" "web_teleop_node"

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

# --- groups -------------------------------------------------------------------

start_real() {
    log "== real: local mapping + RViz + teleop, ROS_DOMAIN_ID=$REAL_DOMAIN =="
    # Mapping first: RViz opens on Fixed Frame "map", so /map and the TF chain
    # should already be on their way when it starts. Neither of these two can
    # move anything - the gate sits in front of the teleop, which can.
    start_local_mapping
    start_rviz "$REAL_PRELUDE" "$RVIZ_CONFIG_REAL" "$REAL_DOMAIN" "real"
    start_teleop "$REAL_PRELUDE" "$REAL_DOMAIN" "real" "real"
    log "real: local mapping state: $LOCAL_MAP_STATE"
}

TWIN_LAUNCHED=0
TWIN_BASELINE_PIDS=""

start_twin() {
    log "== twin: Gazebo + RViz + teleop, ROS_DOMAIN_ID=$TWIN_DOMAIN =="
    # Corrected for NFR-10 / Gazebo Harmonic - see the header. Dropped:
    # sim_steer_bridge / swerve_cmd_node / joint_command_bridge (not started any
    # more, on either side), ros2_control_node (in the sim the controller_manager
    # lives inside the gz_ros2_control plugin, i.e. inside the gz sim process,
    # so no such process exists to collide with), gzserver (Gazebo Classic).
    # Added: clock_ready_gate and teleop_mux_node, which the twin does start.
    # scan_range_filter ADDED 2026-08-25: simulate_robot.launch.py now starts it
    # in the twin as well ("same filter as the real robot, on purpose"), it takes
    # /scan_raw -> /scan, and a second instance would republish the same topic.
    # It survives a plain Ctrl-C like the rest of this list.
    local patterns=("sim_mapping.launch.py" "gz sim" "async_slam_toolbox_node" \
                     "ground_truth_odom_bridge" "clock_ready_gate" \
                     "robot_state_publisher" "teleop_mux_node" "spawn_robot" \
                     "ros_gz_bridge" "parameter_bridge" "scan_range_filter" \
                     "rviz2_mapping")

    # Pre-flight, read-only. spawn_robot.launch.py (via sim_mapping ->
    # simulation -> simulate_robot) starts gripperx_gazebo's clock_ready_gate. If
    # that executable is missing, launch does not degrade - it aborts the ENTIRE
    # launch description and tears Gazebo down again, and the only message is
    # "executable 'clock_ready_gate' not found on the libexec directory", which
    # reads like a code bug rather than a stale install tree. Observed on this
    # laptop 2026-08-20: install/gripperx_gazebo/lib/gripperx_gazebo held only
    # sim_with_logging, dated 2026-07-17, while the source
    # src/gripperx_gazebo/scripts/clock_ready_gate is present. Warn, name the
    # fix, and continue - this script never builds anything and never writes to
    # the shared workspace.
    local gate_exe="$WS/install/gripperx_gazebo/lib/gripperx_gazebo/clock_ready_gate"
    if [ ! -x "$gate_exe" ]; then
        warn "twin: clock_ready_gate is missing from the install tree:"
        warn "  $gate_exe"
        warn "twin: sim_mapping.launch.py will ABORT on it and take Gazebo down with it."
        warn "twin: the laptop install of gripperx_gazebo is stale. Rebuild it yourself:"
        warn "    (cd '$WS' && colcon build --packages-select gripperx_gazebo)"
        warn "twin: continuing so you see the launch's own error; expect Gazebo not to stay up."
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
        warn "twin: refusing to launch Gazebo/sim_mapping - already running on domain $TWIN_DOMAIN:"
        warn "  ${collisions[*]}"
        warn "twin: this looks like a leftover/other session's sim stack. NOT touching it, NOT starting a duplicate."
        warn "twin: Gazebo skipped. Standalone RViz + teleop for the twin will still be attempted."
        TWIN_LAUNCHED=0
    else
        local logfile="$LOG_DIR/gazebo_twin.log"
        bash -c "$TWIN_PRELUDE exec ros2 launch gripperx_gazebo sim_mapping.launch.py headless:=false use_rviz:=false" \
            >"$logfile" 2>&1 &
        local gzpid=$!
        sleep 2
        if kill -0 "$gzpid" 2>/dev/null; then
            track "twin gazebo/sim_mapping" "$gzpid" "gripperx_gazebo"
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
if [ "$INPUT" = web ] && [ "$MODE" = real ]; then
    warn "real + --input web: per HANDOVER (2026-08-25) the browser UI has been run on"
    warn "  the twin (220) and on a desk rig (42, no motors) and NEVER against the real"
    warn "  robot, and the sign convention between /joint_states and the commanded pose"
    warn "  is still unmeasured. This start is that first contact. The steering gate below"
    warn "  applies to it exactly as to the keyboard teleop - it is the same publish tick."
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

log "Setup done. Press Ctrl-C here (or Enter) to stop everything this script started."
read -r -p "> " _ || true
teardown
