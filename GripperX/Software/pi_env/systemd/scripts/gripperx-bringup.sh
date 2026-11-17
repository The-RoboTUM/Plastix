#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/Software/ros2/install/setup.bash
# PlastiX domain convention (2026-08-13, project-wide): gripper robots take ids in the
# 20s, their digital twin the same id +200. Real GripperX-1 = 20, its twin = 220.
#
# Why +200 and not +100: ROS 2 derives the DDS base port as 7400 + 250*domain_id. With
# Linux's default ephemeral range 32768-60999, ids 102-214 land inside it and can lose
# discovery to whatever grabbed the port first — an intermittent, hard-to-reproduce
# failure. Safe bands are 0-101 and 215-232, so 20 and 220 are both clean, and the
# whole gripper range 20-29 maps onto 220-229 without leaving the safe bands.
# Verify the local range with: sysctl net.ipv4.ip_local_port_range
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# UDP-only FastDDS transport (SHM off). After the 17:58 restart on 09.07, the
# localhost SHM data transport between the bringup processes was dead: discovery
# was working (endpoint counts visible), but NO payload data was delivered ->
# the entire on-Pi chain (mux->swerve->bridge->controller->ESP32, bridge->watchdog)
# was silent; only UDP participants (laptop teleop, ESP32 agent) worked. UDP-only
# permanently eliminates SHM as a source of failure.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
export HOME=/home/ubuntu
# WHOLE-DRIVE FAULT MARKER (user decision 2026-10-16). swerve_controller writes it
# on a drive fault and re-adopts it on every load, so a crash-respawn of
# ros2_control_node keeps the robot stopped. A DELIBERATE stack start is the one
# thing that clears it -- this script, on an EXPLICIT start (`systemctl start` /
# `restart`, the Mode R button, gripperx-stack-stop --mode=restart). NOT on
# systemd's automatic Restart=on-failure: NRestarts is > 0 exactly then
# (measured on systemd 255: first start 0, automatic restart 1, explicit
# `systemctl restart` 0 again). If the counter cannot be read the marker is KEPT
# -- the safe direction -- and the line below says how to clear it by hand.
DRIVE_FAULT_MARKER=/run/gripperx/drive_fault
if [ -e "$DRIVE_FAULT_MARKER" ]; then
    NRESTARTS=$(systemctl show -p NRestarts --value gripperx-bringup.service 2>/dev/null)
    if [ "$NRESTARTS" = "0" ]; then
        echo "[bringup] deliberate start: clearing the drive-fault marker:"
        cat "$DRIVE_FAULT_MARKER"
        rm -f "$DRIVE_FAULT_MARKER"
    else
        echo "[bringup] DRIVE FAULT MARKER KEPT (NRestarts='$NRESTARTS', automatic restart or" \
             "unknown): the drive stays stopped. Clear deliberately: rm $DRIVE_FAULT_MARKER"
    fi
fi
# NO LONGER INTERIM (updated 2026-08-25). This started as a stopgap while the arm
# poses were stale (#340). The home pose has since been re-taught and accepted, and
# the user decided startup homing stays OFF permanently: the node runs with
# respawn=True, so homing on startup is unattended arm motion repeated on events
# that have nothing to do with the arm. A correct pose does not make that safe.
#
# The launch default is now false as well, so this flag is redundant -- it is kept
# deliberately as defence in depth, because THIS FILE does not deploy via git pull
# (the unit runs the copy in /usr/local/bin/) and could therefore be older than the
# tree it belongs to. Belt and braces on the same decision, not two decisions.
exec ros2 launch gripperx_bringup real_robot.launch.py use_mock_firmware:=false use_lidar:=true arm_home_on_startup:=false
