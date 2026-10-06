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
