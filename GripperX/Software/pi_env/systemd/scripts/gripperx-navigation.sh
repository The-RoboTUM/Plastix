#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/Software/ros2/install/setup.bash
# PlastiX domain convention — see gripperx-bringup.sh. Real GripperX-1 = 20.
export ROS_DOMAIN_ID=20
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# UDP-only FastDDS transport (SHM off) — must be consistent across all gripperx
# services, otherwise localhost data paths break. Rationale: gripperx-bringup.sh.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
export HOME=/home/ubuntu

# Fix 8 (#12), safety-relevant: previously, EVERY start of this service (i.e.
# also after every boot/restart, without any user action) switched the teleop
# mux into the drive-ready autonomous mode via
# "ros2 topic pub /teleop/set_mode ... autonomous" - so after a boot the robot
# would, on its own, end up in a state where Nav2 commands are passed straight
# through to the wheels. In addition, the code was broken ($!-bug: "MODE_PID="
# without "$!", followed by "kill" with no PID argument at all), so the pub
# background process was never terminated in a targeted way (it only happened
# to exit on its own via "--times 5").
# The block is removed outright rather than fixed: teleop_mux starts with
# initial_mode=keyboard (see gripperx_teleop/config/teleop_mux.yaml) and stays in
# this safe default after a boot/restart of gripperx-navigation.service.
# Switching to "autonomous" still requires an explicit user action (e.g. a key
# in keyboard_teleop_node or a manual
# "ros2 topic pub /teleop/set_mode ...").
# OP-14 consolidation (user decision 2026-07-09, gate opened by the DT-5/M3
# acceptance on 2026-07-17): the canonical Nav2 stack is gripperx_planning, and
# it is now the stack this service starts. gripperx_bringup/navigation.launch.py
# and gripperx_bringup/config/nav2_params.yaml are deleted.
#
# use_sim_time:=false is passed EXPLICITLY even though it is now also the launch
# default. Doubly deliberate: this service is the reason the default was flipped,
# and stating it here documents the intent at the call site. Without it, Nav2
# would block waiting for a /clock that nothing publishes on the real robot.
exec ros2 launch gripperx_planning navigation.launch.py use_sim_time:=false
