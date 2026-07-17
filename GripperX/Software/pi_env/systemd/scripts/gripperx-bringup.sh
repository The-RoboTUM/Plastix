#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# UDP-only FastDDS transport (SHM off). After the 17:58 restart on 09.07, the
# localhost SHM data transport between the bringup processes was dead: discovery
# was working (endpoint counts visible), but NO payload data was delivered ->
# the entire on-Pi chain (mux->swerve->bridge->controller->ESP32, bridge->watchdog)
# was silent; only UDP participants (laptop teleop, ESP32 agent) worked. UDP-only
# permanently eliminates SHM as a source of failure.
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
export HOME=/home/ubuntu
exec ros2 launch gripperx_bringup real_robot.launch.py use_mock_firmware:=false use_lidar:=true
