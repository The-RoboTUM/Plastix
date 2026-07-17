#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Stop any old container
docker stop mros_agent 2>/dev/null

# UDP-only FastDDS transport (SHM off), consistent with the other gripperx
# services (rationale: gripperx-bringup.sh). Mount the profile into the container
# and enable it via FASTRTPS_DEFAULT_PROFILES_FILE so the DDS side of the
# agent (ESP32 <-> /hw/joint_commands etc.) doesn't run over SHM either.
exec docker run --rm --name mros_agent --net=host --ipc=host --privileged \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_udp_only.xml \
  -v /home/ubuntu/fastdds_udp_only.xml:/fastdds_udp_only.xml:ro \
  -v /dev:/dev \
  microros/micro-ros-agent:jazzy \
  serial --dev /dev/esp32 -b 115200
