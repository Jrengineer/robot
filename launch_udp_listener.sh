#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_plc_ws/install/setup.bash
ros2 run plc_comm udp_listener_node
