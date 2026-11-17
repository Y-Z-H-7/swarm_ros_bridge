#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source "$HOME/racer/devel/setup.bash"
source "$HOME/swarm_ros_bridge_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=192.168.0.243
timeout 30 python3 /tmp/scheduler_smoke_publisher.py
