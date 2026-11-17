#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source "$HOME/racer/devel/setup.bash"
source "$HOME/swarm_ros_bridge_ws/devel/setup.bash"
set -u
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_IP=192.168.0.243
roscore -p 11321 >/tmp/scheduler_smoke_nuc2_master.log 2>&1 &
master_pid=$!
trap 'kill "$master_pid" 2>/dev/null || true' EXIT
sleep 3
rosparam load /tmp/scheduler_smoke_nuc2_sender.yaml /sched_test_bridge
"$HOME/swarm_ros_bridge_ws/devel/lib/swarm_ros_bridge/bridge_node" __name:=sched_test_bridge
