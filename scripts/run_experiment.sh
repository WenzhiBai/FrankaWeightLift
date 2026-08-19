#!/bin/bash

# Source environment
source /opt/ros/noetic/setup.bash
source ~/workspace/catkin_ws_franka/devel/setup.bash
source ~/workspace/catkin_ws_fwl/devel/setup.bash

usage() {
  echo "Usage: run this script from the root of the project."
  echo "Example: "
  echo "./scripts/run_experiment.sh [desired_force_in_newtons] [robot_ip]"
  echo "  desired_force_in_newtons: downward force at the end effector, default 0.0"
  echo "  robot_ip: default 192.168.1.11"
  echo "rqt_reconfigure is always opened, so desired_force can be set live."
}

if [ $# -gt 2 ]; then
  usage
  exit 1
fi

DESIRED_FORCE=${1:-0.0}
ROBOT_IP=${2:-192.168.1.11}

echo "Desired force: ${DESIRED_FORCE} N downwards, robot ip: ${ROBOT_IP}"
echo "The arm MOVES ON ITS OWN after startup to align with the robot base."
echo "Stay clear and do NOT touch the robot until 'weight lift active' is logged."
echo "Then set desired_force in the rqt_reconfigure window that opens; the virtual"
echo "walls anchor at the pose the tool is in when it first goes non-zero."

# Trap Ctrl+C
trap ctrl_c INT
ctrl_c() {
  echo "Stopping child processes..."
  kill $FWL_PID
  # Remove ANSI escape sequences from ROS log files
  find ~/.ros/log/latest/ -type f -name "*.log" -exec sed -i 's/\x1b\[[0-9;]*m//g' {} \;
  exit 0
}

roslaunch franka_weight_lift weight_lift.launch \
  desired_force:=$DESIRED_FORCE \
  robot_ip:=$ROBOT_IP \
  rqt_reconfigure:=true &
FWL_PID=$!

# Wait for the processes to finish
wait
