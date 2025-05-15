#!/bin/bash

# Usage: ./launch.sh [--sim] [--persist] [--ros-distro <ros_distro>] [--venv-path <venv_path>] [--ws <workspace_path>] 

# Exit on error
set -e

# Default values
ROS_DISTRO="humble"
WS=~/ros_create3_agent_ws
VENV_PATH=~/ros_create3_agent_venv
USE_SIM="false"
PERSIST="false"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case $1 in
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --venv-path)
      VENV_PATH="$2"
      shift 2
      ;;
    --ws)
      WS="$2"
      shift 2
      ;;
    --sim)
      USE_SIM="true"
      shift
      ;;
    --persist)
      PERSIST="true"
      shift
      ;;
    *)
      shift
      ;;
  esac
done


# Launch the Create 3 Agent (and simulator)
echo ""
if [ "$USE_SIM" == "true" ]; then
  echo "Launching Create 3 simulator..."
  SIM_CMD="export ROS_DISTRO=$ROS_DISTRO; export VENV_PATH=$VENV_PATH; export WS=$WS; source /opt/ros/$ROS_DISTRO/setup.bash && source $VENV_PATH/bin/activate && source $WS/install/setup.bash && ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py"
  if [ "$PERSIST" == "true" ]; then
    SIM_CMD="$SIM_CMD; exec bash"
  fi
  gnome-terminal -- bash -c "$SIM_CMD"
  sleep 5 # Wait for the simulator to start
fi
echo "Launching Create 3 agent..."
AGENT_CMD="export ROS_DISTRO=$ROS_DISTRO; export VENV_PATH=$VENV_PATH; export WS=$WS; source /opt/ros/$ROS_DISTRO/setup.bash && source $VENV_PATH/bin/activate && source $WS/install/setup.bash && ros2 launch ros_create3_agent agent.launch.py"
if [ "$PERSIST" == "true" ]; then
  AGENT_CMD="$AGENT_CMD; exec bash"
fi
gnome-terminal -- bash -c "$AGENT_CMD"
