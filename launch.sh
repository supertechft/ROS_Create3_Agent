#!/bin/bash

# Usage: ./launch.sh [--use-sim] [--ros-distro <ros_distro>] [--venv-path <venv_path>] [--ws <workspace_path>]

# Exit on error
set -e

# Default values
ROS_DISTRO="humble"
VENV_PATH="~/create3_venv"
WS="~/create3_agent_ws"
USE_SIM="false"

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
    --use-sim|--sim)
      USE_SIM="true"
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
  gnome-terminal -- bash -c "export ROS_DISTRO=$ROS_DISTRO; export VENV_PATH=$VENV_PATH; export WS=$WS; source /opt/ros/$ROS_DISTRO/setup.bash && source $VENV_PATH/bin/activate && source $WS/install/setup.bash && ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py"
  sleep 5 # Wait for the simulator to start
fi
echo "Launching Create 3 agent..."
gnome-terminal -- bash -c "export ROS_DISTRO=$ROS_DISTRO; export VENV_PATH=$VENV_PATH; export WS=$WS; source /opt/ros/$ROS_DISTRO/setup.bash && source $VENV_PATH/bin/activate && source $WS/install/setup.bash && ros2 launch rosa_create3_agent agent.launch.py"
