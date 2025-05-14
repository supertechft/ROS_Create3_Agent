#!/bin/bash

# Usage: ./launch.sh [--sim] [--ws <workspace_path>]

WS=~/create3_agent_ws
USE_SIM="false"
while [[ $# -gt 0 ]]; do
  case $1 in
    --sim)
      USE_SIM="true"
      shift
      ;;
    --ws)
      WS="$2"
      shift 2
      ;;
    *)
      shift
      ;;
  esac
done

if [ "$USE_SIM" == "true" ]; then
  gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py"
  sleep 5 # Wait for the simulator to start
fi
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash && source $WS/install/setup.bash && ros2 launch rosa_create3_agent agent.launch.py"
