#!/bin/bash

# Usage: ./setup_create3_agent.sh [--sim] [--ws <workspace_path>] [--run]

set -e

# Default workspace
WS=~/create3_agent_ws

# Parse arguments
USE_SIM="false"
RUN="false"
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
    --run)
      RUN="true"
      shift
      ;;
    *)
      shift
      ;;
  esac
done


echo "Setting up ROS 2 Create 3 Agent workspace at $WS..."


# 1. Install ROS 2 interface dependencies
echo ""
echo "Installing ROS 2 interface..."
sudo dpkg --configure -a
sudo apt-get update
sudo apt-get install -y ros-humble-irobot-create-msgs


# 2. Create workspace
echo ""
echo "Creating workspace..."
mkdir -p $WS/src
cd $WS/src


# 3. Clone repositories
if [ "$USE_SIM" == "true" ]; then
  #  Clone Create 3 Simulator
  echo ""
  echo "Cloning Create 3 Simulator..."
  REPO_DIR="$WS/src/create3_sim"
  REPO_URL="https://github.com/iRobotEducation/create3_sim.git"
  if [ ! -d "$REPO_DIR" ]; then
    git clone -b humble "$REPO_URL"
  else
    echo "Directory '$REPO_DIR' already exists, skipping clone."
  fi
fi

echo ""
echo "Cloning ROSA Create 3 Agent..."
REPO_DIR="$WS/src/ROSA_Create3_Agent"
REPO_URL="https://github.com/supertechft/ROSA_Create3_Agent.git"
if [ ! -d "$REPO_DIR" ]; then
  git clone "$REPO_URL"
else
  echo "Directory '$REPO_DIR' already exists, skipping clone."
fi

cd $WS


# 4. Install dependencies
echo ""
echo "Installing dependencies..."
sudo rosdep init || true
rosdep update
# Install from package.xml
rosdep install --from-paths src --ignore-src -r -y


# 5. Build workspace
# If colcon build fails and we get an error like “c++: fatal error: Killed signal terminated program cc1plus, compilation terminated.”
# Try running it with --parallel-workers 2
echo ""
echo "Building workspace..."
colcon build || colcon build --parallel-workers 2


# 6. Source setup
echo "source $WS/install/setup.bash" >> ~/.bashrc
source $WS/install/setup.bash


echo ""
echo "Setup complete!"
if [ "$RUN" == "true" ]; then
  bash "$(dirname "$0")/launch.sh" ${USE_SIM:+--sim} --ws "$WS"
else
  if [ "$USE_SIM" == "true" ]; then
    echo "To launch the simulator: ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py"
  fi
  echo "To launch the agent: ros2 launch rosa_create3_agent agent.launch.py"
fi
