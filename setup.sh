#!/bin/bash

# Usage: ./setup_create3_agent.sh [--sim] [--run] [--ws <workspace_path>] [--ros-distro <ros_distro>] [--venv-path <venv_path>] 

# Exit on error
set -e

# Set variables
ROS_DISTRO=humble
WS=~/create3_agent_ws
VENV_PATH=~/create3_venv
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

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
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --venv-path)
      VENV_PATH="$2"
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


# Source ROS 2 environment
echo "Sourcing ROS 2 Humble..."
if ! grep -Fxq "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi
source /opt/ros/$ROS_DISTRO/setup.bash


# Install ROS 2 interface dependencies
echo ""
echo "Installing ROS 2 interface..."
sudo dpkg --configure -a
sudo apt-get update
sudo apt-get install -y ros-humble-irobot-create-msgs portaudio19-dev


# Create workspace
echo ""
echo "Setting up ROS 2 Create 3 Agent workspace at $WS..."
mkdir -p $WS/src
cd $WS/src


# Clone repositories
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


# Install ROS dependencies
echo ""
echo "Installing ROS dependencies..."
sudo rosdep init || true
rosdep update
# Install from package.xml
rosdep install --from-paths src --ignore-src -r -y


# Install Python dependencies
echo ""
echo "Creating virtual environment at $VENV_PATH..."
python3 -m venv $VENV_PATH --system-site-packages
echo "Activating virtual environment..."
source $VENV_PATH/bin/activate

echo ""
echo "Installing Python dependencies..."
pip install --upgrade pip setuptools
pip install $WS/src/ROSA_Create3_Agent/rosa_create3_agent


# Build workspace
# If colcon build fails and we get an error like “c++: fatal error: Killed signal terminated program cc1plus, compilation terminated.”
# Try running it with --parallel-workers 2
echo ""
echo "Building workspace..."
colcon build || colcon build --parallel-workers 2

# Fix agent script to use virtualenv's Python
AGENT_SCRIPT="$WS/install/rosa_create3_agent/lib/rosa_create3_agent/agent"
if [ -f "$AGENT_SCRIPT" ]; then
  echo "Patching agent script to use virtualenv Python..."
  sed -i "1s|^.*$|#!$VENV_PATH/bin/python3|" "$AGENT_SCRIPT"
fi


# Source setup
if ! grep -Fxq "source $WS/install/setup.bash" ~/.bashrc; then
  echo "source $WS/install/setup.bash" >> ~/.bashrc
fi
source $WS/install/setup.bash


echo ""
echo "Setup complete!"
if [ "$RUN" == "true" ]; then
  echo ""
  bash "$SCRIPT_DIR/launch.sh" --ros-distro "$ROS_DISTRO" --venv-path "$VENV_PATH" --ws "$WS" ${USE_SIM:+--use-sim}
else
  if [ "$USE_SIM" == "true" ]; then
    echo "To launch the simulator: ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py"
  fi
  echo "To launch the agent: ros2 launch rosa_create3_agent agent.launch.py"
fi
