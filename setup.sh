#!/bin/bash

# Usage: ./setup.sh [--sim] [--run] [--ws <workspace_path>] [--ros-distro <ros_distro>] [--venv-path <venv_path>] 

# Exit on error
set -e

# Set variables
ROS_DISTRO=humble
WS=~/ros_create3_agent_ws
VENV_PATH=~/ros_create3_agent_venv
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


# Install ROS 2 if not installed
if ! command -v ros2 &> /dev/null; then
  echo ""
  echo "Installing ROS 2 $ROS_DISTRO..."

  # Set locale to UTF-8
  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  # Setup sources
  sudo apt install -y software-properties-common
  sudo add-apt-repository -y universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  # Install ROS 2
  sudo apt update && sudo apt upgrade -y
  sudo apt install -y ros-$ROS_DISTRO-desktop ros-dev-tools ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-ros2-control
else
  echo ""
  echo "ROS 2 $ROS_DISTRO is already installed."
fi


# Source ROS 2 environment
echo "Sourcing ROS 2 $ROS_DISTRO..."
if ! grep -Fxq "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
fi
source /opt/ros/$ROS_DISTRO/setup.bash


# Install system dependencies
echo ""
echo "Installing system dependencies..."
sudo dpkg --configure -a
sudo apt-get update
sudo apt-get install -y ros-$ROS_DISTRO-irobot-create-msgs portaudio19-dev python3-rosdep python3-colcon-common-extensions


# Set ROS 2 middleware
# https://iroboteducation.github.io/create3_docs/setup/xml-config/
echo ""
echo "Setting ROS 2 middleware..."
sudo apt-get install -y ros-$ROS_DISTRO-rmw-fastrtps-cpp
if ! grep -Fxq "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" ~/.bashrc; then
  echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
fi
source ~/.bashrc


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
echo "Cloning ROS Create 3 Agent..."
REPO_DIR="$WS/src/ROS_Create3_Agent"
REPO_URL="https://github.com/supertechft/ROS_Create3_Agent.git"
if [ ! -d "$REPO_DIR" ]; then
  git clone "$REPO_URL"
else
  echo "Directory '$REPO_DIR' already exists, skipping clone."
fi

cd $WS


# Install package dependencies
echo ""
echo "Installing package dependencies..."
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
# Install from setup.py / requirements.txt / pyproject.toml
pip install --upgrade pip setuptools vcstool
pip install $WS/src/ROS_Create3_Agent/ros_create3_agent


# Build workspace
# If colcon build fails and we get an error like “c++: fatal error: Killed signal terminated program cc1plus, compilation terminated.”
# Try running it with --parallel-workers 2
echo ""
echo "Building workspace..."
export IGNITION_VERSION=fortress
colcon build || colcon build --parallel-workers 2

# Fix agent script to use virtualenv's Python
AGENT_SCRIPT="$WS/install/ros_create3_agent/lib/ros_create3_agent/agent"
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
  bash "$SCRIPT_DIR/launch.sh" --ros-distro "$ROS_DISTRO" --venv-path "$VENV_PATH" --ws "$WS" ${USE_SIM:+--sim}
else
  if [ "$USE_SIM" == "true" ]; then
    echo "To launch the simulator: ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py"
  fi
  echo "To launch the agent: ros2 launch ros_create3_agent agent.launch.py"
fi
