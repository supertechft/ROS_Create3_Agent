# ROSA Create 3 Agent

This project uses [ROSA](https://github.com/nasa-jpl/rosa) to create an embodied ROS agent that enables controlling an [iRobot Create® 3](https://edu.irobot.com/what-we-offer/create3) robot using natural language commands.

It supports both physical Create 3 robots and simulation via the [Create 3 Simulator](https://github.com/iRobotEducation/create3_sim).

---

## Features

- Natural language control of Create 3 movement and actions via text  or verbal commands 
- Integrated with ROSA's language understanding capabilities
- Works with iRobot Create 3 in a simulation or real environment

---

## Installation

### Prerequisites

This package was developed and tested on Windows 10 running a virtual machine (via [VirtualBox](https://www.virtualbox.org/)). For a complete written step-by-step environment setup installation guide that includes a ISO image with ROS 2 Humble preinstalled, refer to [this doc](https://docs.google.com/document/d/1ZO-zEPBvO-WpP5zc8WkkO2GKfG2-uJWClost-Xz_afM/edit?usp=sharing).

- Ubuntu 22.04
- ROS 2 Humble
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
- Python 3.10+
- OpenAI and HuggingFace inference API Keys // TODO: add details
- [Create 3 ROS 2 interface](https://iroboteducation.github.io/create3_docs/api/ros2/). See [Setup and Build Workspace](#setup-and-build-workspace).
- Simulator or physical robot
  - If you have the [iRobot Create® 3](https://edu.irobot.com/what-we-offer/create3), follow [these steps](https://iroboteducation.github.io/create3_docs/setup/provision/) to set it up and connect to the network.
  - Otherwise, the [Create 3 Simulator](https://github.com/iRobotEducation/create3_sim/tree/humble). See [Setup and Build Workspace](#setup-and-build-workspace).


```bash
mkdir -p create3_ws/src
cd create3_ws/src
git clone https://github.com/supertechft/ROSA_Create3_Agent
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Development Guide
- Follow these ROSA [Custom Agents](https://github.com/nasa-jpl/rosa/wiki/Custom-Agents) and [Developer Documentation](https://github.com/nasa-jpl/rosa/wiki/Developer-Documentation) guides.
- Ensure prompts are similar to [TurtleSim](https://github.com/supertechft/JPL-Mars-Rover/blob/main/src/turtle_agent/scripts/prompts.py).
- Create3 [ROS 2 documentation](https://iroboteducation.github.io/create3_docs/).
- Use [iRobot Create 3 Simulator](https://github.com/iRobotEducation/create3_sim) for testing.
- Follow [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html).

---

## Usage
Launch simulator + agent together
```bash
ros2 launch create3_agent agent_launch.py
```

This will:
- Start the Create 3 simulation in an empty world
- Start the ROSA agent connected to the Create 3 topics

Once running, you can issue commands in natural language (e.g., "Drive forward", "Turn left", "Dock with the charger").

--- 

## Project Structure
! TODO !
```
rosa_create3_agent/
├── launch/             # Launch files
├── rosa_create3_agent/ # Agent code (agent node, tools, prompts)
├── package.xml         # ROS 2 package manifest
├── setup.py            # Python package setup
├── README.md           # This file
└── LICENSE             # License
```

--- 

## License

This project is licensed under the [Apache-2.0 license](LICENSE).

--- 

## Acknowledgements
- NASA JPL - [ROSA framework](https://github.com/nasa-jpl/rosa)
- iRobot - [Create® 3 Robot](https://edu.irobot.com/what-we-offer/create3)
