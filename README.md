# ROS Create 3 Agent

This project enables natural language control of an [iRobot Create® 3](https://edu.irobot.com/what-we-offer/create3) robot using ROS 2 and the [ROSA](https://github.com/nasa-jpl/rosa) framework. It supports both physical robots and simulation, and features a web interface with a chat box.

---

## Features

- **Natural language control**: Command the Create 3 robot via text or voice (OpenAI/HuggingFace LLM integration)
- **Chat box**: Custom chat history system (user, robot, agent messages) for clear, real-time feedback
- **Web interface**: Modern UI for chat and robot status, accessible from your browser
- **Simulation or real robot**: Works with iRobot Create 3 in a simulation or real environment
- **Extensible tools**: Modular robot actions (movement, docking, sensing, etc.)
- **Easy setup**: Automated scripts for environment, dependencies, and workspace

---

## Prerequisites

- **Ubuntu 22.04** (recommended; tested in [VirtualBox](https://www.virtualbox.org/) VM)
- **ROS 2 Humble**
- **Python 3.10+**
- **OpenAI and HuggingFace API keys** (for LLM and speech features)
- **[iRobot Create 3 ROS 2 interface](https://iroboteducation.github.io/create3_docs/api/ros2/)**
- **Simulator or physical robot**
  - If you have the [iRobot Create® 3](https://edu.irobot.com/what-we-offer/create3), follow [these steps](https://iroboteducation.github.io/create3_docs/setup/provision/) to set it up and connect to the network.
  - Otherwise, the [Create 3 Simulator](https://github.com/iRobotEducation/create3_sim/tree/humble). See [Setup and Build Workspace](#setup-and-build-workspace).

For a full step-by-step environment setup (including a prebuilt VM image), see [this doc](https://docs.google.com/document/d/1ZO-zEPBvO-WpP5zc8WkkO2GKfG2-uJWClost-Xz_afM/edit?usp=sharing).

The script mentioned in the next section sets up the workspace, installs the Create 3 simulator, Create 3 ROS 2 interface, among other things. 

---

## Quickstart: Installation & Setup

1. **Download the setup and launch scripts**:

  ```bash
  wget https://raw.githubusercontent.com/supertechft/ROSA_Create3_Agent/refs/heads/main/setup.sh
  wget https://raw.githubusercontent.com/supertechft/ROSA_Create3_Agent/refs/heads/main/launch.sh
  chmod +x setup.sh launch.sh
  ```

2. **Run the setup script**:

  ```bash
  ./setup.sh [--sim] [--run] [--ws <workspace_path>] [--ros-distro <ros_distro>] [--venv-path <venv_path>]
  ```
  - `--sim`: Also installs simulation dependencies (optional)
  - `--run`: Launches the agent after setup (optional)
  - `--ws`: Set custom workspace path (default: `~/ros_create3_agent_ws`)
  - `--venv-path`: Set custom Python venv path (default: `~/ros_create3_agent_venv`)
  - `--ros-distro`: Set ROS 2 distro (default: `humble`)

  This script will:
    - Source ROS 2 environment
    - Install Create 3 dependencies like the [Create 3 ROS 2 interface](https://iroboteducation.github.io/create3_docs/api/ros2/) and the [Create 3 Simulator](https://github.com/iRobotEducation/create3_sim/tree/humble)
    - Create the workspace and clone required repositories: this one and ROSA w/ ASR (Automatic Speech Recognition) from [this fork](https://github.com/supertechft/ROSA/tree/ASR)
    - Install ROS dependencies
    - Install Python dependencies into virtual environment
    - Build the workspace
    - Source the workspace

3. **Configure environment variables**:
  Edit `.env` file to add your `OPENAI_API_KEY` and `HF_API_KEY`. 
  - Get OpenAI key at https://platform.openai.com/account/api-keys
  - Get HuggingFace key at https://huggingface.co/settings/tokens
  // TODO: .env must be edited before colcon build in setup.sh

---

## Running the Agent

### Launch with Simulation

To start both the Create 3 simulator and the agent:

```bash
./launch.sh --sim [--persist]
```

- Add `--persist` to keep the terminal windows open after the processes exit (the default behavior is to close the terminal when the process ends).

This script
  - Sources all environments (ROS and Python `venv`)
  - Opens two terminals: one for the simulator, one for the agent.
  - The simulations starts in an empty world.
  - Opens up the web interface which is available at [http://localhost:5000](http://localhost:5000)

Once running, you can issue commands in natural language (e.g., "Drive forward", "Turn left", "Undock").

### Launch with Physical Robot

To start the agent for a real robot (no simulator):

```bash
./launch.sh [--persist]
```

- Add `--persist` to keep the terminal window open after the process exits.
- Make sure your robot is powered on and connected to the ROS network.

### Custom Options

Similar to `setup.sh`, `launch.sh` accepts `--ros-distro`, `--venv-path`, and `--ws` for custom environments. You can combine these with `--persist` as needed.

---

## Web Interface & Chat System

- Access the web UI at [http://localhost:5000](http://localhost:5000)
- The chat system is immediate and ordered: user, robot, and agent messages are shown in real time, in the order they occur.
- No timestamps are used; order is preserved by the message list.
- Robot status (battery, dock, hazards, sensors) is shown alongside the chat.
- Voice input is supported (type "audio" in the chat box).

---

## Development Notes
- Follow these ROSA [Custom Agents](https://github.com/nasa-jpl/rosa/wiki/Custom-Agents) and [Developer Documentation](https://github.com/nasa-jpl/rosa/wiki/Developer-Documentation) guides.
- Ensure prompts are similar to [TurtleSim](https://github.com/supertechft/JPL-Mars-Rover/blob/main/src/turtle_agent/scripts/prompts.py).
- Create3 [ROS 2 documentation](https://iroboteducation.github.io/create3_docs/).
- Use [iRobot Create 3 Simulator](https://github.com/iRobotEducation/create3_sim) for testing.
- Follow [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html).
- **Chat history** is managed by a custom Python list in `web/app.py` (not by ROSA)
- **Robot state** is updated via callback and shown in the web UI
- **All code is modular and extensible** for new robot actions or LLM integrations

---

## Project Structure

```
ros_create3_agent/
│   ├── package.xml                 # ROS 2 package manifest
│   ├── pyproject.toml              # Python build system config
│   ├── setup.cfg                   # Python package configuration
│   ├── setup.py                    # Python package setup script
│   ├── rosa_config.py              # Project configuration (web port, etc.)
│   ├── logging.py                  # Centralized logging utilities
│   ├── __init__.py                 # Package entry point
│   ├── agent.py                    # Main ROS agent node
│   ├── llm/                        # Language model integration (OpenAI, HF, prompts)
│   ├── robot/                      # Robot state, tools, and core actions
│   │   ├── core/                   # Movement, docking, sensing modules
│   │   ├── robot_state.py          # Robot state manager
│   │   ├── tools.py                # Robot action tool registry
│   ├── web/                        # Web server and UI
│   │   ├── app.py                  # Flask app and chat system
│   │   ├── static/                 # JS/CSS assets
│   │   ├── templates/              # HTML templates
│   │   └── __init__.py             # Web package entry
│   └── ...
├── launch/
│   └── agent.launch.py             # ROS 2 launch file for agent and simulator
├── resource/
│   └── ros_create3_agent           # ROS resource marker
├── test/                           # Lint/test scripts
│   └── ...
├── z_notes/                        # Project notes (not required)
│   └── note.md
├── .env                            # Environment variables for API keys (see below)
├── setup.sh                        # Installation and build script
├── launch.sh                       # Script to launch simulator and agent
├── .gitignore                      # Git ignore rules
├── LICENSE                         # License file
└── README.md                       # Project documentation (this file)
```

---

## License

This project is licensed under the [Apache-2.0 license](LICENSE).

---

## Acknowledgements
- NASA JPL - [ROSA framework](https://github.com/nasa-jpl/rosa)
- iRobot - [Create® 3 Robot](https://edu.irobot.com/what-we-offer/create3)
