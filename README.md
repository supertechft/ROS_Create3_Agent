# ROS Create 3 Agent

This project enables natural language control of an [iRobot Create® 3](https://edu.irobot.com/what-we-offer/create3) robot using ROS 2 and the [ROSA](https://github.com/nasa-jpl/rosa) framework. It supports both the physical [Create® 3](https://edu.irobot.com/what-we-offer/create3) robot and the [Create 3 Simulator](https://github.com/iRobotEducation/create3_sim/tree/humble), and features a web interface with a chat box.

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

---

## Quickstart: Installation & Setup

We tried our best to make the environment setup simple as possible. The script `setup.sh` installs ROS 2, sets up the workspace, installs the Create 3 simulator, Create 3 ROS 2 interface, and all dependencies to run the ROS Create 3 agent.

Alternatively, we've prepared a full step-by-step environment setup guide as well as a prebuilt VM image comes with ROS 2 and Gazebo already installed. [Read the guide here.](https://docs.google.com/document/d/1ZO-zEPBvO-WpP5zc8WkkO2GKfG2-uJWClost-Xz_afM/edit?usp=sharing).


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
  Edit `.env` file to add your `OPENAI_API_KEY` and `HF_API_KEY`. The `.env` file should be in the `~/ros_create3_agent_ws/src/ROS_Create3_Agent/ros_create3_agent` directory unless changed in the setup phase.
  - Get OpenAI key at https://platform.openai.com/account/api-keys
  - Get HuggingFace key at https://huggingface.co/settings/tokens

---

## Running the Agent

You can use the `launch.sh` script to launch the agent with ease, with or without the sim.

### Launch with Simulation

To start both the Create 3 simulator and the agent:

```bash
./launch.sh --sim [--persist]
```

- Add `--persist` to keep the terminal windows open after the processes exit (the default behavior is to close the terminal when the process ends).

This script
  - Sources all environments (ROS and Python `venv`)
  - Opens two terminals: one for the simulator, one for the agent.
  - Starts Create 3 sim (Gazebo Classic) in an empty world.
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
  - Consider adding the `--symlink-install` flag to `colcon build` to speed up development (see [humble docs](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#build-the-workspace)).
- The agent sets the Create 3's `safety_override` parameter to `backup_only` to enable moving backwards. The default is `none` which prevents the robot from not travelling backwards unless it's already explored that area but we didn't find this to work well. Note that the agent does not automatically revert to the default on shutdown. For more details, see the [Create 3 safety documentation](https://iroboteducation.github.io/create3_docs/api/safety/).
- Chat history is managed by a custom Python list in `web/app.py` (not by ROSA)
- Robot state is updated via callback and shown in the web UI `script.js`.

### Threading and Concurrency Architecture

This project uses both Python's `threading` and `concurrent.futures` modules to keep the ROS node, web server, and long-running tasks responsive and non-blocking:

- The ROS 2 node is continuously spun in a background thread (see `agent.py`), ensuring ROS callbacks (like robot state updates) are always processed, even while the web server or other tasks are running.
- The Flask web server runs in its own thread, serving the dashboard and API endpoints independently from ROS spinning.
- The `ros_create3_agent/utils/ros_threading.py` module defines two `ThreadPoolExecutor` pools:
  - One for blocking ROS operations (e.g., `rclpy.spin_until_future_complete`), used in robot tool implementations like movement and docking.
  - One for general background tasks (such as LLM calls), used to offload long-running computations from the main thread.
- All blocking or long-running operations (robot actions, LLM calls) are executed in these thread pools, so the dashboard remains live and robot state updates are always processed in real time.

**Why both are needed:**
- `threading` is ideal for persistent background loops (like spinning the ROS node or running the Flask server).
- `concurrent.futures` is best for managing pools of short-lived or concurrent tasks (like robot actions and LLM processing).

This combination ensures the agent can process ROS events, serve the web dashboard, and handle user commands all at the same time, without freezing or blocking.

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
│   │   ├── core/                   # Tool implementations: movement, docking, sensing 
│   │   ├── robot_state.py          # Robot state manager
│   │   ├── tools.py                # Robot action tool registry
│   ├── utils/                      # Shared utilities (threading, executors, etc.)
│   │   └── ros_threading.py        # Thread pools for ROS and LLM/background tasks
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
