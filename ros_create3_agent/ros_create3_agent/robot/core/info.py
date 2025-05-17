"""
Info operations for the Create 3 robot.
This module contains functions for general information, help, and example queries for the agent.
https://iroboteducation.github.io/create3_docs/hw/overview/
"""

from langchain.agents import tool


@tool
def agent_intro() -> str:
    """
    Introduction to the Create 3 ROS Agent and how to interact with it.
    Describes the agent's capabilities, how to use text and voice (audio) input, and lists available info/help tools.
    Returns:
        A string introduction for the user.
    """
    return """
    Welcome to the iRobot Create 3 ROS Agent!

    You can control and interact with the Create 3 robot using natural language commands. The agent supports both text and voice input—just type "audio" to activate the microphone and use your voice.

    **What can I do?**
    - Move, rotate, and navigate the robot
    - Check battery, hazards, and dock status
    - Dock/undock the robot
    - Query ROS 2 topics, services, actions, and parameters
    - Get help, usage examples, and platform info

    **Example queries:**
    - "Move the robot forward 1 meter."
    - "Rotate 90 degrees left."
    - "Check battery status."
    - "Dock the robot."
    - "List all ROS topics."
    - "Show me example commands."
    - "Help"

    For more, type "help" or ask about any capability!
    """


@tool
def get_help() -> str:
    """
    Show help and usage tips for the Create 3 agent, including core and advanced capabilities, safety, and troubleshooting.
    Returns:
        A string with help and usage tips for the agent.
    """
    return """
    # Create3 Agent Help

    This agent provides an interface for controlling and interacting with the iRobot Create 3 robot using ROS 2.

    ## Getting Started
    - Use text or type "audio" for voice input
    - Ask for help, info, or example commands at any time

    ## Core Capabilities
    - Move, rotate, and navigate the robot
    - Check robot status (battery, dock, hazards)
    - Dock/undock
    - ROS 2 integration: topics, services, parameters, logs

    ## Advanced Commands
    - List/read ROS log files (`roslog_list`, `read_log`)
    - Check ROS setup (`ros2_doctor`)
    - Interact with services (`ros2_service_list`, etc.)
    - Manage parameters (`ros2_param_list`, etc.)
    - Work with topics (`ros2_topic_list`, etc.)
    - Node info (`ros2_node_list`, `ros2_node_info`)
    - Control agent behavior (`set_verbosity`, `wait`)

    ## Tips
    - Always check for hazards before moving
    - Verify position before/after movements
    - Use clear, specific instructions
    - If docked, undock before moving
    - Use ROS tools for debugging

    For more, visit: https://iroboteducation.github.io/create3_docs/
    """


@tool
def get_examples() -> str:
    """
    Show example queries and commands for interacting with the Create 3 robot and ROS 2 environment.
    Returns:
        A string with example queries and commands.
    """
    return """
    Example queries you can try:
    - Move the robot forward at 0.1 m/s for 5 seconds.
    - Rotate the robot 90 degrees clockwise.
    - Check if there are any hazards detected.
    - Check if the robot is currently docked.
    - Draw a square pattern with 0.5 meter sides.
    - List all active ROS topics.
    - Check the system with ros2_doctor.
    - Show me the available ROS services.
    - Listen to the /hazard_detection topic for 3 messages.
    - Get the parameters for the motion_control node.
    - Check the ROS logs for any errors.
    - Show me information about the robot's sensor nodes.
    - What topics are publishing sensor data?
    - Give me a ROS 2 tutorial for working with the Create 3 robot.
    - How do I use ROS parameters with the Create 3?
    """


@tool
def get_dock_info() -> str:
    """
    Provide general information about the Create 3 Home Base docking station, its features, and its role in robot operation and navigation.
    Returns:
        A string describing the docking station and its features.
    """
    return """
    The Create 3 Home Base is the docking station for the robot.

    Features:
    - Charges the robot's battery
    - Provides a fixed reference point for navigation
    - IR emitters guide the robot when docking
    """


@tool
def get_create3_specs() -> str:
    """
    Get general information about hardware specs and platform overview for the iRobot Create 3 robot, including sensors, processing, and connectivity.
    Returns:
        A string describing the robot's hardware specifications.
    """
    return """
    The iRobot Create 3 is an educational robot platform based on the Roomba robot vacuum.

    Specifications:
    - Diameter: 339 mm (13.34 in)
    - Height: 91 mm (3.58 in)
    - Weight: 2.31 kg (5.09 lbs)
    - Max speed: 0.306 m/s
    - Sensors: Bump, cliff, wheel drop, optical tracking
    - Battery: 26Wh Li-ion, ~2 hours runtime
    - Processor: Raspberry Pi 4 Compute Module
    - Connectivity: Wi-Fi, Bluetooth
    - ROS 2 compatible
    """


@tool
def get_create3_interface() -> str:
    """
    Briefly list the main ROS 2 interface for the iRobot Create 3 robot, including all topics, services, actions, and key parameters.
    This tool is useful for users who want to know what APIs are available for interacting with the robot via ROS 2.
    Returns:
        A formatted string summarizing the main topics, services, actions, and parameters.
    """
    return """
    # Create 3 ROS 2 Interface Overview

    ## Topics
    - /battery_state [sensor_msgs/msg/BatteryState]
    - /ir_intensity [irobot_create_msgs/msg/IrIntensityVector]
    - /cmd_audio [irobot_create_msgs/msg/AudioNoteVector]
    - /cmd_lightring [irobot_create_msgs/msg/LightringLeds]
    - /cmd_vel [geometry_msgs/msg/Twist]
    - /dock_status [irobot_create_msgs/msg/DockStatus]
    - /hazard_detection [irobot_create_msgs/msg/HazardDetectionVector]
    - /imu [sensor_msgs/msg/Imu]
    - /interface_buttons [irobot_create_msgs/msg/InterfaceButtons]
    - /ir_opcode [irobot_create_msgs/msg/IrOpcode]
    - /kidnap_status [irobot_create_msgs/msg/KidnapStatus]
    - /mouse [irobot_create_msgs/msg/Mouse]
    - /odom [nav_msgs/msg/Odometry]
    - /parameter_events [rcl_interfaces/msg/ParameterEvent]
    - /rosout [rcl_interfaces/msg/Log]
    - /slip_status [irobot_create_msgs/msg/SlipStatus]
    - /stop_status [irobot_create_msgs/msg/StopStatus]
    - /tf [tf2_msgs/msg/TFMessage]
    - /tf_static [tf2_msgs/msg/TFMessage]
    - /wheel_status [irobot_create_msgs/msg/WheelStatus]
    - /wheel_ticks [irobot_create_msgs/msg/WheelTicks]
    - /wheel_vels [irobot_create_msgs/msg/WheelVels]

    ## Services
    - /e_stop [irobot_create_msgs/srv/EStop]
    - /describe_parameters [rcl_interfaces/srv/DescribeParameters]
    - /get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
    - /get_parameters [rcl_interfaces/srv/GetParameters]
    - /list_parameters [rcl_interfaces/srv/ListParameters]
    - /set_parameters [rcl_interfaces/srv/SetParameters]
    - /set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
    - /robot_power [irobot_create_msgs/srv/RobotPower]
    - /change_state [lifecycle_msgs/srv/ChangeState]
    - /get_available_states [lifecycle_msgs/srv/GetAvailableStates]
    - /get_available_transitions [lifecycle_msgs/srv/GetAvailableTransitions]
    - /get_state [lifecycle_msgs/srv/GetState]
    - /get_transition_graph [lifecycle_msgs/srv/GetAvailableTransitions]

    ## Actions
    - /audio_note_sequence [irobot_create_msgs/action/AudioNoteSequence]
    - /dock [irobot_create_msgs/action/Dock]
    - /drive_arc [irobot_create_msgs/action/DriveArc]
    - /drive_distance [irobot_create_msgs/action/DriveDistance]
    - /led_animation [irobot_create_msgs/action/LedAnimation]
    - /navigate_to_position [irobot_create_msgs/action/NavigateToPosition]
    - /rotate_angle [irobot_create_msgs/action/RotateAngle]
    - /undock [irobot_create_msgs/action/Undock]
    - /wall_follow [irobot_create_msgs/action/WallFollow]

    ## Key Parameters
    - max_speed, min_speed, wheel_accel_limit, wheel_base, wheels_encoder_resolution, wheels_radius
    - reflexes_enabled, safety_override, use_sim_time
    - reflexes.REFLEX_BUMP, reflexes.REFLEX_CLIFF, reflexes.REFLEX_DOCK_AVOID, reflexes.REFLEX_GYRO_CAL, reflexes.REFLEX_PANIC, reflexes.REFLEX_PROXIMITY_SLOWDOWN, reflexes.REFLEX_STUCK, reflexes.REFLEX_VIRTUAL_WALL, reflexes.REFLEX_WHEEL_DROP
    - publish_odom_tfs, lightring_led_brightness, log_period
    - qos_overrides./parameter_events.publisher.depth, qos_overrides./parameter_events.publisher.durability, qos_overrides./parameter_events.publisher.history, qos_overrides./parameter_events.publisher.reliability
    - qos_overrides./tf.publisher.depth, qos_overrides./tf.publisher.durability, qos_overrides./tf.publisher.history, qos_overrides./tf.publisher.reliability
    - qos_overrides./tf_static.publisher.depth, qos_overrides./tf_static.publisher.history, qos_overrides./tf_static.publisher.reliability

    For full details, see: https://iroboteducation.github.io/create3_docs/api/ros2/
    """
