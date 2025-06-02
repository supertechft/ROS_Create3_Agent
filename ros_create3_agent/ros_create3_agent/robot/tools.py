"""
Tools for operating the iRobot Create 3 robot using ROS 2 Interface
https://iroboteducation.github.io/create3_docs/api/ros2/

This module re-exports all robot tools from the core modules.
"""

from rclpy.action import ActionClient
from rclpy.node import Node

# Import Create 3 Actions
from irobot_create_msgs.action import (
    DriveDistance,
    RotateAngle,
    DriveArc,
    NavigateToPosition,
    Undock,
    Dock,
)

# Import from core modules to re-export
from .core.docking import (
    dock_robot,
    undock_robot,
    check_dock_status,
)
from .core.info import (
    agent_intro,
    get_help,
    get_examples,
    get_dock_info,
    get_create3_specs,
    get_create3_interface,
)
from .core.movement import (
    drive_distance,
    rotate_angle,
    drive_arc,
    navigate_to_position,
)
from .core.sensing import (
    get_battery_status,
    check_hazards,
    get_imu_status,
    get_kidnap_status,
    get_odometry,
    get_stop_status,
)
from .core.sequences import (
    dance,
)

# Import robot state manager
from .robot_state import get_robot_state

# Import safety configuration
from .safety import configure_safety_parameters

# Global variables for shared state
_node = None
_drive_distance_client = None
_rotate_angle_client = None
_drive_arc_client = None
_navigate_to_position_client = None
_dock_client = None
_undock_client = None


def initialize(node: Node):
    """Initialize all the ROS clients and subscribers."""
    global _node, _drive_distance_client, _rotate_angle_client, _drive_arc_client, _navigate_to_position_client, _dock_client, _undock_client

    _node = node

    # Action clients
    _drive_distance_client = ActionClient(_node, DriveDistance, "drive_distance")
    _rotate_angle_client = ActionClient(_node, RotateAngle, "rotate_angle")
    _drive_arc_client = ActionClient(_node, DriveArc, "drive_arc")
    _navigate_to_position_client = ActionClient(
        _node, NavigateToPosition, "navigate_to_position"
    )
    _dock_client = ActionClient(_node, Dock, "dock")
    _undock_client = ActionClient(_node, Undock, "undock")

    # Wait for action servers
    _node.get_logger().info("Waiting for action servers...")
    _drive_distance_client.wait_for_server()
    _rotate_angle_client.wait_for_server()
    _drive_arc_client.wait_for_server()
    _navigate_to_position_client.wait_for_server()
    _dock_client.wait_for_server()
    _undock_client.wait_for_server()
    _node.get_logger().info("All action servers are ready!")

    # Initialize the robot state with the node
    get_robot_state(node)

    # Configure safety parameters: disable backup limit
    # https://iroboteducation.github.io/create3_docs/api/safety/
    configure_safety_parameters(_node, safety_override="backup_only")
