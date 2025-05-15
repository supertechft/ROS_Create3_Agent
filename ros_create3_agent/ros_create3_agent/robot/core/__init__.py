"""
Core robot functionality module.
This package contains core functionality for the robot's operation.
https://iroboteducation.github.io/create3_docs/api/ros2/
"""

# Re-export core functionality
from .movement import drive_distance, rotate_angle, navigate_to_position, stop_robot
from .docking import dock_robot, undock_robot, check_dock_status, get_dock_info
from .sensing import (
    check_battery, 
    check_hazards,
    get_create3_info,
    get_help,
    get_examples
)
