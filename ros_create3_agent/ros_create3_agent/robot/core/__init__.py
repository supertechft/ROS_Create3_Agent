"""
Core robot functionality module.
This package contains core functionality for the robot's operation.
https://iroboteducation.github.io/create3_docs/api/ros2/
"""

# Re-export core functionality
from .docking import dock_robot, undock_robot, check_dock_status
from .info import (
    agent_intro,
    get_help,
    get_examples,
    get_dock_info,
    get_create3_specs,
    get_create3_interface,
)
from .movement import drive_distance, rotate_angle, drive_arc, navigate_to_position
from .sensing import (
    get_battery_status,
    check_hazards,
    get_imu_status,
    get_kidnap_status,
    get_odometry,
    get_stop_status,
)
