"""
Movement operations for the Create 3 robot.
This module contains tools for controlling robot movement including driving and rotation.

All movement operations utilize ROS action clients defined in tools.py.
https://iroboteducation.github.io/create3_docs/api/moving-the-robot/
https://iroboteducation.github.io/create3_docs/api/drive-goals/
"""

from langchain.agents import tool
import math
from irobot_create_msgs.action import DriveDistance, RotateAngle
from ros_create3_agent.web import app as web
from ros_create3_agent.utils.ros_threading import spin_until_complete_in_executor
from .sensing import check_hazards


# Accessors for shared ROS node and action clients from the tools module.
# These are used instead of direct imports because the objects are initialized at runtime.
def _get_tools():
    from .. import tools

    return tools


def _get_node():
    return _get_tools()._node


def _get_drive_distance_client():
    return _get_tools()._drive_distance_client


def _get_rotate_angle_client():
    return _get_tools()._rotate_angle_client


@tool
def drive_distance(distance: float = 0.305) -> str:
    """
    Drive robot a set distance (meters).
    Args:
        distance (float): Meters to move (+forward, -back).
    Returns:
        str: Result message.
    """
    try:
        # Parameter validation
        if not isinstance(distance, (int, float)):
            return "Invalid distance parameter. Must be a number."

        # Safety check: Ensure no hazards before moving
        hazard_result = check_hazards.invoke({"include_ir_sensors": True})
        if hazard_result.startswith("⚠️ Hazards detected:"):
            web.add_robot_message(f"Cannot move due to hazards. {hazard_result}")
            return f"Movement canceled: {hazard_result}"

        # Send the drive goal
        goal = DriveDistance.Goal()
        goal.distance = distance
        _get_node().get_logger().info(f"Moving {distance} meters...")
        direction = "forward" if distance > 0 else "backward"
        web.add_robot_message(f"Moving {abs(distance):.2f} meters {direction}...")
        future = _get_drive_distance_client().send_goal_async(goal)
        spin_until_complete_in_executor(_get_node(), future).result()
        goal_handle = future.result()
        if not goal_handle.accepted:
            web.add_robot_message(f"Move distance goal was rejected")
            return "Move distance goal was rejected"
        result_future = goal_handle.get_result_async()
        spin_until_complete_in_executor(_get_node(), result_future).result()
        result_message = f"Finished moving {abs(distance):.2f} meters {direction}"
        web.add_robot_message(f"{result_message}")
        return result_message
    except Exception as e:
        error_message = f"Error while trying to move: {e}"
        web.add_robot_message(f"{error_message}")
        return error_message


@tool
def rotate_angle(angle_degrees: float = 90) -> str:
    """
    Rotate robot by angle (degrees).
    Args:
        angle_degrees (float): Degrees to rotate (+ccw, -cw).
    Returns:
        str: Result message.
    """
    try:
        # Parameter validation
        if not isinstance(angle_degrees, (int, float)):
            return "Invalid angle_degrees parameter. Must be a number."

        # Safety check: Ensure no hazards before rotating
        hazard_result = check_hazards.invoke({"include_ir_sensors": False})
        if hazard_result.startswith("⚠️ Hazards detected:"):
            web.add_robot_message(f"Cannot rotate due to hazards. {hazard_result}")
            return f"Rotation canceled: {hazard_result}"

        # Prepare and send the rotate goal
        angle_rad = math.radians(angle_degrees)
        goal = RotateAngle.Goal()
        goal.angle = angle_rad
        direction = "counterclockwise" if angle_degrees > 0 else "clockwise"
        _get_node().get_logger().info(f"Rotating {angle_degrees} degrees...")
        web.add_robot_message(
            f"Rotating {abs(angle_degrees):.2f} degrees {direction}..."
        )
        future = _get_rotate_angle_client().send_goal_async(goal)
        spin_until_complete_in_executor(_get_node(), future).result()
        goal_handle = future.result()
        if not goal_handle.accepted:
            web.add_robot_message(f"Failed to rotate: goal rejected")
            return "Rotate angle goal was rejected"
        result_future = goal_handle.get_result_async()
        spin_until_complete_in_executor(_get_node(), result_future).result()
        result_message = (
            f"Finished rotating {abs(angle_degrees):.2f} degrees {direction}"
        )
        web.add_robot_message(f"{result_message}")
        return result_message
    except Exception as e:
        error_message = f"Error while trying to rotate: {e}"
        web.add_robot_message(f"{error_message}")
        return error_message
