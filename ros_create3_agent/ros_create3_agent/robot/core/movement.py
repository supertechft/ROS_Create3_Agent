"""
Movement operations for the Create 3 robot.
This module contains tools for controlling robot movement including driving and rotation.

All movement operations utilize ROS action clients defined in tools.py.
https://iroboteducation.github.io/create3_docs/api/moving-the-robot/
https://iroboteducation.github.io/create3_docs/api/drive-goals/
"""

from langchain.agents import tool
import math
from irobot_create_msgs.action import (
    DriveDistance,
    RotateAngle,
    DriveArc,
    NavigateToPosition,
)
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


def _get_drive_arc_client():
    return _get_tools()._drive_arc_client


def _get_navigate_to_position_client():
    return _get_tools()._navigate_to_position_client


@tool
def drive_distance(distance: float = 0.305) -> str:
    """
    Drive the robot forward or backward by a specified distance in meters.

    Args:
        distance (float): The distance to travel in meters (positive for forward, negative for backward).

    Returns:
        str: A message describing the result of the action.

    Performs a hazard check before moving. Will not move if hazards are detected.
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
    Rotate the robot by a specified angle in degrees.

    Args:
        angle_degrees (float): The angle to rotate in degrees (positive for counterclockwise, negative for clockwise).

    Returns:
        str: A message describing the result of the action.

    Performs a hazard check before rotating. Will not rotate if hazards are detected.
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


@tool
def drive_arc(
    radius: float = 0.3,
    angle_degrees: float = 90.0,
    translate_direction: int = 1,
    max_translation_speed: float = 0.3,
) -> str:
    """
    Drive the robot in an arc.

    Args:
        radius (float): Arc radius (meters).
        angle_degrees (float): Arc angle (degrees).
        translate_direction (int): 1=forward, -1=backward.
        max_translation_speed (float): Max speed (m/s).
    """
    try:
        # Parameter validation
        if not isinstance(radius, (int, float)):
            return "Invalid radius parameter. Must be a number."
        if not isinstance(angle_degrees, (int, float)):
            return "Invalid angle_degrees parameter. Must be a number."
        if not isinstance(translate_direction, int) or translate_direction not in [
            -1,
            1,
        ]:
            return "Invalid translate_direction parameter. Must be 1 (forward) or -1 (backward)."
        if (
            not isinstance(max_translation_speed, (int, float))
            or max_translation_speed <= 0
        ):
            return "Invalid max_translation_speed parameter. Must be a positive number."
        if radius <= 0:
            return "Invalid radius parameter. Must be positive."

        # Safety check: Ensure no hazards before moving
        hazard_result = check_hazards.invoke({"include_ir_sensors": True})
        if hazard_result.startswith("⚠️ Hazards detected:"):
            web.add_robot_message(f"Cannot drive arc due to hazards. {hazard_result}")
            return f"Arc movement canceled: {hazard_result}"

        # Prepare and send the drive arc goal
        angle_rad = math.radians(angle_degrees)
        goal = DriveArc.Goal()
        goal.angle = angle_rad
        goal.radius = radius
        goal.translate_direction = translate_direction
        goal.max_translation_speed = max_translation_speed

        direction = "forward" if translate_direction > 0 else "backward"
        arc_direction = "counterclockwise" if angle_degrees > 0 else "clockwise"
        _get_node().get_logger().info(
            f"Driving arc: radius={radius}m, angle={angle_degrees}°, direction={direction}"
        )
        web.add_robot_message(
            f"Driving {direction} in {arc_direction} arc: radius={radius:.2f}m, angle={abs(angle_degrees):.1f}°"
        )

        future = _get_drive_arc_client().send_goal_async(goal)
        spin_until_complete_in_executor(_get_node(), future).result()
        goal_handle = future.result()
        if not goal_handle.accepted:
            web.add_robot_message(f"Failed to drive arc: goal rejected")
            return "Drive arc goal was rejected"
        result_future = goal_handle.get_result_async()
        spin_until_complete_in_executor(_get_node(), result_future).result()
        result_message = f"Finished driving {direction} in {arc_direction} arc: radius={radius:.2f}m, angle={abs(angle_degrees):.1f}°"
        web.add_robot_message(f"{result_message}")
        return result_message
    except Exception as e:
        error_message = f"Error while trying to drive arc: {e}"
        web.add_robot_message(f"{error_message}")
        return error_message


@tool
def navigate_to_position(
    x: float = 1.0,
    y: float = 0.0,
    theta_degrees: float = 0.0,
    achieve_goal_heading: bool = True,
) -> str:
    """
    Move robot to (x, y, theta) using odometry.
    rotate -> translate -> rotate

    Args:
        x (float): Target x (meters).
        y (float): Target y (meters).
        theta_degrees (float): Target heading (degrees) (0 = facing positive x-axis)
        achieve_goal_heading (bool): Rotate to heading at end.
    """
    try:
        # Parameter validation
        if not isinstance(x, (int, float)):
            return "Invalid x parameter. Must be a number."
        if not isinstance(y, (int, float)):
            return "Invalid y parameter. Must be a number."
        if not isinstance(theta_degrees, (int, float)):
            return "Invalid theta_degrees parameter. Must be a number."
        if not isinstance(achieve_goal_heading, bool):
            return "Invalid achieve_goal_heading parameter. Must be a boolean."

        # Safety check: Ensure no hazards before moving
        hazard_result = check_hazards.invoke({"include_ir_sensors": True})
        if hazard_result.startswith("⚠️ Hazards detected:"):
            web.add_robot_message(f"Cannot navigate due to hazards. {hazard_result}")
            return f"Navigation canceled: {hazard_result}"

        # Convert theta to radians and create quaternion
        theta_rad = math.radians(theta_degrees)

        # Create quaternion from yaw angle (rotation around z-axis)
        # For a rotation around z-axis: q = (0, 0, sin(θ/2), cos(θ/2))
        qz = math.sin(theta_rad / 2)
        qw = math.cos(theta_rad / 2)

        # Prepare and send the navigate to position goal
        goal = NavigateToPosition.Goal()
        goal.achieve_goal_heading = achieve_goal_heading

        # Create the pose manually instead of using geometry_msgs
        goal.goal_pose.pose.position.x = x
        goal.goal_pose.pose.position.y = y
        goal.goal_pose.pose.position.z = 0.0
        goal.goal_pose.pose.orientation.x = 0.0
        goal.goal_pose.pose.orientation.y = 0.0
        goal.goal_pose.pose.orientation.z = qz
        goal.goal_pose.pose.orientation.w = qw

        _get_node().get_logger().info(
            f"Navigating to position: x={x}, y={y}, theta={theta_degrees}°"
        )
        heading_msg = (
            f", then rotate to {theta_degrees:.1f}°" if achieve_goal_heading else ""
        )
        web.add_robot_message(
            f"Navigating to position: x={x:.2f}m, y={y:.2f}m{heading_msg}"
        )

        future = _get_navigate_to_position_client().send_goal_async(goal)
        spin_until_complete_in_executor(_get_node(), future).result()
        goal_handle = future.result()
        if not goal_handle.accepted:
            web.add_robot_message(f"Failed to navigate: goal rejected")
            return "Navigate to position goal was rejected"
        result_future = goal_handle.get_result_async()
        spin_until_complete_in_executor(_get_node(), result_future).result()
        result_message = f"Finished navigating to position: x={x:.2f}m, y={y:.2f}m, theta={theta_degrees:.1f}°"
        web.add_robot_message(f"{result_message}")
        return result_message
    except Exception as e:
        error_message = f"Error while trying to navigate: {e}"
        web.add_robot_message(f"{error_message}")
        return error_message
