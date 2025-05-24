"""
Docking operations for the Create 3 robot.
This module contains tools for docking, undocking, and checking dock status.
https://iroboteducation.github.io/create3_docs/api/docking/
"""

import time
from langchain.agents import tool
from ros_create3_agent.web import app as web
from ..robot_state import get_robot_state
from irobot_create_msgs.action import Dock, Undock
import rclpy


# Accessors for shared ROS node and action clients from the tools module.
# These are used instead of direct imports because the objects are initialized at runtime.
def _get_tools():
    from .. import tools

    return tools


def _get_node():
    return _get_tools()._node


def _get_dock_client():
    return _get_tools()._dock_client


def _get_undock_client():
    return _get_tools()._undock_client


def _get_dock_status_info():
    """Get dock status info."""
    robot_state = get_robot_state()
    state = robot_state.get_state()
    dock_status = state.get("dock_status", {})

    return dock_status.get("dock_visible"), dock_status.get("is_docked")


def _check_dock_visibility_and_warn(dock_visible, operation="dock"):
    """Check visibility and warn."""
    if dock_visible is False:
        warning_msg = f"⚠️ Dock is not visible to robot sensors. {operation.capitalize()}ing may fail."
        web.add_robot_message(f"🤖 {warning_msg}")
        _get_node().get_logger().warn(
            f"Dock not visible, attempting to {operation} anyway"
        )
    elif dock_visible is None:
        warning_msg = f"⚠️ Dock visibility unknown. Proceeding with {operation} attempt."
        web.add_robot_message(f"🤖 {warning_msg}")


def _execute_dock_action(action_client, goal, action_name):
    """Execute dock action."""
    _get_node().get_logger().info(f"{action_name}...")
    web.add_robot_message(f"🤖 Attempting to {action_name.lower()}...")

    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(_get_node(), future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        web.add_robot_message(f"🤖 {action_name} goal was rejected")
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(_get_node(), result_future)

    return True


def _get_status_message(include_details=True):
    """Get status message."""
    dock_visible, is_docked = _get_dock_status_info()
    battery = get_robot_state().get_state().get("battery", {})
    battery_status = battery.get("status", "Unknown")

    if is_docked:
        charging = ""
        if battery_status == "Charging":
            charging = " and actively charging"
        elif battery_status == "Idle":
            charging = " with battery idle"
        elif battery_status == "Discharging":
            charging = " but discharging (check connections)"

        level = ""
        if include_details and isinstance(battery.get("percentage"), (int, float)):
            level = f" (Battery level: {battery['percentage']}%)"

        return f"docked{charging}{level}"
    else:
        if not include_details:
            return "not docked"
        if dock_visible is True:
            return "not docked. The dock is visible to the robot sensors."
        elif dock_visible is False:
            return "not docked. The dock is not visible to the robot sensors."
        else:
            return "not docked. Dock visibility is unknown."


def _verify_operation_result(expected_docked_state, operation, robot_state):
    """Verify dock/undock result."""
    time.sleep(1)  # Give a moment for status to update

    updated_state = robot_state.get_state()
    final_is_docked = updated_state.get("dock_status", {}).get("is_docked")

    if final_is_docked == expected_docked_state:
        web.add_robot_message(f"🤖 ✅ Robot successfully {operation}ed")
    else:
        status = _get_status_message(include_details=False)
        result_message = f"{operation.capitalize()} command completed, but robot is {status}. Please verify {operation}ing manually."
        web.add_robot_message(f"🤖 ⚠️ {result_message}")

    return result_message


@tool
def dock_robot() -> str:
    """
    Command the robot to return to its docking station and begin recharging.

    Returns:
        str: A message describing the result of the docking action, including success or failure.

    Sends the robot to search for and connect to its charging dock. Useful for charging, moving to a known position, or starting recharge when battery is low.
    """
    try:
        # Get current dock status information
        robot_state = get_robot_state()
        dock_visible, is_docked = _get_dock_status_info()

        # Check if already docked
        if is_docked:
            message = "Robot is already docked"
            web.add_robot_message(f"🤖 {message}")
            return message

        # Check dock visibility and warn if needed
        _check_dock_visibility_and_warn(dock_visible, "dock")

        # Execute docking action
        goal = Dock.Goal()
        if not _execute_dock_action(_get_dock_client(), goal, "Docking"):
            return "Dock goal was rejected"

        # Verify operation result
        return _verify_operation_result(True, "dock", robot_state)

    except Exception as e:
        error_message = f"Error while attempting to dock: {e}"
        web.add_robot_message(f"🤖 ❌ {error_message}")
        return error_message


@tool
def undock_robot() -> str:
    """
    Command the robot to undock from its charging station.

    Returns:
        str: A message describing the result of the undocking action, including success or failure.

    Commands the robot to disengage from its charging dock and back away to become available for other operations.
    """
    try:
        # Get current dock status information
        robot_state = get_robot_state()
        dock_visible, is_docked = _get_dock_status_info()

        # Check if robot is already undocked
        if not is_docked:
            message = "Robot is already undocked"
            web.add_robot_message(f"🤖 {message}")
            return message

        # Execute undocking action
        goal = Undock.Goal()
        if not _execute_dock_action(_get_undock_client(), goal, "Undocking"):
            return "Undock goal was rejected"

        # Verify operation result
        return _verify_operation_result(False, "undock", robot_state)

    except Exception as e:
        error_message = f"Error while attempting to undock: {e}"
        web.add_robot_message(f"🤖 ❌ {error_message}")
        return error_message


@tool
def check_dock_status() -> str:
    """
    Check if the robot is currently docked at its charging station.

    Returns:
        str: A message indicating whether the robot is docked and charging, or not docked.

    Queries the robot's docking state and battery status to determine if it is docked and whether it is charging.
    """
    try:
        status = _get_status_message(include_details=True)
        return f"The robot is currently {status}."
    except Exception as e:
        return f"Error checking dock status: {e}"
