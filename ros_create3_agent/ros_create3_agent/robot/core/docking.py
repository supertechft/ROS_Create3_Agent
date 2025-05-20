"""
Docking operations for the Create 3 robot.
This module contains tools for docking, undocking, and checking dock status.
https://iroboteducation.github.io/create3_docs/api/docking/
"""

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


@tool
def dock_robot() -> str:
    """
    Command the robot to return to its docking station and begin recharging.

    Returns:
        str: A message describing the result of the docking action, including success or failure.

    Sends the robot to search for and connect to its charging dock. Useful for charging, moving to a known position, or starting recharge when battery is low.
    """
    try:
        goal = Dock.Goal()
        _get_node().get_logger().info("Docking...")
        web.add_robot_message("🤖 Attempting to dock...")
        future = _get_dock_client().send_goal_async(goal)
        rclpy.spin_until_future_complete(_get_node(), future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            web.add_robot_message("🤖 Dock goal was rejected")
            return "Dock goal was rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(_get_node(), result_future)
        result_message = "Robot successfully docked"
        web.add_robot_message(f"🤖 {result_message}")

        return result_message
    except Exception as e:
        error_message = f"Error while attempting to dock: {e}"
        web.add_robot_message(f"🤖 {error_message}")
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
        goal = Undock.Goal()
        _get_node().get_logger().info("Undocking...")
        web.add_robot_message("🤖 Attempting to undock...")
        future = _get_undock_client().send_goal_async(goal)
        rclpy.spin_until_future_complete(_get_node(), future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            web.add_robot_message("🤖 Undock goal was rejected")
            return "Undock goal was rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(_get_node(), result_future)
        result_message = "Robot successfully undocked"
        web.add_robot_message(f"🤖 {result_message}")

        return result_message
    except Exception as e:
        error_message = f"Error while attempting to undock: {e}"
        web.add_robot_message(f"🤖 {error_message}")
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
        robot_state = get_robot_state()
        state = robot_state.get_state()
        dock_status = state.get("dock_status", "Unknown")
        is_docked = dock_status == "Docked"

        battery = state.get("battery", {})
        battery_status = battery.get("status", "Unknown")
        battery_level = battery.get("percentage", "Unknown")

        if is_docked:
            charging_status = ""
            if battery_status == "Charging":
                charging_status = " and actively charging"
            elif battery_status == "Full":
                charging_status = " with a full battery"
            elif battery_status == "NotCharging":
                charging_status = " but not currently charging"

            battery_info = ""
            if isinstance(battery_level, (int, float)):
                battery_info = f" (Battery level: {battery_level}%)"

            return f"The robot is currently docked{charging_status}.{battery_info}"
        else:
            return "The robot is not docked."
    except Exception as e:
        return f"Error checking dock status: {e}"
