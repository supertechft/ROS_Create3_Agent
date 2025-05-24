"""
Docking operations for the Create 3 robot.
This module contains tools for docking, undocking, and checking dock status.
https://iroboteducation.github.io/create3_docs/api/docking/
"""

from langchain.agents import tool
from ros_create3_agent.web import app as web
from ..robot_state import get_robot_state
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import IrOpcode
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


# @tool
def dock_robot() -> str:
    """
    Command the robot to return to its docking station and begin recharging.

    Returns:
        str: A message describing the result of the docking action, including success or failure.

    Sends the robot to search for and connect to its charging dock. Useful for charging, moving to a known position, or starting recharge when battery is low.
    """
    pass
    # try:
    #     goal = Dock.Goal()
    #     _get_node().get_logger().info("Docking...")
    #     web.add_robot_message("🤖 Attempting to dock...")
    #     future = _get_dock_client().send_goal_async(goal)
    #     rclpy.spin_until_future_complete(_get_node(), future)
    #     goal_handle = future.result()

    #     if not goal_handle.accepted:
    #         web.add_robot_message("🤖 Dock goal was rejected")
    #         return "Dock goal was rejected"

    #     result_future = goal_handle.get_result_async()
    #     rclpy.spin_until_future_complete(_get_node(), result_future)
    #     # TODO: Check if the robot is actually docked
    #     result_message = "Robot successfully docked"
    #     web.add_robot_message(f"🤖 {result_message}")

    #     return result_message
    # except Exception as e:
    #     error_message = f"Error while attempting to dock: {e}"
    #     web.add_robot_message(f"🤖 {error_message}")
    #     return error_message


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
        dock_status = state.get("dock_status", {})
        
        # Handle both old string format and new dict format for backwards compatibility
        if isinstance(dock_status, str):
            is_docked = dock_status == "Docked"
        else:
            is_docked = dock_status.get("is_docked", False)

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


def _navigate_using_ir_opcodes() -> bool:
    """
    Intelligently navigate toward the dock using IR opcode signals.
    
    Returns:
        bool: True if navigation was successful and dock is now visible, False otherwise.
    """
    from ..movement import rotate_angle, drive_distance
    from geometry_msgs.msg import Twist
    import time
    
    max_attempts = 10
    attempt = 0
    
    web.add_robot_message("🤖 Using IR guidance to navigate toward dock...")
    _get_node().get_logger().info("Starting IR-guided navigation to dock")
    
    while attempt < max_attempts:
        attempt += 1
        
        # Get current robot state and IR opcode data
        robot_state = get_robot_state()
        state = robot_state.get_state()
        ir_opcode = state.get("ir_opcode", {})
        dock_status = state.get("dock_status", {})
        
        # Check if dock is now visible
        if dock_status.get("dock_visible", False):
            web.add_robot_message("🤖 Dock is now visible! Proceeding to dock...")
            return True
        
        # Check for any IR signals
        opcode = ir_opcode.get("opcode", 0)
        sensor = ir_opcode.get("sensor", 0)
        
        if opcode == 0:
            # No IR signal detected, try rotating to scan for signals
            web.add_robot_message(f"🤖 Scanning for dock signals... (attempt {attempt}/{max_attempts})")
            rotate_angle(45.0)  # Rotate 45 degrees to scan
            time.sleep(1)  # Wait a moment for new readings
            continue
        
        # Analyze the IR signal and determine navigation strategy
        if opcode == IrOpcode.CODE_IR_BUOY_GREEN:
            # Green buoy on left side of dock - turn right to center
            if sensor == IrOpcode.SENSOR_DIRECTIONAL_FRONT:
                web.add_robot_message("🤖 Green buoy detected ahead-left - turning right to center")
                rotate_angle(-10.0)  # Small adjustment right
            else:  # SENSOR_OMNI
                web.add_robot_message("🤖 Green buoy detected nearby - turning right to center")
                rotate_angle(-15.0)  # Larger adjustment for omni detection
            
        elif opcode == IrOpcode.CODE_IR_BUOY_RED:
            # Red buoy on right side of dock - turn left to center
            if sensor == IrOpcode.SENSOR_DIRECTIONAL_FRONT:
                web.add_robot_message("🤖 Red buoy detected ahead-right - turning left to center")
                rotate_angle(10.0)  # Small adjustment left
            else:  # SENSOR_OMNI
                web.add_robot_message("🤖 Red buoy detected nearby - turning left to center")
                rotate_angle(15.0)  # Larger adjustment for omni detection
            
        elif opcode == IrOpcode.CODE_IR_BUOY_BOTH:
            # Both buoys detected (yellow) - we're centered, move forward
            if sensor == IrOpcode.SENSOR_DIRECTIONAL_FRONT:
                web.add_robot_message("🤖 Centered on dock beacon - moving forward")
                drive_distance(0.3)  # Move forward 30cm when well-aligned
            else:  # SENSOR_OMNI
                web.add_robot_message("🤖 Dock beacon nearby - moving forward slowly")
                drive_distance(0.15)  # Move forward 15cm when close
            
        elif opcode == IrOpcode.CODE_IR_FORCE_FIELD:
            # Force field detected - within dock radius but not necessarily aligned
            web.add_robot_message("🤖 Detected dock force field - within dock radius, scanning for alignment")
            rotate_angle(20.0)  # Rotate to find proper alignment with buoys
            
        elif opcode == 173:  # Red + Green + Force Field (CODE_IR_BUOY_BOTH + CODE_IR_FORCE_FIELD)
            # Perfectly centered and very close to dock - final approach
            web.add_robot_message("🤖 Perfectly aligned with dock - final approach")
            if sensor == IrOpcode.SENSOR_DIRECTIONAL_FRONT:
                drive_distance(0.1)  # Move forward slowly for final docking
            else:
                drive_distance(0.05)  # Very small movement if omni sensor
            
        elif opcode in [IrOpcode.CODE_IR_EVAC_GREEN_FIELD, IrOpcode.CODE_IR_EVAC_RED_FIELD, IrOpcode.CODE_IR_EVAC_BOTH_FIELD]:
            # Evacuation field signals - move away slightly then reapproach
            web.add_robot_message("🤖 Detected evacuation field - adjusting approach")
            drive_distance(-0.1)  # Back up slightly
            time.sleep(0.5)
            
        elif opcode == IrOpcode.CODE_IR_VIRTUAL_WALL:
            # Virtual wall detected - turn around and search
            web.add_robot_message("🤖 Virtual wall detected - turning to find alternate path")
            rotate_angle(90.0)  # Turn 90 degrees to avoid wall
            
        else:
            # Unknown signal - continue scanning
            web.add_robot_message(f"🤖 Unknown IR signal ({opcode}) on sensor {sensor} - continuing search")
            rotate_angle(30.0)
        
        # Brief pause between navigation steps
        time.sleep(0.5)
    
    web.add_robot_message("🤖 Unable to locate dock using IR guidance")
    return False


@tool
def return_to_dock() -> str:
    """
    Navigate back to the docking station using IR guidance and dock the robot.
    
    Returns:
        str: A message describing the result of the return-to-dock operation.
    
    This function intelligently navigates using IR opcode signals when the dock
    is not immediately visible, then attempts to dock. Uses buoy signals (green/red)
    to center on the dock and force field signals for close approach.
    """
    try:
        # First check if we can detect the dock IR signals
        robot_state = get_robot_state()
        state = robot_state.get_state()
        dock_status = state.get("dock_status", {})
        
        # Check if already docked
        current_dock_status = dock_status.get("is_docked", False)
        if current_dock_status:
            already_docked_message = "Robot is already docked"
            web.add_robot_message(f"🤖 {already_docked_message}")
            return already_docked_message
        
        # Check if dock is immediately visible
        dock_visible = dock_status.get("dock_visible", False)
        
        if not dock_visible:
            # Check for any IR opcode signals
            ir_opcode = state.get("ir_opcode", {})
            opcode = ir_opcode.get("opcode", 0)
            
            # If we have IR signals, try intelligent navigation
            if opcode in [IrOpcode.CODE_IR_BUOY_GREEN, IrOpcode.CODE_IR_BUOY_RED, 
                         IrOpcode.CODE_IR_BUOY_BOTH, IrOpcode.CODE_IR_FORCE_FIELD,
                         IrOpcode.CODE_IR_EVAC_GREEN_FIELD, IrOpcode.CODE_IR_EVAC_RED_FIELD,
                         IrOpcode.CODE_IR_EVAC_BOTH_FIELD, 173]:
                
                navigation_success = _navigate_using_ir_opcodes()
                if not navigation_success:
                    warning_message = "IR-guided navigation failed. Attempting direct dock anyway."
                    web.add_robot_message(f"🤖 {warning_message}")
                    _get_node().get_logger().warn(warning_message)
            else:
                warning_message = "No dock IR signals detected. Attempting direct dock - success may be limited."
                web.add_robot_message(f"🤖 {warning_message}")
                _get_node().get_logger().warn(warning_message)
        
        # Proceed with docking action
        goal = Dock.Goal()
        _get_node().get_logger().info("Executing dock action...")
        web.add_robot_message("🤖 Initiating docking sequence...")
        
        future = _get_dock_client().send_goal_async(goal)
        rclpy.spin_until_future_complete(_get_node(), future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            rejected_message = "Dock goal was rejected"
            web.add_robot_message(f"🤖 {rejected_message}")
            return rejected_message

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(_get_node(), result_future)
        
        # Check the actual result
        result = result_future.result().result
        
        # Verify docking was successful by checking dock status
        updated_state = get_robot_state().get_state()
        final_dock_status = updated_state.get("dock_status", {})
        is_now_docked = final_dock_status.get("is_docked", False)
        
        if is_now_docked:
            success_message = "Robot successfully returned to dock and is now charging"
            web.add_robot_message(f"🤖 {success_message}")
            return success_message
        else:
            partial_success_message = "Docking action completed, but robot may not be properly docked. Please check dock status."
            web.add_robot_message(f"🤖 {partial_success_message}")
            return partial_success_message
            
    except Exception as e:
        error_message = f"Error while returning to dock: {e}"
        web.add_robot_message(f"🤖 {error_message}")
        return error_message
