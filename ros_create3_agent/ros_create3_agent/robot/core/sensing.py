"""
Sensing tools for the Create 3 robot.
Provides robust, LLM-friendly tools for battery and hazard checking.
All sensor data is accessed via robot_state.py, which manages ROS subscriptions.
https://iroboteducation.github.io/create3_docs/api/hazards/
https://iroboteducation.github.io/create3_docs/api/safety/
"""

from langchain.agents import tool
from ..robot_state import get_robot_state


@tool
def get_battery_status() -> str:
    """
    Get the current battery status of the robot, including charging state.

    Returns:
        str: A message describing the battery status, including charge percentage and charging state.

    Queries the robot's battery state and returns a summary. Warns if battery is low and not charging.
    """
    try:
        # Get battery state from robot state manager
        state = get_robot_state().get_state()
        battery = state.get("battery", {})
        battery_level = battery.get("level", "Unknown")
        status = battery.get("status", "Unknown")
        percentage = battery.get("percentage", "Unknown")

        response = f"Battery level: {battery_level}, Status: {status}"

        # Warn if battery is low (<20%) and not charging
        if (
            isinstance(percentage, (int, float))
            and percentage < 20
            and status != "Charging"
        ):
            response += f"\n ⚠️ Battery level is low at {percentage}%! Consider docking the robot soon."

        return response
    except Exception as e:
        return f"Error checking battery status: {e}"


@tool
def check_hazards(include_ir_sensors: bool = False) -> str:
    """
    Check for any hazards detected by the robot, optionally including proximity hazards.

    Args:
        include_ir_sensors (bool): Whether to include IR sensor proximity as hazards. Defaults to False.

    Returns:
        str: A message describing any detected hazards or confirming the path is clear.

    Combines hazards reported by the robot's sensors with any proximity hazards picked up by IR proximity sensors (intensity > 50) if include_ir_sensors is True.
    """
    try:
        # Get hazard and IR sensor state
        state = get_robot_state().get_state()
        hazards = state.get("hazards", [])
        ir_intensities = state.get("ir_intensities", {})

        proximity_threshold = 1000  # Threshold for proximity hazard
        proximity_hazards = []
        if include_ir_sensors:
            for sensor, value in ir_intensities.items():
                if value > proximity_threshold:
                    proximity_hazards.append(
                        {
                            "type": "OBJECT_PROXIMITY",
                            "description": f"Object detected very close by IR sensor: {sensor}",
                            "location": sensor,
                        }
                    )
        all_hazards = hazards + proximity_hazards
        if all_hazards:
            hazard_messages = []
            for hazard in all_hazards:
                desc = hazard.get("description", "Unknown hazard")
                loc = hazard.get("location")
                msg = f"- {desc}"
                if loc:
                    msg += f" (location: {loc})"
                hazard_messages.append(msg)
            result = "⚠️ Hazards detected:\n" + "\n".join(hazard_messages)
            result += (
                "\n\nPlease address these hazards before attempting to move the robot."
            )
            return result
        return "No hazards detected. The path is clear."
    except Exception as e:
        return f"Error checking hazards: {e}"


@tool
def get_imu_status() -> str:
    """
    Get the robot's IMU (Inertial Measurement Unit) sensor readings.

    Returns:
        str: A summary of the robot's orientation (quaternion), acceleration (x, y, z), and angular velocity (x, y, z).

    Provides access to the sensor_msgs/Imu topic data.
    """
    try:
        # Get IMU data from robot state manager
        state = get_robot_state().get_state()
        imu_data = state.get("imu", {})

        if not imu_data:
            return "IMU data is not currently available."

        orientation = imu_data.get("orientation", {})
        orientation_str = (
            f"Orientation (quaternion): x={orientation.get('x', 0):.3f}, "
            f"y={orientation.get('y', 0):.3f}, "
            f"z={orientation.get('z', 0):.3f}, "
            f"w={orientation.get('w', 0):.3f}"
        )

        linear_accel = imu_data.get("linear_acceleration", {})
        accel_str = (
            f"Linear acceleration (m/s²): x={linear_accel.get('x', 0):.3f}, "
            f"y={linear_accel.get('y', 0):.3f}, "
            f"z={linear_accel.get('z', 0):.3f}"
        )

        angular_vel = imu_data.get("angular_velocity", {})
        ang_vel_str = (
            f"Angular velocity (rad/s): x={angular_vel.get('x', 0):.3f}, "
            f"y={angular_vel.get('y', 0):.3f}, "
            f"z={angular_vel.get('z', 0):.3f}"
        )

        return f"IMU Status:\n{orientation_str}\n{accel_str}\n{ang_vel_str}"
    except Exception as e:
        return f"Error retrieving IMU status: {e}"


@tool
def get_kidnap_status() -> str:
    """
    Check if the robot has been 'kidnapped' (picked up or moved unexpectedly).

    Returns:
        str: A message indicating whether the robot is currently picked up.

    Reports the status from the KidnapStatus topic, which detects when the robot has been lifted off the ground.
    """
    try:
        # Get kidnap status from robot state manager
        state = get_robot_state().get_state()
        is_picked_up = state.get("is_picked_up", False)

        if is_picked_up:
            return (
                "The robot is currently being held off the ground (kidnapped). "
                "Please place the robot on a flat surface before attempting any movement commands."
            )
        else:
            return "The robot is properly positioned on the ground."
    except Exception as e:
        return f"Error checking kidnap status: {e}"


@tool
def get_odometry() -> str:
    """
    Get the robot's current odometry information (position and velocity).

    Returns:
        str: A message containing the robot's current position (x, y), orientation (quaternion) and velocity.

    Provides access to nav_msgs/Odometry data.
    """
    try:
        # Get odometry data from robot state manager
        state = get_robot_state().get_state()
        odom_data = state.get("odometry", {})

        if not odom_data:
            return "Odometry data is not currently available."

        position = odom_data.get("position", {})
        position_str = (
            f"Position: x={position.get('x', 0):.3f}m, "
            f"y={position.get('y', 0):.3f}m"
        )

        orientation = odom_data.get("orientation", {})
        orientation_str = (
            f"Orientation (quaternion): x={orientation.get('x', 0):.3f}, "
            f"y={orientation.get('y', 0):.3f}, "
            f"z={orientation.get('z', 0):.3f}, "
            f"w={orientation.get('w', 0):.3f}"
        )

        linear_vel = odom_data.get("linear_velocity", {})
        angular_vel = odom_data.get("angular_velocity", {})

        velocity_str = (
            f"Linear velocity: {linear_vel.get('x', 0):.3f} m/s, "
            f"Angular velocity: {angular_vel.get('z', 0):.3f} rad/s"
        )

        return (
            f"Odometry Information:\n{position_str}\n{orientation_str}\n{velocity_str}"
        )
    except Exception as e:
        return f"Error retrieving odometry data: {e}"


@tool
def get_stop_status() -> str:
    """
    Check if the robot is currently in a stopped state.

    Returns:
        str: A message indicating whether the robot is stopped.

    Reports if the robot has been stopped due to a safety trigger or other reasons.
    """
    try:
        # Get stop status from robot state manager
        state = get_robot_state().get_state()
        stop_status = state.get("stop_status", {})

        if not stop_status:
            return "Stop status information is not currently available."

        is_stopped = stop_status.get("is_stopped", False)

        if is_stopped:
            return "The robot is currently stopped."
        else:
            return "The robot is not stopped and is free to move."
    except Exception as e:
        return f"Error retrieving stop status: {e}"
