"""
Centralized robot state management for the Create 3 robot.
This module handles all ROS topic subscriptions and maintains the current robot state.
https://iroboteducation.github.io/create3_docs/api/ros2/
"""

import threading
import time
from typing import Dict, Any, Optional, Callable
from threading import Lock
from rclpy.node import Node

# Import Create 3 messages
from irobot_create_msgs.msg import (
    HazardDetectionVector,
    DockStatus,
    KidnapStatus,
    IrIntensityVector,
    InterfaceButtons,
    StopStatus,
)
from sensor_msgs.msg import BatteryState, Imu
from nav_msgs.msg import Odometry


# Map of hazard types to descriptive strings
# https://github.com/iRobotEducation/irobot_create_msgs/blob/rolling/msg/HazardDetection.msg
HAZARD_TYPES = {
    0: "BACKUP_LIMIT",
    1: "BUMP",
    2: "CLIFF",
    3: "STALL",
    4: "WHEEL_DROP",
    5: "OBJECT_PROXIMITY",
}

# Human-readable hazard descriptions for UI and feedback
HAZARD_DESCRIPTIONS = {
    "BACKUP_LIMIT": "The robot has reached its backward movement safety limit",
    "BUMP": "The robot's bumper has detected contact with an obstacle",
    "CLIFF": "The robot has detected a cliff or drop-off",
    "STALL": "The robot's wheels are stalled against an obstacle",
    "WHEEL_DROP": "One or more of the robot's wheels are not on the ground",
    "OBJECT_PROXIMITY": "The robot has detected an object in close proximity",
}

# Map of IR proximity sensor positions
"""
https://github.com/iRobotEducation/create3_sim/blob/humble/irobot_create_common/irobot_create_description/urdf/create3.urdf.xacro#L188
"""
IR_SENSOR_NAMES = {
    0: "Front center left",
    1: "Front center right",
    2: "Front left",
    3: "Front right",
    4: "Left",
    5: "Right",
    6: "Side left",
}

# Map of cliff sensor positions
CLIFF_SENSOR_NAMES = {
    0: "Front left",
    1: "Front right",
    2: "Side left",
    3: "Side right",
}


# Singleton instance
_instance = None
_instance_lock = threading.Lock()


class RobotState:
    """
    Centralized manager for robot state.
    Subscribes to all relevant ROS topics and provides access to current robot state.
    """

    def __init__(self, node: Node):
        """Initialize the robot state manager.

        Args:
            node: The ROS 2 node to use for subscriptions
        """
        self.node = node

        # State data with locks for thread safety
        self._state_lock = Lock()
        self._state = {
            "battery": {
                "level": "Unknown",
                "percentage": None,
                "voltage": None,
                "current": None,
                "status": "Unknown",
            },
            "dock_status": {"is_docked": None, "dock_visible": None},
            "is_picked_up": None,
            "hazards": [],
            "ir_intensities": {},
            "cliff_intensities": {},
            "button_states": {
                "button_1": None,  # Left button on faceplate marked with 1 dot
                "button_power": None,  # Power button on faceplate marked with a power symbol
                "button_2": None,  # Right button on faceplate marked with 2 dots
            },
            "imu": {
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "odometry": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
            },
            "stop_status": {"is_stopped": None},
        }

        # Callback registration
        self._update_callbacks = []

        # Set up all the subscriptions
        self._setup_subscriptions()

        self.node.get_logger().info("Robot state manager initialized")

    def _setup_subscriptions(self):
        """Set up all the ROS topic subscriptions."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Hazard detection
        self.node.create_subscription(
            HazardDetectionVector,
            "hazard_detection",
            self._hazard_callback,
            qos_profile,
        )

        # Dock status
        self.node.create_subscription(
            DockStatus, "dock_status", self._dock_status_callback, qos_profile
        )

        # Battery state
        self.node.create_subscription(
            BatteryState, "battery_state", self._battery_callback, qos_profile
        )

        # Kidnap (picked up) status
        self.node.create_subscription(
            KidnapStatus, "kidnap_status", self._kidnap_callback, qos_profile
        )

        # IR intensity (proximity sensors)
        self.node.create_subscription(
            IrIntensityVector, "ir_intensity", self._ir_intensity_callback, qos_profile
        )

        # Cliff intensity
        #! cliff_intensity topic doesn't seem to be publishing in SIM
        self.node.create_subscription(
            IrIntensityVector,
            "cliff_intensity",
            self._cliff_intensity_callback,
            qos_profile,
        )

        # Interface buttons
        self.node.create_subscription(
            InterfaceButtons,
            "interface_buttons",
            self._interface_buttons_callback,
            qos_profile,
        )

        # IMU data
        self.node.create_subscription(Imu, "imu", self._imu_callback, qos_profile)

        # Odometry data
        self.node.create_subscription(
            Odometry, "odom", self._odometry_callback, qos_profile
        )

        # Stop status
        self.node.create_subscription(
            StopStatus, "stop_status", self._stop_status_callback, qos_profile
        )

    def _hazard_callback(self, msg: HazardDetectionVector) -> None:
        """Callback for hazard detection messages."""
        with self._state_lock:
            hazard_details = []
            current_time = time.time()
            for detection in msg.detections:

                # Convert numerical type to string representation
                hazard_name = HAZARD_TYPES.get(
                    detection.type, f"UNKNOWN_{detection.type}"
                )

                # Get detailed description
                description = HAZARD_DESCRIPTIONS.get(
                    hazard_name, f"Unknown hazard: {hazard_name}"
                )
                hazard_details.append(
                    {
                        "type": hazard_name,
                        "description": description,
                        "location": detection.header.frame_id,
                    }
                )

                # Send robot message for BUMP but not more than once every 10s
                if hazard_name == "BUMP":
                    last_bump_notify = getattr(self, "_last_notify_bump", 0)
                    if current_time - last_bump_notify > 10.0:  # 10 seconds
                        setattr(self, "_last_notify_bump", current_time)
                        try:
                            from ros_create3_agent.web.app import add_robot_message

                            add_robot_message("Ouch! I bumped into something.")
                        except Exception as e:
                            self.node.get_logger().error(
                                f"Error sending hazard message: {e}"
                            )

            self._state["hazards"] = hazard_details

        self._notify_update()

    def _dock_status_callback(self, msg: DockStatus) -> None:
        """Callback for dock status messages."""
        with self._state_lock:
            self._state["dock_status"] = {
                "is_docked": msg.is_docked,
                "dock_visible": msg.dock_visible,
            }

        self._notify_update()

    def _battery_callback(self, msg: BatteryState) -> None:
        """Callback for battery state messages."""
        with self._state_lock:
            percentage = int(msg.percentage * 100)

            # Notify if battery is low (<20%), but not more than once every 5 minutes
            if percentage < 20:
                current_time = time.time()
                last_low_battery_time = getattr(
                    self, "_last_low_battery_notification_time", 0
                )
                if current_time - last_low_battery_time > 300:  # 5 minutes
                    self._last_low_battery_notification_time = current_time
                    try:
                        from ros_create3_agent.web.app import add_robot_message

                        add_robot_message(
                            "⚠️ Battery low: " + str(percentage) + "% remaining!"
                        )
                    except Exception as e:
                        self.node.get_logger().error(
                            f"Error sending low battery message: {e}"
                        )

            # Determine battery status based on current flow
            current = msg.current
            if current < -0.05:
                charging_status = "Discharging"
            elif current > 0.05:
                charging_status = "Charging"
            else:
                charging_status = "Idle"

            # Store battery state information
            self._state["battery"] = {
                "level": f"{percentage}%",
                "percentage": percentage,
                "voltage": msg.voltage,
                "current": current,
                "status": charging_status,
            }

        self._notify_update()

    def _kidnap_callback(self, msg: KidnapStatus) -> None:
        """Callback for kidnap status messages (robot being picked up)."""
        with self._state_lock:
            was_picked_up = self._state["is_picked_up"]
            is_picked_up = msg.is_kidnapped
            self._state["is_picked_up"] = is_picked_up

            # Store the last notification time as a class attribute
            current_time = time.time()
            last_notification_time = getattr(self, "_last_pickup_notification_time", 0)

            # Determine if we should notify (once every 5 seconds)
            should_notify = (
                not was_picked_up
                and is_picked_up
                and (current_time - last_notification_time > 5.0)
            )

            if should_notify:
                self._last_pickup_notification_time = current_time
                try:
                    from ros_create3_agent.web.app import add_robot_message

                    add_robot_message("I'm being picked up!")
                except Exception as e:
                    self.node.get_logger().error(f"Error sending pickup message: {e}")

        self._notify_update()

    def _process_intensity_readings(
        self, msg: IrIntensityVector, sensor_names: Dict[int, str], state_key: str
    ) -> None:
        """Process intensity readings (IR or cliff).

        Args:
            msg: The intensity vector message
            sensor_names: Dictionary mapping indices to human-readable sensor names
            state_key: The state dictionary key to update
        """
        with self._state_lock:
            # Store only the named intensity values
            named_intensities = {}
            for i, reading in enumerate(msg.readings):
                # Add named version using the human-readable names
                sensor_name = sensor_names.get(i, f"Sensor {i}")
                named_intensities[sensor_name] = reading.value

            self._state[state_key] = named_intensities

        self._notify_update()

    # Callback for IR intensity messages (objects in proximity)
    def _ir_intensity_callback(self, msg: IrIntensityVector) -> None:
        """Callback for IR intensity messages (objects in proximity)."""
        self._process_intensity_readings(msg, IR_SENSOR_NAMES, "ir_intensities")

    def _cliff_intensity_callback(self, msg: IrIntensityVector) -> None:
        """Callback for cliff intensity messages (floor detection)."""
        self._process_intensity_readings(msg, CLIFF_SENSOR_NAMES, "cliff_intensities")

    def _interface_buttons_callback(self, msg: InterfaceButtons) -> None:
        """Callback for interface button messages."""
        with self._state_lock:
            # Get previous button states to detect changes
            prev_button_1 = self._state["button_states"]["button_1"]
            prev_button_power = self._state["button_states"]["button_power"]
            prev_button_2 = self._state["button_states"]["button_2"]

            self._state["button_states"] = {
                "button_1": msg.button_1.is_pressed,  # Left button on faceplate marked with 1 dot
                "button_power": msg.button_power.is_pressed,  # Power button on faceplate marked with a power symbol
                "button_2": msg.button_2.is_pressed,  # Right button on faceplate marked with 2 dots
            }

            # Check for button press events (transition from not pressed to pressed)
            # Otherwise, we will spam the web app with messages
            from ros_create3_agent.web.app import add_robot_message

            if not prev_button_1 and msg.button_1.is_pressed:
                try:
                    add_robot_message("Button 1 pressed")
                except Exception as e:
                    self.node.get_logger().error(f"Error sending button message: {e}")

            if not prev_button_power and msg.button_power.is_pressed:
                try:
                    add_robot_message("Button Power pressed")
                except Exception as e:
                    self.node.get_logger().error(f"Error sending button message: {e}")

            if not prev_button_2 and msg.button_2.is_pressed:
                try:
                    add_robot_message("Button 2 pressed")
                except Exception as e:
                    self.node.get_logger().error(f"Error sending button message: {e}")

        self._notify_update()

    def _imu_callback(self, msg: Imu) -> None:
        """Callback for IMU messages."""
        with self._state_lock:
            orientation = msg.orientation
            self._state["imu"]["orientation"] = {
                "x": orientation.x,
                "y": orientation.y,
                "z": orientation.z,
                "w": orientation.w,
            }

            angular_velocity = msg.angular_velocity
            self._state["imu"]["angular_velocity"] = {
                "x": angular_velocity.x,
                "y": angular_velocity.y,
                "z": angular_velocity.z,
            }

            linear_acceleration = msg.linear_acceleration
            self._state["imu"]["linear_acceleration"] = {
                "x": linear_acceleration.x,
                "y": linear_acceleration.y,
                "z": linear_acceleration.z,
            }

        self._notify_update()

    def _odometry_callback(self, msg: Odometry) -> None:
        """Callback for odometry messages."""
        with self._state_lock:
            position = msg.pose.pose.position
            self._state["odometry"]["position"] = {
                "x": position.x,
                "y": position.y,
                "z": position.z,
            }

            orientation = msg.pose.pose.orientation
            self._state["odometry"]["orientation"] = {
                "x": orientation.x,
                "y": orientation.y,
                "z": orientation.z,
                "w": orientation.w,
            }

            linear_velocity = msg.twist.twist.linear
            self._state["odometry"]["linear_velocity"] = {
                "x": linear_velocity.x,
                "y": linear_velocity.y,
                "z": linear_velocity.z,
            }

            angular_velocity = msg.twist.twist.angular
            self._state["odometry"]["angular_velocity"] = {
                "x": angular_velocity.x,
                "y": angular_velocity.y,
                "z": angular_velocity.z,
            }

        self._notify_update()

    def _stop_status_callback(self, msg: StopStatus) -> None:
        """Callback for stop status messages."""
        with self._state_lock:
            self._state["stop_status"] = {"is_stopped": msg.is_stopped}

        self._notify_update()

    def get_state(self) -> Dict[str, Any]:
        """Get the current robot state as a dictionary."""
        with self._state_lock:
            # Return a copy to avoid thread safety issues
            return dict(self._state)

    def update_state(self, updates: Dict[str, Any]) -> None:
        """Update the robot state with new values.

        Args:
            updates: Dictionary of state values to update
        """
        with self._state_lock:
            self._state.update(updates)

        self._notify_update()

    def register_update_callback(
        self, callback: Callable[[Dict[str, Any]], None]
    ) -> None:
        """Register a callback that will be called when the robot state changes.

        Args:
            callback: A function that takes the current state as an argument
        """
        self._update_callbacks.append(callback)

    def _notify_update(self) -> None:
        """Notify all registered callbacks that the state has been updated."""
        current_state = self.get_state()
        for callback in self._update_callbacks:
            try:
                callback(current_state)
            except Exception as e:
                self.node.get_logger().error(f"Error in state update callback: {e}")


def get_robot_state(node: Optional[Node] = None) -> RobotState:
    """Get the singleton RobotState instance.

    Args:
        node: The ROS 2 node to use (only needed on first call)

    Returns:
        The singleton RobotState instance

    Raises:
        ValueError: If node is not provided on first call
    """
    global _instance

    with _instance_lock:
        if _instance is None:
            if node is None:
                raise ValueError("Node must be provided when initializing RobotState")
            _instance = RobotState(node)

        return _instance
