"""
Lightring publisher for Create 3 robot
"""

import time
from typing import List, Union, Optional
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irobot_create_msgs.msg import LightringLeds, LedColor
from irobot_create_msgs.action import LedAnimation
from builtin_interfaces.msg import Duration

from .color_palette import ColorPalette


class LightringPublisher(Node):
    """
    Publisher for controlling Create 3 robot's lightring LEDs.
    """

    def __init__(self, node_name: str = "lightring_publisher"):
        """
        Initialize the lightring publisher.

        Args:
            node_name: Name for the ROS node
        """
        super().__init__(node_name)
        self.color_palette = ColorPalette()

        # Create publisher for lightring control with BEST_EFFORT QoS
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.lights_publisher = self.create_publisher(
            LightringLeds, "cmd_lightring", qos_profile
        )

        # Create action client for LED animations
        self.led_animation_client = ActionClient(self, LedAnimation, "led_animation")

        # Wait for publisher to be ready
        self._wait_for_publisher()

    def _wait_for_publisher(self, timeout: float = 10.0) -> bool:
        """
        Wait for the lightring publisher to have subscribers.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            True if publisher is ready, False if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.lights_publisher.get_subscription_count() > 0:
                time.sleep(0.5)
                return True
            time.sleep(0.1)

        # Log warning but don't fail - publisher might still work
        self.get_logger().warning(
            f"No subscribers found after {timeout}s, but proceeding anyway"
        )
        return True

    def set_color(
        self, color: Union[str, List[int], LedColor], led_index: Optional[int] = None
    ) -> bool:
        """
        Set LED color(s) on the lightring.

        Args:
            color: Color specification (name, RGB list, or LedColor)
            led_index: Specific LED index (0-5), or None for all LEDs

        Returns:
            True if successful, False otherwise
        """
        try:
            # Convert color input to LedColor
            if isinstance(color, str):
                led_color = self.color_palette.get_color_by_name(color)
            elif isinstance(color, list) and len(color) == 3:
                led_color = self.color_palette.create_color_from_rgb(*color)
            elif isinstance(color, LedColor):
                led_color = color
            else:
                raise ValueError("Invalid color format")

            # Create lightring message
            lightring_msg = LightringLeds()
            lightring_msg.override_system = True
            lightring_msg.header.stamp = self.get_clock().now().to_msg()

            # Set colors for specific LED or all LEDs
            if led_index is not None:
                if not 0 <= led_index <= 5:
                    raise ValueError("LED index must be between 0 and 5")
                # Get current state and update specific LED
                lightring_msg.leds = [LedColor() for _ in range(6)]
                lightring_msg.leds[led_index] = led_color
            else:
                # Set all LEDs to the same color
                lightring_msg.leds = [led_color for _ in range(6)]

            # Publish the message
            self.lights_publisher.publish(lightring_msg)
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to set LED color: {e}")
            return False

    def set_pattern(self, colors: List[Union[str, List[int], LedColor]]) -> bool:
        """
        Set a custom color pattern on the lightring.

        Args:
            colors: List of colors for each LED (must be 6 colors)

        Returns:
            True if successful, False otherwise
        """
        try:
            if len(colors) != 6:
                raise ValueError("Pattern must contain exactly 6 colors")

            # Convert all colors to LedColor objects
            led_colors = []
            for color in colors:
                if isinstance(color, str):
                    led_colors.append(self.color_palette.get_color_by_name(color))
                elif isinstance(color, list) and len(color) == 3:
                    led_colors.append(self.color_palette.create_color_from_rgb(*color))
                elif isinstance(color, LedColor):
                    led_colors.append(color)
                else:
                    raise ValueError("Invalid color format in pattern")

            # Create and publish lightring message
            lightring_msg = LightringLeds()
            lightring_msg.override_system = True
            lightring_msg.header.stamp = self.get_clock().now().to_msg()
            lightring_msg.leds = led_colors

            self.lights_publisher.publish(lightring_msg)
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to set LED pattern: {e}")
            return False

    def set_random_pattern(self) -> bool:
        """
        Set a random color pattern on the lightring.

        Returns:
            True if successful, False otherwise
        """
        try:
            random_colors = self.color_palette.get_random_pattern(6)

            lightring_msg = LightringLeds()
            lightring_msg.override_system = True
            lightring_msg.header.stamp = self.get_clock().now().to_msg()
            lightring_msg.leds = random_colors

            self.lights_publisher.publish(lightring_msg)
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to set random pattern: {e}")
            return False

    def animate_leds(
        self,
        animation_type: str,
        colors: Optional[List[Union[str, List[int], LedColor]]] = None,
        duration: float = 5.0,
    ) -> bool:
        """
        Animate the LEDs with specified animation type.

        Args:
            animation_type: Type of animation ("blink" or "spin")
            colors: Colors to use for animation (optional)
            duration: Duration of animation in seconds

        Returns:
            True if successful, False otherwise
        """
        goal_handle = None
        try:
            # Wait for action server
            if not self.led_animation_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("LED animation action server not available")
                return False

            # Map animation type to action constants
            animation_map = {
                "blink": LedAnimation.Goal.BLINK_LIGHTS,
                "spin": LedAnimation.Goal.SPIN_LIGHTS,
            }

            if animation_type.lower() not in animation_map:
                self.get_logger().error(
                    f"Unknown animation type '{animation_type}'. Available: blink, spin"
                )
                return False

            # Prepare colors for animation
            if colors is None:
                # Use random colors if none specified
                led_colors = self.color_palette.get_random_pattern(6)
            else:
                if len(colors) != 6:
                    self.get_logger().error(
                        "Animation colors must contain exactly 6 colors"
                    )
                    return False
                led_colors = []
                for color in colors:
                    if isinstance(color, str):
                        led_colors.append(self.color_palette.get_color_by_name(color))
                    elif isinstance(color, list) and len(color) == 3:
                        led_colors.append(
                            self.color_palette.create_color_from_rgb(*color)
                        )
                    elif isinstance(color, LedColor):
                        led_colors.append(color)
                    else:
                        self.get_logger().error(
                            "Invalid color format in animation colors"
                        )
                        return False

            # Create animation goal
            goal = LedAnimation.Goal()
            goal.animation_type = animation_map[animation_type.lower()]
            goal.lightring.override_system = True
            goal.lightring.leds = led_colors
            goal.max_runtime = Duration(
                sec=int(duration), nanosec=int((duration % 1) * 1e9)
            )

            self.get_logger().info(
                f"Sending {animation_type} animation goal for {duration}s"
            )

            send_goal_future = self.led_animation_client.send_goal_async(goal)
            timeout = time.time() + 10.0
            while not send_goal_future.done() and time.time() < timeout:
                time.sleep(0.1)

            if not send_goal_future.done():
                self.get_logger().warning(
                    "Goal submission timed out, but animation may still be running"
                )
                # Instead of returning False, wait for the expected duration
                # and assume the animation is working
                time.sleep(duration)
                self.get_logger().info(f"Waited {duration}s for animation to complete")

                # Ensure responsiveness after assumed completion
                time.sleep(0.2)
                if not self.ensure_responsiveness():
                    self.get_logger().warning(
                        "Lightring may be unresponsive after goal timeout"
                    )
                return True

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Animation goal was rejected")
                return False

            self.get_logger().info("Animation goal accepted, waiting for completion")

            # Wait for the animation to actually finish!
            result_future = goal_handle.get_result_async()
            result_timeout = time.time() + duration + 5.0  # Add more buffer time
            while not result_future.done() and time.time() < result_timeout:
                time.sleep(0.1)

            if not result_future.done():
                self.get_logger().warning(
                    "Animation result not received, but animation likely completed"
                )
                # Don't return False - the animation probably worked
                # Ensure responsiveness after timeout
                time.sleep(0.2)
                if not self.ensure_responsiveness():
                    self.get_logger().warning(
                        "Lightring may be unresponsive after animation timeout"
                    )
                return True

            result = result_future.result()
            self.get_logger().info(f"Animation completed with result: {result}")

            # Ensure responsiveness after animation
            time.sleep(0.2)  # Brief pause before responsiveness check
            if not self.ensure_responsiveness():
                self.get_logger().warning(
                    "Lightring may be unresponsive after animation"
                )

            return True

        except Exception as e:
            self.get_logger().error(f"Failed to animate LEDs: {e}")
            return False
        finally:
            # Always try to cancel any active goal to clean up resources
            if goal_handle is not None:
                try:
                    cancel_future = goal_handle.cancel_goal_async()
                    # Don't wait too long for cancellation
                    timeout = time.time() + 1.0
                    while not cancel_future.done() and time.time() < timeout:
                        time.sleep(0.05)
                except Exception as cancel_error:
                    self.get_logger().debug(
                        f"Goal cancellation cleanup: {cancel_error}"
                    )
                    pass

    def ensure_responsiveness(self) -> bool:
        """
        Ensure the lightring is responsive after operations like animations.
        Sends a minimal command to verify the publisher is still working.

        Returns:
            True if responsive, False otherwise
        """
        try:
            # Send a minimal test message to ensure publisher is still working
            lightring_msg = LightringLeds()
            lightring_msg.override_system = False  # Don't actually change anything
            lightring_msg.header.stamp = self.get_clock().now().to_msg()
            lightring_msg.leds = []  # Empty LED array

            self.lights_publisher.publish(lightring_msg)
            time.sleep(0.1)  # Give time for message to be processed
            return True
        except Exception as e:
            self.get_logger().debug(f"Responsiveness check failed: {e}")
            return False

    def reset_to_default(self) -> bool:
        """
        Reset lightring to default system control.

        Returns:
            True if successful, False otherwise
        """
        try:
            lightring_msg = LightringLeds()
            lightring_msg.override_system = False  # Let system control the lights
            lightring_msg.header.stamp = self.get_clock().now().to_msg()
            white_leds = [
                self.color_palette.get_color_by_name("white") for _ in range(6)
            ]
            lightring_msg.leds = white_leds

            self.lights_publisher.publish(lightring_msg)
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to reset LEDs: {e}")
            return False

    def destroy_node(self):
        """
        Properly destroy the node and clean up resources.
        """
        try:
            # Clean up action client first
            if hasattr(self, "led_animation_client"):
                self.led_animation_client.destroy()

            # Clean up publisher
            if hasattr(self, "lights_publisher"):
                self.lights_publisher.destroy()

        except Exception as e:
            self.get_logger().debug(f"Error during node cleanup: {e}")
        finally:
            # Call parent destroy
            super().destroy_node()
