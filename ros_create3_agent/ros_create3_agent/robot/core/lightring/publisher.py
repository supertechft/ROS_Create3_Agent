"""
Lightring publisher for Create 3 robot
"""

import time
from typing import List, Union, Optional
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from irobot_create_msgs.msg import LightringLeds, LedColor

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
            # Clean up publisher
            if hasattr(self, "lights_publisher"):
                self.lights_publisher.destroy()

        except Exception as e:
            self.get_logger().debug(f"Error during node cleanup: {e}")
        finally:
            # Call parent destroy
            super().destroy_node()
