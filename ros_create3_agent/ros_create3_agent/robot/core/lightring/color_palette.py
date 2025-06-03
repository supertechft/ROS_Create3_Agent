"""
Color palette definitions for Create 3 robot lightring
https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Code/Package/example_package/pub_lightring.py
"""

import random
from typing import List
from irobot_create_msgs.msg import LedColor


class ColorPalette:
    """
    Color palette class providing predefined colors and utility functions
    for Create 3 robot lightring control.
    """

    # Color definitions as RGB tuples
    COLOR_DEFINITIONS = {
        "red": (255, 0, 0),
        "orange": (255, 165, 0),
        "green": (0, 255, 0),
        "lime": (50, 205, 50),
        "blue": (0, 0, 255),
        "yellow": (255, 255, 0),
        "pink": (255, 0, 255),
        "cyan": (0, 255, 255),
        "purple": (127, 0, 255),
        "white": (255, 255, 255),
        "grey": (189, 189, 189),
        "gray": (189, 189, 189),  # Alternative spelling
        "black": (0, 0, 0),
        "off": (0, 0, 0),
    }

    def __init__(self):
        """Initialize common colors for easy access."""
        for color_name, (r, g, b) in self.COLOR_DEFINITIONS.items():
            setattr(self, color_name, LedColor(red=r, green=g, blue=b))

    def get_color_by_name(self, color_name: str) -> LedColor:
        """
        Get a color by its name.

        Args:
            color_name: Name of the color (case-insensitive)

        Returns:
            LedColor object for the specified color

        Raises:
            ValueError: If color name is not found
        """
        color_name = color_name.lower().strip()

        if color_name not in self.COLOR_DEFINITIONS:
            available_colors = ", ".join(sorted(self.COLOR_DEFINITIONS.keys()))
            raise ValueError(
                f"Unknown color '{color_name}'. Available colors: {available_colors}"
            )

        return getattr(self, color_name)

    def create_color_from_rgb(self, red: int, green: int, blue: int) -> LedColor:
        """
        Create a color from RGB values.

        Args:
            red: Red component (0-255)
            green: Green component (0-255)
            blue: Blue component (0-255)

        Returns:
            LedColor object

        Raises:
            ValueError: If RGB values are out of range
        """
        if not all(0 <= val <= 255 for val in [red, green, blue]):
            raise ValueError("RGB values must be between 0 and 255")

        return LedColor(red=red, green=green, blue=blue)

    def get_random_color(self) -> LedColor:
        """
        Get a random color from the predefined palette.

        Returns:
            Random LedColor object
        """
        color_name = random.choice(list(self.COLOR_DEFINITIONS.keys()))
        return getattr(self, color_name)

    def get_random_pattern(self, num_leds: int = 6) -> List[LedColor]:
        """
        Generate a random color pattern for the lightring.

        Args:
            num_leds: Number of LEDs in the ring (default: 6)

        Returns:
            List of LedColor objects for each LED
        """
        return [self.get_random_color() for _ in range(num_leds)]

    def get_available_colors(self) -> List[str]:
        """
        Get list of available color names.

        Returns:
            List of color names
        """
        return list(self.COLOR_DEFINITIONS.keys())
