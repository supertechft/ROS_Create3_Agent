"""
Dance choreographer classes for Create 3 robot
Based on iRobot Create 3 example
https://github.com/iRobotEducation/create3_examples/blob/humble/create3_examples_py/create3_examples_py/dance/dance_choreograph.py
"""

import math
from typing import List, Tuple, Union
from irobot_create_msgs.msg import LedColor
from ..lightring.color_palette import ColorPalette


class Move:
    """Class to tell the robot to move as part of dance sequence"""

    def __init__(self, x_m_s: float, theta_degrees_second: float):
        """
        Parameters
        ----------
        x_m_s : float
            The speed to drive the robot forward (positive) / backwards (negative) in m/s
        theta_degrees_second : float
            The speed to rotate the robot counter clockwise (positive) / clockwise (negative) in deg/s
        """
        self.x = x_m_s
        self.theta = math.radians(theta_degrees_second)


class Lights:
    """Class to tell the robot to set lightring lights as part of dance sequence"""

    def __init__(self, led_colors: List[LedColor]):
        """
        Parameters
        ----------
        led_colors : list of LedColor
            The list of 6 LedColors corresponding to the 6 LED lights on the lightring
        """
        if len(led_colors) != 6:
            raise ValueError("LED colors list must contain exactly 6 colors")
        self.led_colors = led_colors


class FinishedDance(Exception):
    """Exception to signal that the dance sequence has finished"""

    pass


class DanceChoreographer:
    """Class to manage a dance sequence, returning current actions to perform"""

    def __init__(
        self, dance_sequence: List[Tuple[float, Union[Move, Lights, FinishedDance]]]
    ):
        """
        Parameters
        ----------
        dance_sequence : list of (time, action) pairs
            The time is time since start_dance was called to initiate action,
            the action is one of the classes above [Move, Lights, FinishedDance]
        """
        self.dance_sequence = dance_sequence
        self.action_index = 0
        self.start_time = None

    def start_dance(self, time):
        """
        Parameters
        ----------
        time : rclpy::Time
            The ROS 2 time to mark the start of the sequence
        """
        self.start_time = time
        self.action_index = 0

    def get_next_actions(self, time):
        """
        Parameters
        ----------
        time : rclpy::Time
            The ROS 2 time to compare against start time to give
            actions that should be applied given how much time sequence has been running for
        """
        if self.start_time is None:
            return []
        time_into_dance = time - self.start_time
        time_into_dance_seconds = time_into_dance.nanoseconds / 1e9
        actions = []
        while (
            self.action_index < len(self.dance_sequence)
            and time_into_dance_seconds >= self.dance_sequence[self.action_index][0]
        ):
            actions.append(self.dance_sequence[self.action_index][1])
            self.action_index += 1
        return actions
