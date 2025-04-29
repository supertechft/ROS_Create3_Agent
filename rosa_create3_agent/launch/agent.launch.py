from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

"""
ROS 2 uses python
https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html#writing-the-launch-file
"""


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "streaming",
                default_value="true",
                description="Enable streaming response mode",
            ),
            Node(
                package="create3_agent",
                executable="create3_agent.py",
                name="create3_rosa",
                output="screen",
                parameters=[{"streaming": LaunchConfiguration("streaming")}],
            ),
        ]
    )
