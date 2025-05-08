from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable

"""
ROS 2 uses python
https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html#writing-the-launch-file
"""


def generate_launch_description():
    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "use_simulator",
                default_value="false",
                description="Launch the Create 3 simulator",
            ),
            # Set environment variable for simulator usage
            SetEnvironmentVariable(
                name="USE_SIMULATOR",
                value=LaunchConfiguration("use_simulator"),
            ),
            # Launch the ROSA Create 3 agent
            Node(
                package="rosa_create3_agent",
                executable="agent",
                name="create3_rosa",
                output="screen",
                parameters=[{"use_simulator": LaunchConfiguration("use_simulator")}],
            ),
        ]
    )
