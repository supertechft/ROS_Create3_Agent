from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

"""
https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html#writing-the-launch-file
"""


def generate_launch_description():
    # Use a variable for the launch argument name
    use_sim_arg = "use_simulator"
    use_sim_lc = LaunchConfiguration(use_sim_arg)

    actions = [
        # Declare launch argument for simulator usage
        DeclareLaunchArgument(
            use_sim_arg,
            default_value="false",
            description="Launch the Create 3 simulator",
        ),
        # Include the Create 3 simulator launch file if use_simulator is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("irobot_create_gazebo_bringup"),
                            "launch",
                            "create3_gazebo.launch.py",
                        ]
                    )
                ]
            ),
            condition=IfCondition(use_sim_lc),
        ),
        # Launch the ROS Create 3 agent
        Node(
            package="ros_create3_agent",
            executable="agent",
            name="ros_create3_agent",
            output="screen",
            parameters=[
                {
                    "use_simulator": use_sim_lc,
                }
            ],
        ),
    ]

    return LaunchDescription(actions)
