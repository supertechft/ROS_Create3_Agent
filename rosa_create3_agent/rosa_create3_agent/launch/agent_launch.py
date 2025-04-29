from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
import os

def start_agent(context):
    return ExecuteProcess(
        cmd=["ros2", "run", "rosa_create3_agent", "agent_node"],
        output="screen"
    )

def generate_launch_description():
    return LaunchDescription([
        # 1. Simulation
        ExecuteProcess(
            cmd=[
                "ros2", "launch",
                "irobot_create_gz_bringup", "create3_gz.launch.py"
            ],
            output="screen"
        ),
        # 2. ROSA agent (after a short delay)
        OpaqueFunction(function=start_agent),
    ])
