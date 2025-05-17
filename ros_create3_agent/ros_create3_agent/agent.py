"""
ROSA Documentation
- https://github.com/nasa-jpl/rosa/wiki/Custom-Agents
- https://github.com/nasa-jpl/rosa/wiki/Developer-Documentation
"""

import os
from pathlib import Path
import rclpy  # https://docs.ros.org/en/humble/p/rclpy/
from rclpy.node import Node
from rosa import ROSA


# Import local modules
from ros_create3_agent.robot import tools
from ros_create3_agent.web import app
from ros_create3_agent.llm.prompts import get_prompts
from ros_create3_agent.llm.llm import get_llm, get_HF_inference
from ros_create3_agent.logging import set_ros_node, configure_logging, get_logger


# Configure logging once at module level
configure_logging()
logger = get_logger(__name__)


# ROS 2 node for Create 3 robot agent
class Create3AgentNode(Node, ROSA):

    def __init__(self):
        # Initialize Node
        Node.__init__(self, "ros_create3_agent")

        # Set the ROS node for logging
        set_ros_node(self)
        logger.info("Starting Create 3 ROS Agent...")

        # Get ROSA arguments
        tools.initialize(self)
        llm = get_llm()
        hf_inference = get_HF_inference()
        audio_path = str(Path(__file__).parent / "data" / "audio.wav")
        prompts = get_prompts()

        # Initialize ROSA as superclass
        # https://github.com/nasa-jpl/rosa/blob/db554ad253a1637577ff552809f0cecf54e41ca3/src/rosa/rosa.py#L39
        ROSA.__init__(
            self,
            ros_version=2,
            llm=llm,
            inference=hf_inference,
            audio_path=audio_path,
            # tools=[tools], # Explicitly list all tools for clarity
            tools=[
                # Docking
                tools.dock_robot,
                tools.undock_robot,
                tools.check_dock_status,
                # Info
                tools.agent_intro,
                tools.get_help,
                tools.get_examples,
                tools.get_dock_info,
                tools.get_create3_specs,
                tools.get_create3_interface,
                # Movement
                tools.drive_distance,
                tools.rotate_angle,
                # Sensing
                tools.get_battery_status,
                tools.check_hazards,
                tools.get_imu_status,
                tools.get_kidnap_status,
                tools.get_odometry,
                tools.get_stop_status,
            ],
            prompts=prompts,
            verbose=False,  # Reduce terminal output
            streaming=False,  # Disable LLM streaming
            accumulate_chat_history=False,  # Manage chat history ourselves in app.py
            show_token_usage=True,  # Shows total API token usage and cost after exiting
        )

        self.running = True
        logger.info("Create 3 ROS Agent initialized successfully!")

    def initialize_web_interface(self):
        """Initialize the web interface for user interaction."""
        app.initialize(self, self)  # Pass self as both the node and the ROSA agent
        logger.info("Web interface started")


# Main function to start the Create 3 ROS agent
def main():
    try:
        # Set up logging at the start
        from ros_create3_agent.logging import set_loggers

        # Configure external libraries for quiet logging (WARNING level) and internal modules for INFO
        set_loggers("external", "WARNING")  # Set external libraries to WARNING
        set_loggers("internal", "INFO")  # Set our modules to INFO

        rclpy.init()
        agent = Create3AgentNode()
        agent.initialize_web_interface()

        # Set up executor for ROS callbacks
        from rclpy.executors import SingleThreadedExecutor

        executor = SingleThreadedExecutor()
        executor.add_node(agent)

        logger.info("ROS 2 node is running. Press Ctrl+C to stop.")
        while agent.running and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt, shutting down...")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        print("\nCleaning up and shutting down...")
        try:
            if "agent" in locals():
                agent.running = False
                agent.destroy_node()
        except Exception as e:
            print(f"Error during cleanup: {e}")

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error shutting down ROS: {e}")

        print("ROS 2 node shut down successfully.")
        return 0


if __name__ == "__main__":
    main()
