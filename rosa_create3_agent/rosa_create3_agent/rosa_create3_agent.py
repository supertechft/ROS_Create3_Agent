"""
ROSA Documentation
- https://github.com/nasa-jpl/rosa/wiki/Custom-Agents
- https://github.com/nasa-jpl/rosa/wiki/Developer-Documentation
"""

import os
import rclpy    # https://docs.ros.org/en/humble/p/rclpy/
from rclpy.node import Node
from rosa import ROSA

# Import local modules
from rosa_create3_agent import tools
from rosa_create3_agent.prompts import get_prompts
from rosa_create3_agent.llm import get_llm


"""ROS 2 node for Create 3 robot agent."""
class Create3AgentNode(Node):

    def __init__(self):
        
        # Configure ROS 2 logging
        if os.environ.get("ROS_QUIET_LOGGING") != "1":
            os.environ["ROS_QUIET_LOGGING"] = "1"  # Reduce ROS 2 log verbosity
        
        super().__init__("create3_agent")
        self.get_logger().info("Starting Create 3 ROSA Agent...")

        llm = get_llm()
        prompts = get_prompts()

        # Initialize ROSA
        self.rosa = ROSA(
            ros_version=2,
            llm=llm,
            tools=[tools],
            prompts=prompts,
            verbose=False,          # Set to False to reduce terminal output
            streaming=False,        # Set to False to reduce terminal output
            accumulate_chat_history=True,
        )

        self.running = True
        self.get_logger().info("Create 3 ROSA Agent initialized successfully!")


"""Main function to start the Create 3 ROSA agent."""
def main():
    try:
        rclpy.init()
        agent = Create3AgentNode()

        # Set up executor for ROS callbacks
        from rclpy.executors import SingleThreadedExecutor
        executor = SingleThreadedExecutor()
        executor.add_node(agent)
        agent.get_logger().info("ROS 2 node is running. Press Ctrl+C to stop.")
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
