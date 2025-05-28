"""
Utilities for executing ROS operations and background tasks using thread pools.
This module provides functions to prevent ROS operations from blocking the main thread.
"""

import concurrent.futures
import rclpy

# Shared thread pool executor for ROS commands (like spin_until_future_complete)
_ros_executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

# Shared thread pool executor for general async tasks (LLM processing, etc.)
_general_executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)


def spin_until_complete_in_executor(node, future):
    """
    Run rclpy.spin_until_future_complete in a background thread.

    Args:
        node: The ROS node to use for spinning
        future: The future to wait for completion

    Returns:
        A Future object that can be used to wait for the result
    """
    return _ros_executor.submit(rclpy.spin_until_future_complete, node, future)


def run_in_executor(func, *args, **kwargs):
    """
    Run any function in a background thread.

    Args:
        func: The function to execute
        *args: Positional arguments to pass to the function
        **kwargs: Keyword arguments to pass to the function

    Returns:
        A Future object that can be used to wait for the result
    """
    return _general_executor.submit(func, *args, **kwargs)


def shutdown_executors():
    """
    Shutdown all thread pool executors gracefully.
    Should be called during application shutdown.
    """
    _ros_executor.shutdown(wait=True)
    _general_executor.shutdown(wait=True)
