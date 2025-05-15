"""
Centralized logging configuration for the ROS Create 3 Agent.
Provides utilities to control logging verbosity across all components.
"""

import os
import logging
import inspect

# Base package name - the root of all our modules
BASE_PACKAGE_NAME = "ros_create3_agent"

# Internal modules that should use ROS logging when available
ROS_MODULES = [
    BASE_PACKAGE_NAME,
    "agent",
    "web",
]

# Groupings of loggers
EXTERNAL_LOGGERS = [
    "httpx",
    "openai",
    "langchain",
    "werkzeug",
    "huggingface_hub",
    "rosa",
    "rclpy",
    "ros",
]
INTERNAL_LOGGERS = ["root", BASE_PACKAGE_NAME]

# Mapping of human-readable log levels to logging constants
LOG_LEVELS = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}

# Global ROS node reference
_ros_node = None


def set_ros_node(node):
    """Set the ROS node for ROS logging functions.

    Args:
        node: The ROS node instance
    """
    global _ros_node
    _ros_node = node


def get_ros_logger():
    """Get the ROS node logger if available.

    Returns:
        The ROS node logger or None if not set
    """
    if _ros_node is not None:
        return _ros_node.get_logger()
    return None


def configure_logging():
    """Configure Python standard logging system with basic settings.

    This should be called only once at application startup (in agent.py).
    Individual modules should use get_logger() to obtain loggers.
    """
    # Configure basic logging format only if not already configured
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )


def get_logger(module_name=None):
    """Get a logger for the specified module or for the calling module if not specified.

    Args:
        module_name: Optional name of the module to get a logger for

    Returns:
        A logger for the module (standard Python logger or ROS logger)
    """
    # If module_name not provided, get it from the caller
    if module_name is None:
        module_name = inspect.stack()[1].frame.f_globals["__name__"]

    # For ROS components, if a ROS node is available, return the ROS logger directly
    if _ros_node is not None and (
        module_name.startswith(BASE_PACKAGE_NAME) or module_name in ROS_MODULES
    ):
        return _ros_node.get_logger()

    # Otherwise return a standard Python logger
    return logging.getLogger(module_name)


def set_loggers(loggers, level):
    """Set specified Python loggers to the given level.

    Args:
        loggers: A list of logger names, 'all' for all common loggers,
                'external' for external libraries, 'internal' for internal modules,
                or a single logger name as a string
        level: Level name ("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL")
              or a logging level constant
    """
    if isinstance(level, str):
        level = LOG_LEVELS.get(level.upper(), logging.INFO)

    # Convert single logger to list
    if isinstance(loggers, str):
        if loggers == "all":
            target_loggers = EXTERNAL_LOGGERS + INTERNAL_LOGGERS
        elif loggers == "external":
            target_loggers = EXTERNAL_LOGGERS
        elif loggers == "internal":
            target_loggers = INTERNAL_LOGGERS
        else:
            target_loggers = [loggers]
    else:
        target_loggers = loggers

    # Configure the loggers
    for logger_name in target_loggers:
        if logger_name == "root":
            logging.getLogger().setLevel(level)
        else:
            logging.getLogger(logger_name).setLevel(level)

    # Special case for ROS logging
    if "ros" in target_loggers or loggers == "all" or loggers == "external":
        if level >= logging.WARNING:
            os.environ["ROS_QUIET_LOGGING"] = "1"
        else:
            os.environ["ROS_QUIET_LOGGING"] = "0"
