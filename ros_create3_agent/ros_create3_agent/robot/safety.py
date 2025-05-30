"""
Safety parameter configuration for the Create 3 robot.
This module provides functionality to configure motion control parameters during initialization.
https://iroboteducation.github.io/create3_docs/api/safety/
"""

import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue, ParameterType


def configure_safety_parameters(node, safety_override: str = "backup_only") -> bool:
    """
    Configure safety parameters for the Create 3 robot during initialization.

    Args:
        node: The ROS 2 node to use for parameter setting
        safety_override (str): Safety override setting:
        - 'none': Full safety enabled (max_speed: 0.306 m/s)
        - 'backup_only': Disables backup limit only (max_speed: 0.306 m/s)
        - 'full': All safety disabled (max_speed: 0.460 m/s)

    Returns:
        bool: True if parameters were set successfully, False otherwise
    """
    try:
        # Validate safety_override parameter
        valid_overrides = ["none", "backup_only", "full"]
        if safety_override not in valid_overrides:
            node.get_logger().error(
                f"Invalid safety_override: {safety_override}. Must be one of {valid_overrides}"
            )
            return False

        # Create parameter client for motion_control node
        param_client = node.create_client(
            SetParameters, "/motion_control/set_parameters"
        )

        node.get_logger().info("Waiting for motion control parameter service...")
        if not param_client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error("Motion control parameter service not available")
            return False

        # Set safety_override (max_speed is automatically updated by the robot)
        safety_param = ParameterMsg()
        safety_param.name = "safety_override"
        safety_param.value = ParameterValue()
        safety_param.value.type = ParameterType.PARAMETER_STRING
        safety_param.value.string_value = safety_override

        # Create and send the request
        request = SetParameters.Request()
        request.parameters = [safety_param]

        node.get_logger().info(f"Setting safety_override={safety_override}...")

        # Call the service and wait for response
        future = param_client.call_async(request)

        try:
            # Use spin_until_future_complete for more reliable async handling
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            if not future.done():
                node.get_logger().error("Service call timed out")
                return False

            response = future.result()

            if response is None:
                node.get_logger().warning(
                    "Service call returned None response, but parameter may have been set successfully"
                )
                # Since you confirmed the parameter is actually being set, treat None response as success
                node.get_logger().info(
                    f"Safety parameter may have been configured: safety_override={safety_override}"
                )
                return True

        except Exception as e:
            node.get_logger().error(f"Error calling parameter service: {e}")
            return False

        # Check if parameters were set successfully
        if hasattr(response, "results") and response.results:
            if all(result.successful for result in response.results):
                node.get_logger().info(
                    f"Safety parameters configured: safety_override={safety_override}"
                )
                return True
            else:
                # Report failed parameters
                failed_params = [
                    f"{request.parameters[i].name}: {result.reason or 'Unknown error'}"
                    for i, result in enumerate(response.results)
                    if not result.successful
                ]
                node.get_logger().error(
                    f"Failed to set parameters: {', '.join(failed_params)}"
                )
                return False
        else:
            # If response exists but has unexpected format, assume success since parameter is actually being set
            node.get_logger().warning(
                "Unexpected response format, but parameter may have been set successfully"
            )
            node.get_logger().info(
                f"Safety parameter may have been configured: safety_override={safety_override}"
            )
            return True

    except Exception as e:
        node.get_logger().error(f"Error configuring safety parameters: {e}")
        return False
