"""
User interface tools for Create 3 robot
Including dance sequences, light controls, and audio interactions
"""

import time
from typing import List, Optional
from langchain.agents import tool
import rclpy
from rclpy.executors import SingleThreadedExecutor

from ..robot_state import get_robot_state
from ...utils.ros_threading import run_in_executor
from .sensing import check_hazards
from ros_create3_agent.web import app as web
from .dance import (
    DanceChoreographer,
    DanceCommandPublisher,
    DANCE_PATTERNS,
    FinishedDance,
)
from .lightring import LightringPublisher, ColorPalette


@tool
def dance(pattern: str = "random") -> str:
    """
    Perform a dance routine with synchronized movement and LED patterns.

    Args:
        pattern: Dance pattern to perform. Options:
            "random" (default): Picks a random dance
            "circle": Circular motion with blue lights (9s)
            "spin": Spinning with rainbow colors (7s)
            "party": Complex choreography with varied movements (22s)

    Returns:
        Status message about the dance performance or error

    Robot performs safety checks and stops if hazards detected.
    """
    import random

    # Validate the pattern input
    if not isinstance(pattern, str):
        return f"Error: Pattern must be a string, got {type(pattern).__name__}"
    pattern = pattern.lower().strip()
    if pattern == "random":
        pattern = random.choice(list(DANCE_PATTERNS))
    elif pattern not in DANCE_PATTERNS:
        available = ", ".join(DANCE_PATTERNS)
        return f"Error: Unknown dance pattern '{pattern}'. Available patterns: random, {available}"

    duration_limit = 30.0  # Set a maximum duration limit for the dance
    robot_state_manager = get_robot_state()
    if not robot_state_manager:
        return "Error: Unable to access robot state. Robot may not be connected"

    # Check for hazards before starting the dance
    robot_state = robot_state_manager.get_state()
    hazard_result = check_hazards.invoke({"include_ir_sensors": True})
    if "No hazards detected" not in hazard_result:
        return f"Error: Cannot perform dance - {hazard_result}"

    start_messages = {
        "circle": "Watch me spin in perfect circles! Time to show off my smooth moves!",
        "spin": "Get ready for some twirls and spins! I hope you don't get dizzy watching!",
        "party": "It's party time! I'm about to bust out my best robot dance moves!",
    }
    web.add_robot_message(
        start_messages.get(
            pattern, f"Time to dance! I'll show you my {pattern} routine!"
        )
    )

    if robot_state.get("dock_status", {}).get("is_docked"):
        return "Error: Cannot perform dance while robot is docked. Please undock first"
    hazards = robot_state.get("hazards", [])
    if hazards:
        names = ", ".join(h.get("type", "unknown") for h in hazards)
        return f"Error: Cannot perform dance with active hazards: {names}"
    battery = robot_state.get("battery", {}).get("percentage", 0)
    if battery < 20:
        return f"Error: Battery too low for dance ({battery}%). Charge to at least 20% first"

    try:
        # Check if the dance pattern duration exceeds the limit
        dance_sequence = DANCE_PATTERNS[pattern]
        expected_duration = (
            max(step[0] for step in dance_sequence if isinstance(step[0], (int, float)))
            + 1.0
        )
        if expected_duration > duration_limit:
            return f"Error: Selected pattern '{pattern}' duration ({expected_duration:.1f}s) exceeds limit ({duration_limit:.1f}s)"

        choreographer = DanceChoreographer(dance_sequence)
        # Use unique node name with timestamp and random component to avoid conflicts
        import random

        timestamp = int(time.time() * 1000000)
        random_suffix = random.randint(1000, 9999)
        node_name = f"rosa_dance_publisher_{timestamp}_{random_suffix}"
        publisher = None
        executor = None

        try:
            publisher = DanceCommandPublisher(choreographer, node_name)
            publisher.get_logger().info(f"Starting dance routine: {pattern}")
            executor = SingleThreadedExecutor()
            executor.add_node(publisher)
            dance_error = []

            # Start the dance choreographer
            def run_dance():
                try:
                    start_time = time.time()
                    while rclpy.ok():
                        executor.spin_once(timeout_sec=0.1)
                        if time.time() - start_time > duration_limit:
                            dance_error.append("Dance stopped due to duration limit")
                            break
                        hazard_check = check_hazards.invoke(
                            {"include_ir_sensors": True}
                        )
                        if "No hazards detected" not in hazard_check:
                            dance_error.append(
                                f"Dance stopped due to hazards: {hazard_check}"
                            )
                            break
                except FinishedDance:
                    pass
                except Exception as e:
                    dance_error.append(f"Dance error: {e}")

            dance_future = run_in_executor(run_dance)

            # Wait for the dance to complete or timeout
            try:
                dance_future.result(timeout=duration_limit + 5.0)
            except Exception as e:
                dance_error.append(f"Dance execution error: {e}")

        except Exception as e:
            dance_error = [f"Dance setup error: {e}"]
        finally:
            # Cleanup the publisher and executor
            if publisher is not None:
                try:
                    # Give any pending operations time to complete
                    time.sleep(0.2)
                    if executor is not None:
                        executor.remove_node(publisher)
                except Exception as e:
                    publisher.get_logger().debug(
                        f"Error removing node from executor: {e}"
                    )
                try:
                    publisher.destroy_node()
                except Exception as e:
                    if hasattr(publisher, "get_logger"):
                        publisher.get_logger().debug(f"Error destroying node: {e}")
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception as e:
                    pass

        # Reset lightring to default after dance using change_lightring_color tool
        try:
            reset_result = change_lightring_color.invoke({"reset": True})
            if "Successfully reset" in reset_result:
                web.add_robot_message("My lights are back to normal!")
        except Exception:
            pass  # Continue even if reset fails

        if dance_error:
            return f"Oops! Had to stop my {pattern} dance routine: {dance_error[0]}. Let's try again when it's safe!"
        dance_messages = {
            "circle": "I just showed off my perfect circles! Did you enjoy my smooth moves?",
            "spin": "Wheeee! That spin dance was dizzying but so much fun!",
            "party": "That party routine really got my circuits pumping! Ready for an encore?",
        }

        return dance_messages.get(
            pattern, f"Ta-da! My {pattern} dance is complete! How did I do?"
        )
    except Exception as e:
        return f"Oh no! I tripped on my dance moves: {e}"


@tool
def change_lightring_color(
    color: Optional[str] = None,
    rgb: Optional[List[int]] = None,
    led_index: Optional[int] = None,
    animation: str = "none",
    duration: float = 5.0,
    reset: bool = False,
) -> str:
    """
    Control the robot's LED lightring with colors, patterns, and animations.

    Args:
        color: Named color. If not provided, uses random pattern.
        rgb: RGB color values as [red, green, blue] where each value is 0-255.
             Takes precedence over named color if both provided.
        led_index: Specific LED to control (0-5). If None, controls all LEDs.
                  LED positions: 0=4 o'clock, 1=2 o'clock, 2=12 o'clock,
                  3=10 o'clock, 4=8 o'clock, 5=6 o'clock
        animation: Animation type - "none" (default), "blink", or "spin"
        duration: Duration for animations in seconds (default: 5.0)
        reset: If True, resets lightring to system default control

    Returns:
        Status message about the lightring control action or error

    Examples:
        - change_lightring_color(color="blue") - All LEDs blue
        - change_lightring_color(rgb=[255, 0, 0], led_index=2) - Top LED red
        - change_lightring_color(animation="blink", duration=3.0) - Random blink 3s
        - change_lightring_color(reset=True) - Reset to default
    """
    robot_state_manager = get_robot_state()
    if not robot_state_manager:
        return "Error: Unable to access robot state. Robot may not be connected"

    try:
        # Handle reset request using proper threading
        if reset:

            def execute_reset():
                # Use unique node name with timestamp, microsecond precision and random component to avoid conflicts
                import random

                timestamp = int(time.time() * 1000000)
                random_suffix = random.randint(1000, 9999)
                node_name = f"rosa_lightring_reset_{timestamp}_{random_suffix}"
                publisher = None
                executor = None

                try:
                    publisher = LightringPublisher(node_name)
                    publisher.get_logger().info(
                        f"Creating new LightringPublisher node: {node_name}"
                    )
                    executor = SingleThreadedExecutor()
                    executor.add_node(publisher)

                    # Wait for publisher to be ready
                    for _ in range(20):
                        executor.spin_once(timeout_sec=0.1)
                        time.sleep(0.1)

                    # Execute reset
                    success = publisher.reset_to_default()
                    return success
                except Exception as e:
                    if publisher is not None:
                        publisher.get_logger().error(f"Exception in execute_reset: {e}")
                    return False
                finally:
                    # Cleanup the publisher and executor
                    if publisher is not None:
                        try:
                            # Give time for any pending operations to complete
                            time.sleep(0.5)
                            if executor is not None:
                                executor.remove_node(publisher)
                        except Exception as e:
                            publisher.get_logger().debug(
                                f"Error removing node from executor: {e}"
                            )
                        try:
                            publisher.destroy_node()
                        except Exception as e:
                            if hasattr(publisher, "get_logger"):
                                publisher.get_logger().debug(
                                    f"Error destroying node: {e}"
                                )

                        # Additional delay after node destruction to ensure cleanup
                        time.sleep(0.3)

                    if executor is not None:
                        try:
                            executor.shutdown()
                        except Exception as e:
                            pass

            reset_future = run_in_executor(execute_reset)
            try:
                success = reset_future.result(timeout=5.0)
                if success:
                    web.add_robot_message(
                        "My lights are back to normal! The system is controlling them now."
                    )
                    return "Successfully reset lightring to system default control"
                else:
                    return "Warning: Reset command sent but could not verify success"
            except Exception as e:
                return f"Error during lightring reset: {e}"

        # Validate parameters
        if rgb is not None:
            if not isinstance(rgb, list) or len(rgb) != 3:
                return "Error: RGB must be a list of 3 values [red, green, blue]"
            if not all(isinstance(val, int) and 0 <= val <= 255 for val in rgb):
                return "Error: RGB values must be integers between 0 and 255"

        if led_index is not None:
            if not isinstance(led_index, int) or not 0 <= led_index <= 5:
                return "Error: LED index must be an integer between 0 and 5"

        if not isinstance(animation, str):
            return "Error: Animation must be a string"
        animation = animation.lower().strip()
        if animation not in ["none", "blink", "spin"]:
            return "Error: Animation must be 'none', 'blink', or 'spin'"

        if not isinstance(duration, (int, float)) or duration <= 0:
            return "Error: Duration must be a positive number"

        # Execute lightring control using proper threading
        def execute_lightring_control():
            # Use unique node name with timestamp, microsecond precision and random component to avoid conflicts
            import random

            timestamp = int(time.time() * 1000000)  # Microsecond precision
            random_suffix = random.randint(1000, 9999)
            node_name = f"rosa_lightring_control_{timestamp}_{random_suffix}"
            publisher = None
            executor = None
            color_palette = ColorPalette()

            try:
                publisher = LightringPublisher(node_name)
                publisher.get_logger().info(
                    f"Creating new LightringPublisher node: {node_name}"
                )
                executor = SingleThreadedExecutor()
                executor.add_node(publisher)

                # Wait longer for publisher to be ready and stable
                for _ in range(25):  # Increased from 20 to 25 iterations
                    executor.spin_once(timeout_sec=0.1)
                    time.sleep(0.1)  # Increased delay for better stability

                if animation != "none":
                    # Handle animations
                    led_colors = None
                    if rgb is not None:
                        led_colors = [
                            color_palette.create_color_from_rgb(*rgb) for _ in range(6)
                        ]
                    elif color is not None:
                        try:
                            led_colors = [
                                color_palette.get_color_by_name(color) for _ in range(6)
                            ]
                        except Exception as e:
                            publisher.get_logger().error(f"Invalid color name: {color}")
                            return False, f"Invalid color name: {color}"
                    else:
                        led_colors = None  # Will trigger random pattern in animate_leds

                    publisher.get_logger().info(
                        f"Starting {animation} animation for {duration}s"
                        + (
                            f" with {color}"
                            if color
                            else f" with RGB {rgb}" if rgb else " with random colors"
                        )
                    )
                    success = publisher.animate_leds(animation, led_colors, duration)
                    if success:
                        # Add a small delay after animation completes to ensure proper cleanup
                        time.sleep(0.5)
                        color_desc = (
                            f"with {color}"
                            if color
                            else f"with RGB {rgb}" if rgb else "with random colors"
                        )
                        return (
                            True,
                            f"Completed {animation} animation {color_desc} for {duration} seconds",
                        )
                    else:
                        return False, f"Failed to start {animation} animation"

                elif rgb is not None:
                    # Handle RGB color
                    publisher.get_logger().info(
                        f"Changing lightring color to RGB {rgb}"
                        + (f" (LED {led_index})" if led_index is not None else "")
                    )
                    success = publisher.set_color(rgb, led_index)
                    if success:
                        if led_index is not None:
                            return True, f"Set LED {led_index} to RGB {rgb}"
                        else:
                            return True, f"Set all LEDs to RGB {rgb}"
                    else:
                        return False, f"Failed to set LEDs to RGB {rgb}"

                elif color is not None:
                    # Handle named color
                    publisher.get_logger().info(
                        f"Changing lightring color to {color}"
                        + (f" (LED {led_index})" if led_index is not None else "")
                    )
                    try:
                        success = publisher.set_color(color, led_index)
                        if success:
                            if led_index is not None:
                                return True, f"Set LED {led_index} to {color}"
                            else:
                                return True, f"Set all LEDs to {color}"
                        else:
                            return False, f"Failed to set LEDs to {color}"
                    except ValueError as e:
                        return False, str(e)

                else:
                    # No color specified, use random pattern
                    publisher.get_logger().info(
                        "Changing lightring to random color pattern"
                    )
                    success = publisher.set_random_pattern()
                    if success:
                        return True, "Set lightring to random color pattern"
                    else:
                        return False, "Failed to set random color pattern"

            except Exception as e:
                if publisher is not None:
                    publisher.get_logger().error(f"Error controlling lights: {e}")
                return False, f"Error controlling lights: {e}"
            finally:
                # Cleanup the publisher and executor
                if publisher is not None:
                    try:
                        # Give time for any pending operations to complete
                        time.sleep(0.5)
                        if executor is not None:
                            executor.remove_node(publisher)
                    except Exception as e:
                        publisher.get_logger().debug(
                            f"Error removing node from executor: {e}"
                        )
                    try:
                        publisher.destroy_node()
                    except Exception as e:
                        if hasattr(publisher, "get_logger"):
                            publisher.get_logger().debug(f"Error destroying node: {e}")

                    # Additional delay after node destruction to ensure cleanup
                    time.sleep(0.3)

                if executor is not None:
                    try:
                        executor.shutdown()
                    except Exception as e:
                        pass

        control_future = run_in_executor(execute_lightring_control)

        try:
            # Use longer timeout for animations: goal submission (10s) + animation duration + result timeout (duration+5s) + buffer
            timeout = 25.0 if animation != "none" else 10.0
            success, action_description = control_future.result(timeout=timeout)

            if success:
                # Add robot message for successful operations
                if animation != "none":
                    web.add_robot_message(
                        f"Check out my lights! I'm doing a {animation} animation!"
                    )
                elif color or rgb:
                    color_name = color if color else f"RGB {rgb}"
                    web.add_robot_message(
                        f"Look at my beautiful {color_name} lights! How do they look?"
                    )
                else:
                    web.add_robot_message(
                        "I've got some colorful random lights going now! Pretty cool, right?"
                    )

                return f"Success: {action_description}"
            else:
                return f"Error: {action_description}"

        except Exception as e:
            return f"Error: Lightring control operation timed out or failed: {e}"

    except Exception as e:
        return f"Error: Failed to control lightring: {e}"
