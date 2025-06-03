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
from .lightring import LightringPublisher


@tool
def dance(pattern: str = "random") -> str:
    """
    Perform a dance routine with the robot.

    Args:
        pattern: Dance pattern name (random, circle, spin, party).
    Returns:
        Status or error message.
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
        node_name = "rosa_dance_publisher"
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


def _get_lightring_publisher() -> LightringPublisher:
    """Get a fixed-name lightring publisher instance."""
    node_name = "rosa_lightring_control"
    return LightringPublisher(node_name)


@tool
def change_lightring_color(
    color: Optional[str] = None,
    rgb: Optional[List[int]] = None,
    led_index: Optional[int] = None,
    reset: bool = False,
) -> str:
    """
    Set or reset the robot's lightring color.

    Args:
        color: Named color for all or one LED.
        rgb: RGB list for all or one LED (overrides color).
        led_index: LED index (0-5) or all if None.
            0=4 o'clock, 1=2 o'clock, 2=12 o'clock,
            3=10 o'clock, 4=8 o'clock, 5=6 o'clock
        reset: If True, reset to system default.
    Returns:
        Status or error message.
    """
    robot_state_manager = get_robot_state()
    if not robot_state_manager:
        return "Error: Unable to access robot state. Robot may not be connected"

    try:
        publisher = _get_lightring_publisher()
        executor = SingleThreadedExecutor()
        executor.add_node(publisher)

        # Wait for publisher to be ready
        for _ in range(10):
            executor.spin_once(timeout_sec=0.1)

        if reset:
            publisher.get_logger().info("Resetting lightring to default system control")
            success = publisher.reset_to_default()
            if success:
                web.add_robot_message(
                    "My lights are back to normal! The system is controlling them now."
                )
                return "Successfully reset lightring to system default control"
            else:
                return "Warning: Reset command sent but could not verify success"

        if rgb is not None:
            if not isinstance(rgb, list) or len(rgb) != 3:
                return "Error: RGB must be a list of 3 values [red, green, blue]"
            if not all(isinstance(val, int) and 0 <= val <= 255 for val in rgb):
                return "Error: RGB values must be integers between 0 and 255"

            publisher.get_logger().info(
                f"Changing lightring color to RGB {rgb}"
                + (f" (LED {led_index})" if led_index is not None else "")
            )
            success = publisher.set_color(rgb, led_index)
            if success:
                if led_index is not None:
                    return f"Set LED {led_index} to RGB {rgb}"
                else:
                    return f"Set all LEDs to RGB {rgb}"
            else:
                return f"Failed to set LEDs to RGB {rgb}"

        if color is not None:
            publisher.get_logger().info(
                f"Changing lightring color to {color}"
                + (f" (LED {led_index})" if led_index is not None else "")
            )
            try:
                success = publisher.set_color(color, led_index)
                if success:
                    if led_index is not None:
                        return f"Set LED {led_index} to {color}"
                    else:
                        return f"Set all LEDs to {color}"
                else:
                    return f"Failed to set LEDs to {color}"
            except ValueError as e:
                return str(e)

        publisher.get_logger().info("Changing lightring to random color pattern")
        success = publisher.set_random_pattern()
        if success:
            return "Set lightring to random color pattern"
        else:
            return "Failed to set random color pattern"

    except Exception as e:
        return f"Error: Failed to control lightring: {e}"

    finally:
        try:
            executor.remove_node(publisher)
        except Exception as e:
            publisher.get_logger().debug(f"Error removing node from executor: {e}")
        try:
            publisher.destroy_node()
        except Exception as e:
            if hasattr(publisher, "get_logger"):
                publisher.get_logger().debug(f"Error destroying node: {e}")
        try:
            executor.shutdown()
        except Exception:
            pass
