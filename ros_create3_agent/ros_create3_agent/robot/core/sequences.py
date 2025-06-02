"""
Dance sequence tools for Create 3 robot
"""

import time
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
        publisher = DanceCommandPublisher(choreographer, "rosa_dance_publisher")
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
                    hazard_check = check_hazards.invoke({"include_ir_sensors": True})
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
        try:
            publisher.destroy_node()
        except Exception:
            pass

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
