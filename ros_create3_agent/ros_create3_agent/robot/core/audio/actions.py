"""
Audio action implementation for the Create 3 robot.
This module contains the core audio playback logic and validation.
https://github.com/tuftsceeo/Tufts_Create3_Examples/blob/main/Code/Package/example_package/combined_audio_bump.py
https://iroboteducation.github.io/create3_docs/api/ui/
"""

import random
from typing import List, Tuple, Union, Optional, Dict, Any

from irobot_create_msgs.action import AudioNoteSequence
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration
from ros_create3_agent.utils.ros_threading import spin_until_complete_in_executor
from ros_create3_agent.web import app as web

from .tunes import PREDEFINED_TUNES


def convert_dict_frequencies_to_tuples(
    frequencies: List[Dict[str, Any]],
) -> Tuple[bool, str, Optional[List[Tuple[int, Union[int, float]]]]]:
    """
    Convert frequencies from dict format to tuple format.

    Args:
        frequencies: List of dicts with 'frequency' and 'duration' keys

    Returns:
        Tuple of (is_valid, error_message, tuple_frequencies)
    """
    if not isinstance(frequencies, list):
        return False, "Error: Frequencies must be a list of dictionaries", None

    tuple_frequencies = []
    for i, note in enumerate(frequencies):
        if not isinstance(note, dict):
            return (
                False,
                f"Error: Entry {i} must be a dictionary with 'frequency' and 'duration'",
                None,
            )
        if "frequency" not in note or "duration" not in note:
            return (
                False,
                f"Error: Entry {i} must have 'frequency' and 'duration' keys",
                None,
            )
        tuple_frequencies.append((note["frequency"], note["duration"]))

    return True, "", tuple_frequencies


def validate_audio_parameters(
    frequencies: Optional[
        Union[List[Tuple[int, Union[int, float]]], List[Dict[str, Any]]]
    ] = None,
    tune: Optional[str] = None,
) -> Tuple[bool, str, Optional[List[Tuple[int, int]]]]:
    """
    Validate audio parameters and return processed frequencies.
    Supports both dict and tuple formats for frequencies.

    Args:
        frequencies: List of (frequency_hz, duration) tuples or dicts with 'frequency' and 'duration'
        tune: Predefined tune name

    Returns:
        Tuple of (is_valid, error_message, validated_frequencies)
    """
    # Convert dict format to tuple format if needed
    if (
        frequencies is not None
        and len(frequencies) > 0
        and isinstance(frequencies[0], dict)
    ):
        is_valid, error_message, tuple_frequencies = convert_dict_frequencies_to_tuples(
            frequencies
        )
        if not is_valid:
            return False, error_message, None
        frequencies = tuple_frequencies
        web.add_robot_message(
            f"🎵 Playing custom audio sequence with {len(frequencies)} notes!"
        )

    # Parameter validation
    if frequencies is not None and tune is not None:
        return (
            False,
            "Error: Cannot specify both frequencies and tune. Please provide only one.",
            None,
        )

    if frequencies is None and tune is None:
        tune = "random"

    # Handle predefined tunes
    if tune is not None:
        if not isinstance(tune, str):
            return (
                False,
                f"Error: Tune must be a string, got {type(tune).__name__}",
                None,
            )

        tune = tune.lower().strip()
        if tune == "random":
            tune = random.choice(list(PREDEFINED_TUNES.keys()))
        elif tune not in PREDEFINED_TUNES:
            available = ", ".join(PREDEFINED_TUNES.keys())
            return (
                False,
                f"Error: Unknown tune '{tune}'. Available tunes: random, {available}",
                None,
            )

        frequencies = PREDEFINED_TUNES[tune]
        web.add_robot_message(
            f"🎵 Time to play '{tune}'! Get ready for some musical magic!"
        )

    # Validate frequencies list
    if not isinstance(frequencies, list):
        return (
            False,
            "Error: Frequencies must be a list of (frequency, duration) tuples",
            None,
        )

    if len(frequencies) == 0:
        return False, "Error: Cannot play audio with empty frequency list", None

    # Validate individual frequency entries and calculate total duration
    total_duration_microseconds = 0
    validated_notes = []

    for i, entry in enumerate(frequencies):
        if not isinstance(entry, tuple) or len(entry) != 2:
            return (
                False,
                f"Error: Entry {i} must be a tuple of (frequency, duration)",
                None,
            )

        freq, duration = entry

        # Validate frequency
        if not isinstance(freq, int):
            return (
                False,
                f"Error: Frequency at position {i} must be an integer, got {type(freq).__name__}",
                None,
            )
        if freq != 0 and (freq < 50 or freq > 1200):  # 0 is for REST notes
            return (
                False,
                f"Error: Frequency {freq} at position {i} is out of range (50-1200 Hz). Use 0 for rest notes.",
                None,
            )

        # Validate and convert duration
        if isinstance(duration, float):
            duration_microseconds = int(
                duration * 1_000_000
            )  # Convert seconds to microseconds
        elif isinstance(duration, int):
            duration_microseconds = duration  # Assume already in microseconds
        else:
            return (
                False,
                f"Error: Duration at position {i} must be int (microseconds) or float (seconds)",
                None,
            )

        if duration_microseconds <= 0:
            return False, f"Error: Duration at position {i} must be positive", None

        total_duration_microseconds += duration_microseconds
        validated_notes.append((freq, duration_microseconds))

    # Check total duration limit (30 seconds = 30,000,000 microseconds)
    max_duration_microseconds = 30_000_000
    if total_duration_microseconds > max_duration_microseconds:
        total_seconds = total_duration_microseconds / 1_000_000
        return (
            False,
            f"Error: Total audio duration ({total_seconds:.2f}s) exceeds 30 second limit",
            None,
        )

    return True, "", validated_notes


def create_audio_notes(validated_notes: List[Tuple[int, int]]) -> List[AudioNote]:
    """
    Create AudioNote messages from validated frequency/duration tuples.

    Args:
        validated_notes: List of (frequency, duration_microseconds) tuples

    Returns:
        List of AudioNote messages
    """
    audio_notes = []
    for freq, duration_microseconds in validated_notes:
        note = AudioNote()
        note.frequency = freq
        note.max_runtime = Duration()
        note.max_runtime.sec = duration_microseconds // 1_000_000
        note.max_runtime.nanosec = (duration_microseconds % 1_000_000) * 1000
        audio_notes.append(note)

    return audio_notes


def execute_audio_sequence(
    audio_client,
    node,
    audio_notes: List[AudioNote],
    total_duration_microseconds: int,
    tune: Optional[str] = None,
) -> str:
    """
    Execute audio sequence and return result message.

    Args:
        audio_client: AudioNoteSequence action client
        node: ROS node
        audio_notes: List of AudioNote messages
        total_duration_microseconds: Total duration in microseconds
        tune: Tune name if playing a predefined tune

    Returns:
        Result message string
    """
    # Create audio note vector
    note_vector = AudioNoteVector()
    note_vector.notes = audio_notes
    note_vector.append = False  # Overwrite any currently playing notes

    # Create goal
    goal = AudioNoteSequence.Goal()
    goal.iterations = 1
    goal.note_sequence = note_vector

    # Send goal and wait for result
    node.get_logger().info(f"Playing audio sequence with {len(audio_notes)} notes")

    try:
        # Wait for action to complete with timeout
        timeout_seconds = (total_duration_microseconds / 1_000_000) + 5.0

        # Send goal and wait for result using spin_until_complete_in_executor
        future = audio_client.send_goal_async(goal)
        spin_until_complete_in_executor(node, future).result(timeout=timeout_seconds)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return "Error: Audio goal was rejected by the robot or no goal handle was returned"
        result_future = goal_handle.get_result_async()
        spin_until_complete_in_executor(node, result_future).result(
            timeout=timeout_seconds
        )
        result = result_future.result()
        if result.result.complete:
            duration_played = result.result.runtime.sec + (
                result.result.runtime.nanosec / 1_000_000_000
            )
            if tune:
                return f"🎵 Finished playing '{tune}'! That was {duration_played:.2f} seconds of musical bliss!"
            else:
                return f"🎵 Audio sequence complete! Played {len(audio_notes)} notes in {duration_played:.2f} seconds."
        else:
            return f"⚠️ Audio playback was interrupted after {result.result.iterations_played} iterations"
    except Exception as e:
        return f"Error during audio playback: {e}"
