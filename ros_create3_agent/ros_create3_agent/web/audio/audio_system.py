"""
Audio system for text-to-speech functionality in the ROS Create3 Agent.
Uses prerecorded audio files for the most important robot messages.
"""

import os
import threading
import queue
from typing import Dict, Optional
import re

try:
    from playsound import playsound

    PLAYSOUND_AVAILABLE = True
except ImportError:
    PLAYSOUND_AVAILABLE = False


class AudioSystem:
    """Handles audio playback for robot messages."""

    def __init__(self, audio_dir: str = "speech_files"):
        """
        Initialize the audio system.

        Args:
            audio_dir: Directory containing prerecorded audio files
        """
        # Initialize audio system
        self.enabled = True
        self.audio_dir = audio_dir
        self.audio_queue = queue.Queue()
        self.is_playing = False
        self.current_thread = None

        # Create audio directory if it doesn't exist
        os.makedirs(audio_dir, exist_ok=True)

        # Define the 13 most important message categories with their audio files
        self.priority_messages = {
            # Movement messages
            "moving": {
                "pattern": r"Moving .* (forward|backward)",
                "audio_file": "moving.m4a",
            },
            "rotating": {
                "pattern": r"Rotating .* (clockwise|counterclockwise)",
                "audio_file": "rotating.m4a",
            },
            "navigating": {
                "pattern": r"Navigating to position",
                "audio_file": "navigating.m4a",
            },
            # Docking messages
            # "docking": {
            #     "pattern": r"docking\.\.\.",
            #     "audio_file": "docking.m4a",
            #     "sample_text": "Docking"
            # },
            "undocking": {
                "pattern": r"undocking",
                "audio_file": "undocking.m4a",
            },
            # Safety and hazard messages (2 priority messages)
            "hazard_detected": {
                "pattern": r"Cannot .* due to hazards",
                "audio_file": "hazards.m4a",
            },
            # "bumped_into_something": {
            #     "pattern": r"Ouch! I bumped into something",
            #     "audio_file": "bumped.m4a",
            #     "sample_text": "Ouch! I bumped into something"
            # },
            # Battery and system messages (2 priority messages)
            "picked_up": {
                "pattern": r"I'm being picked up!",
                "audio_file": "picked_up.m4a",
            },
            # Entertainment messages (2 priority messages)
            "time_to_dance": {
                "pattern": r"Time to dance|Watch me spin|Get ready for some twirls|It's.*time!",
                "audio_file": "dance.m4a",
            },
            # "playing_music": {
            #     "pattern": r"🎵 Time to play|🎵 Playing custom audio",
            #     "audio_file": "playing_music.m4a",
            #     "sample_text": "Time to play some music!"
            # },
            # Completion message (1 priority message)
            "task_completed": {
                "pattern": r"Finished (moving|rotating|navigating|driving)",
                "audio_file": "task_completed.m4a",
            },
        }

        # Start audio processing thread
        self._start_audio_thread()

    def _start_audio_thread(self):
        """Start the background audio processing thread."""

        def audio_worker():
            while True:
                try:
                    audio_file = self.audio_queue.get(timeout=1)
                    if audio_file is None:  # Shutdown signal
                        break
                    self._play_audio_file(audio_file)
                    self.audio_queue.task_done()
                except queue.Empty:
                    continue
                except Exception as e:
                    raise RuntimeError(f"Error in audio worker: {e}")

        self.current_thread = threading.Thread(target=audio_worker, daemon=True)
        self.current_thread.start()

    def _play_audio_file(self, audio_file: str):
        """Play an audio file using playsound."""
        if not self.enabled:
            return

        if not PLAYSOUND_AVAILABLE:
            raise RuntimeError(f"Cannot play {audio_file}: playsound not available")

        try:
            self.is_playing = True
            audio_path = os.path.join(self.audio_dir, audio_file)

            if not os.path.exists(audio_path):
                raise FileNotFoundError(f"Audio file not found: {audio_path}")

            # Use playsound for cross-platform audio playback
            playsound(audio_path, block=True)

        except Exception as e:
            raise RuntimeError(f"Error playing audio file {audio_file}: {e}")
        finally:
            self.is_playing = False

    def should_play_audio(self, message: str) -> Optional[str]:
        """
        Check if a message should have audio and return the audio file.

        Args:
            message: The robot message to check

        Returns:
            Audio file name if message should have audio, None otherwise
        """
        message_lower = message.lower()

        for category, config in self.priority_messages.items():
            pattern = config["pattern"]
            if re.search(pattern, message, re.IGNORECASE):
                return config["audio_file"]

        return None

    def play_message_audio(self, message: str):
        """
        Play audio for a message if it matches a priority pattern.

        Args:
            message: The robot message to potentially play audio for
        """
        if not self.enabled:
            return

        audio_file = self.should_play_audio(message)
        if audio_file:
            # Add to queue for background playback
            try:
                self.audio_queue.put_nowait(audio_file)
            except queue.Full:
                raise RuntimeError("Audio queue is full, skipping audio playback")

    def enable(self):
        """Enable audio playback."""
        self.enabled = True

    def disable(self):
        """Disable audio playback."""
        self.enabled = False

    def shutdown(self):
        """Shutdown the audio system."""
        try:
            self.audio_queue.put_nowait(None)  # Shutdown signal
            if self.current_thread and self.current_thread.is_alive():
                self.current_thread.join(timeout=2)
        except Exception as e:
            print(f"Error during audio system shutdown: {e}")


# Global audio system instance
_audio_system = None


def get_audio_system() -> AudioSystem:
    """Get the global audio system instance."""
    global _audio_system
    if _audio_system is None:
        # Use absolute path relative to the web module pointing to speech_files
        audio_dir = os.path.join(os.path.dirname(__file__), "speech_files")
        _audio_system = AudioSystem(audio_dir)
    return _audio_system


def play_audio_for_message(message: str):
    """
    Play audio for a robot message if it matches a priority pattern.

    Args:
        message: The robot message to potentially play audio for
    """
    audio_system = get_audio_system()
    audio_system.play_message_audio(message)
