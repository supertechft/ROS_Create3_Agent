"""
Audio module for Create 3 robot
Contains predefined tunes and audio utilities
"""

from .tunes import PREDEFINED_TUNES
from .actions import (
    validate_audio_parameters,
    create_audio_notes,
    execute_audio_sequence,
)
