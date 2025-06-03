"""
Predefined audio tunes for the Create 3 robot
Frequencies are in Hz, durations are in microseconds for precision
"""

# Musical note frequencies in Hz
NOTE_FREQUENCIES = {
    "C4": 262,
    "D4": 294,
    "E4": 330,
    "F4": 349,
    "G4": 392,
    "A4": 440,
    "B4": 494,
    "C5": 523,
    "D5": 587,
    "E5": 659,
    "F5": 698,
    "G5": 784,
    "A5": 880,
    "B5": 988,
    "REST": 0,  # Silent pause
}

# Duration constants in microseconds
QUARTER_NOTE = 500000  # 0.5 seconds
HALF_NOTE = 1000000  # 1.0 second
WHOLE_NOTE = 2000000  # 2.0 seconds
EIGHTH_NOTE = 250000  # 0.25 seconds

# Predefined tunes as lists of (frequency, duration_microseconds) tuples
PREDEFINED_TUNES = {
    "twinkle": [
        (NOTE_FREQUENCIES["C4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["G4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["G4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["A4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["A4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["G4"], HALF_NOTE),
        (NOTE_FREQUENCIES["F4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["F4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["E4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["E4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], HALF_NOTE),
    ],
    "happy_birthday": [
        (NOTE_FREQUENCIES["C4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["F4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["E4"], HALF_NOTE),
        (NOTE_FREQUENCIES["REST"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["G4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["F4"], HALF_NOTE),
    ],
    "mary_had_little_lamb": [
        (NOTE_FREQUENCIES["E4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["C4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["E4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["E4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["E4"], HALF_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["D4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["D4"], HALF_NOTE),
        (NOTE_FREQUENCIES["E4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["G4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["G4"], HALF_NOTE),
    ],
    "alert": [
        (NOTE_FREQUENCIES["A5"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["REST"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["A5"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["REST"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["A5"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["REST"], QUARTER_NOTE),
    ],
    "success": [
        (NOTE_FREQUENCIES["C4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["E4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["G4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["C5"], QUARTER_NOTE),
    ],
    "error": [
        (NOTE_FREQUENCIES["G4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["F4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["E4"], QUARTER_NOTE),
        (NOTE_FREQUENCIES["D4"], HALF_NOTE),
    ],
    "startup": [
        (NOTE_FREQUENCIES["C4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["D4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["E4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["F4"], EIGHTH_NOTE),
        (NOTE_FREQUENCIES["G4"], QUARTER_NOTE),
    ],
}
