"""
Dance patterns for Create 3 robot
https://github.com/iRobotEducation/create3_examples/blob/humble/create3_examples_py/create3_examples_py/dance/create3_dance.py
"""

from .choreographer import Move, Lights, FinishedDance, ColorPalette

# Initialize color palette
cp = ColorPalette()

# Circle dance pattern - 9 seconds of circular motion with blue/cyan lights
CIRCLE_DANCE = [
    (0.0, Lights([cp.blue] * 6)),
    (0.0, Move(0.2, 45)),
    (0.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (1.0, Lights([cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan])),
    (1.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (2.0, Lights([cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan])),
    (2.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (3.0, Lights([cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan])),
    (3.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (4.0, Lights([cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan])),
    (4.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (5.0, Lights([cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan])),
    (5.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (6.0, Lights([cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan])),
    (6.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (7.0, Lights([cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan])),
    (7.5, Lights([cp.cyan, cp.blue, cp.cyan, cp.blue, cp.cyan, cp.blue])),
    (8.0, Move(0.0, 0.0)),
    (8.5, Lights([cp.white] * 6)),
    (9.0, FinishedDance()),
]

# Spin dance pattern - 7 seconds of spinning with rainbow color rotation
SPIN_DANCE = [
    (0.0, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
    (0.0, Move(0.0, 180)),
    (0.5, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
    (1.0, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
    (1.5, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
    (2.0, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
    (2.5, Lights([cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red])),
    (3.0, Move(0.0, -180)),
    (3.5, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
    (4.0, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
    (4.5, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
    (5.0, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
    (5.5, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
    (6.0, Move(0.0, 0)),
    (6.5, Lights([cp.white] * 6)),
    (7.0, FinishedDance()),
]

# Party dance - 23 seconds of complex movements and lights
PARTY_DANCE = [
    (0.0, Lights([cp.green] * 6)),
    (0.0, Move(0.0, 0.0)),
    (0.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
    (0.5, Move(0.15, 70)),
    (1.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
    (1.0, Move(0.15, -70)),
    (1.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
    (1.5, Move(0.15, 70)),
    (2.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
    (2.0, Move(0.15, -70)),
    (2.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
    (2.5, Move(-0.15, 70)),
    (3.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
    (3.0, Move(-0.15, -70)),
    (3.5, Lights([cp.white, cp.grey, cp.white, cp.grey, cp.white, cp.grey])),
    (3.5, Move(-0.15, 70)),
    (4.0, Lights([cp.grey, cp.white, cp.grey, cp.white, cp.grey, cp.white])),
    (4.0, Move(-0.15, -70)),
    (4.5, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
    (4.5, Move(0.0, 60)),
    (5.0, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
    (5.5, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
    (6.0, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
    (6.5, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
    (6.5, Move(0.0, -75)),
    (7.0, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
    (7.5, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
    (8.0, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
    (8.5, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
    (9.0, Lights([cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red])),
    (9.5, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
    (9.5, Move(-0.15, 50)),
    (10.0, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
    (10.0, Move(-0.15, -50)),
    (10.5, Lights([cp.pink, cp.cyan, cp.red, cp.green, cp.blue, cp.yellow])),
    (10.5, Move(-0.15, 50)),
    (11.0, Lights([cp.yellow, cp.pink, cp.cyan, cp.red, cp.green, cp.blue])),
    (11.0, Move(-0.15, -50)),
    (11.5, Lights([cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red, cp.green])),
    (11.5, Move(0.0, 75)),
    (13.0, Lights([cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan, cp.red])),
    (13.0, Move(0.0, 75)),
    (14.0, Lights([cp.red, cp.green, cp.blue, cp.yellow, cp.pink, cp.cyan])),
    (14.0, Move(0.0, -100)),
    (15.5, Lights([cp.cyan, cp.red, cp.green, cp.blue, cp.yellow, cp.pink])),
    (15.5, Move(0.0, 100)),
    (20.5, Lights([cp.red, cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow])),
    (20.5, Move(-0.15, -70)),
    (21.0, Lights([cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow, cp.red])),
    (21.0, Move(-0.15, 70)),
    (21.5, Lights([cp.red, cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow])),
    (21.5, Move(0.15, -70)),
    (22.0, Lights([cp.yellow, cp.red, cp.yellow, cp.red, cp.yellow, cp.red])),
    (22.0, Move(0.15, 70)),
    (22.5, FinishedDance()),
]

# Dictionary of all available dance patterns
DANCE_PATTERNS = {
    "circle": CIRCLE_DANCE,
    "spin": SPIN_DANCE,
    "party": PARTY_DANCE,
}
