#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
robot=Robot()
#endregion VEXcode Generated Robot Configuration

# Sample routine 1: dead reckoning, no sensors.
# Written for layout dev-01: the robot starts in the middle of the near wall facing the far wall.
# Positions are in mm from the start (x to the right, y forward). It drives a fixed plan and
# never looks at the field, so any slip adds up.

ROBOT_REACH = 100  # stop this far short of a barrel's centre so the magnet catches it

BARRELS = [  # in the order they were measured, not a smart order
    ("orange", 386, 130), ("orange", -314, 830), ("orange", 236, 580),
    ("blue", -364, 180), ("blue", 136, 930), ("blue", -164, 500),
]
ZONES = {"orange": (-700, 490), "blue": (700, 490)}

x, y = 0.0, 0.0  # where we think we are


def go_to(tx, ty, stop_short=0):
    global x, y
    dx, dy = tx - x, ty - y
    dist = math.sqrt(dx * dx + dy * dy)
    heading = math.degrees(math.atan2(dx, dy))  # clockwise from forward
    robot.turn_to(heading)
    robot.move_for(dist - stop_short, 0)
    k = (dist - stop_short) / dist if dist > 0 else 0
    x, y = x + dx * k, y + dy * k


robot.set_move_velocity(80)
robot.set_turn_velocity(100)
for colour, bx, by in BARRELS:
    go_to(bx, by, ROBOT_REACH)
    zx, zy = ZONES[colour]
    go_to(zx, zy)
    robot.kicker.place()
    robot.move_for(80, 180)
    y -= 0  # (backing up straight back; the plan ignores it)
