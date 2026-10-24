#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
robot=Robot()
#endregion VEXcode Generated Robot Configuration

# Sample routine 4: plan the order, drive fast, let AI Vision do the last few centimetres.
# Layout dev-01 (positions in mm from the start, x right, y forward). Picks the next barrel by
# "distance to it + distance from it to its zone", drives most of the way by odometry at full
# speed, then steers onto the barrel with the camera. Whatever barrel the magnet catches is
# delivered to its own zone.

BARRELS = [("orange", 386, 130), ("orange", -314, 830), ("orange", 236, 580),
           ("blue", -364, 180), ("blue", 136, 930), ("blue", -164, 500)]
ZONES = {"orange": (-764, 490), "blue": (764, 490)}
# three drop spots per zone, so a new barrel doesn't land on (and grab) one already there
SLOTS = {"orange": [(-800, 340), (-800, 640), (-760, 490)], "blue": [(800, 340), (800, 640), (760, 490)]}
placed = {"orange": 0, "blue": 0}
PLACE_REACH = 124  # the kicker puts a barrel this far in front of the robot's centre
KIND = {"orange": ORANGE_BARREL, "blue": BLUE_BARREL}


def pos():
    return robot.get_x_position(), robot.get_y_position()


def heading_to(tx, ty):
    x, y = pos()
    return math.degrees(math.atan2(tx - x, ty - y)), math.hypot(tx - x, ty - y)


def cost(b):
    x, y = pos()
    zx, zy = ZONES[b[0]]
    return math.hypot(b[1] - x, b[2] - y) + math.hypot(zx - b[1], zy - b[2])


def fetch(b):
    colour, bx, by = b
    h, d = heading_to(bx, by)
    robot.turn_to(h)
    if d > 230:
        robot.move_for(d - 230, 0)
    for _ in range(120):
        if robot.has_any_barrel():
            robot.stop_all_movement()
            return True
        seen = robot.vision.get_data(KIND[colour])[0]
        robot.move_at(seen.bearing if seen.exists else 0, 100)
        wait(30, MSEC)
    robot.stop_all_movement()
    return robot.has_any_barrel()


def deliver():
    colour = "orange" if robot.has_orange_barrel() else "blue"
    zx, zy = SLOTS[colour][placed[colour] % 3]
    placed[colour] += 1
    h, d = heading_to(zx, zy)
    robot.turn_to(h)
    robot.move_for(max(0, d - PLACE_REACH), 0)
    robot.kicker.place()
    robot.move_for(90, 180)
    return colour


robot.set_move_velocity(100)
robot.set_turn_velocity(150)
todo = list(BARRELS)
while todo:
    b = min(todo, key=cost)
    if fetch(b):
        x, y = pos()
        colour = deliver()
        # cross off the barrel of that colour nearest to where we picked one up
        mine = [t for t in todo if t[0] == colour]
        if mine:
            todo.remove(min(mine, key=lambda t: math.hypot(t[1] - x, t[2] - y)))
    else:
        todo.remove(b)
