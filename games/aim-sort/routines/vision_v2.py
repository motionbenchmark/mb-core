#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
robot=Robot()
#endregion VEXcode Generated Robot Configuration

# Sample routine 2: find barrels with AI Vision, carry them home by odometry.
# Layout dev-01. Zones are at x = -764 (orange) and x = +764 (blue), y = 490, in mm from the start.

ZONES = {"orange": (-720, 490), "blue": (720, 490)}
ZONE_HALF_X = 150  # a barrel this close (in x) to a zone centre is already sorted


def position():
    return robot.get_x_position(), robot.get_y_position()


def estimate(obj):
    """Where a seen barrel is, from its bearing and apparent width (bigger = closer)."""
    dist = 12200 / max(obj.width, 1) + 60  # mm, from the camera's field of view
    a = math.radians(robot.inertial.get_heading() + obj.bearing)
    x, y = position()
    return x + dist * math.sin(a), y + dist * math.cos(a)


def unsorted(obj, colour):
    bx, by = estimate(obj)
    zx, _ = ZONES[colour]
    return abs(bx - zx) > ZONE_HALF_X


def look():
    """The biggest (closest) barrel that still needs sorting, or None."""
    best = None
    for kind, colour in ((ORANGE_BARREL, "orange"), (BLUE_BARREL, "blue")):
        for obj in robot.vision.get_data(kind):
            if obj.exists and unsorted(obj, colour) and (best is None or obj.width > best[0].width):
                best = (obj, colour)
    return best


def fetch():
    # turn on the spot until a barrel is in view
    for _ in range(60):
        seen = look()
        if seen:
            break
        robot.turn_for(RIGHT, 25)
    else:
        return False
    # steer at it until the magnet has it
    for _ in range(200):
        if robot.has_any_barrel():
            robot.stop_all_movement()
            return True
        seen = look()
        if not seen:
            robot.move_for(40, 0)
            continue
        robot.move_at(seen[0].bearing, 70)
        wait(50, MSEC)
    robot.stop_all_movement()
    return robot.has_any_barrel()


def deliver():
    colour = "orange" if robot.has_orange_barrel() else "blue"
    zx, zy = ZONES[colour]
    x, y = position()
    dx, dy = zx - x, zy - y
    robot.turn_to(math.degrees(math.atan2(dx, dy)))
    robot.move_for(math.sqrt(dx * dx + dy * dy) - 60, 0)
    robot.kicker.place()
    robot.move_for(120, 180)


robot.set_move_velocity(80)
robot.set_turn_velocity(100)
while True:
    if fetch():
        deliver()
    else:
        break
