#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
robot=Robot()
#endregion VEXcode Generated Robot Configuration

# Sample routine 3: AI Vision for barrels AND for the AprilTags behind each zone.
# Same plan as routine 2, but the last stretch to a zone homes in on its tag (tag 1 behind
# orange, tag 2 behind blue), so wheel slip doesn't put barrels outside the zone. It also
# drives faster and picks barrels close to where they need to go.

ZONES = {"orange": (-720, 490), "blue": (720, 490)}
TAGS = {"orange": TAG1, "blue": TAG2}
ZONE_HALF_X = 150


def position():
    return robot.get_x_position(), robot.get_y_position()


def estimate(obj):
    dist = 12200 / max(obj.width, 1) + 60
    a = math.radians(robot.inertial.get_heading() + obj.bearing)
    x, y = position()
    return x + dist * math.sin(a), y + dist * math.cos(a)


def look():
    """Best barrel to go for: close to us, and close to its own zone."""
    best, best_cost = None, 1e9
    x, y = position()
    for kind, colour in ((ORANGE_BARREL, "orange"), (BLUE_BARREL, "blue")):
        for obj in robot.vision.get_data(kind):
            if not obj.exists:
                continue
            bx, by = estimate(obj)
            zx, zy = ZONES[colour]
            if abs(bx - zx) <= ZONE_HALF_X:
                continue
            cost = math.hypot(bx - x, by - y) + 0.5 * math.hypot(bx - zx, by - zy)
            if cost < best_cost:
                best, best_cost = (obj, colour), cost
    return best


def fetch():
    for _ in range(40):
        seen = look()
        if seen:
            break
        robot.turn_for(RIGHT, 30)
    else:
        return False
    for _ in range(200):
        if robot.has_any_barrel():
            robot.stop_all_movement()
            return True
        seen = look()
        if not seen:
            robot.move_for(40, 0)
            continue
        robot.move_at(seen[0].bearing, 100)
        wait(40, MSEC)
    robot.stop_all_movement()
    return robot.has_any_barrel()


def deliver():
    colour = "orange" if robot.has_orange_barrel() else "blue"
    zx, zy = ZONES[colour]
    x, y = position()
    dx, dy = zx - x, zy - y
    robot.turn_to(math.degrees(math.atan2(dx, dy)))
    robot.move_for(max(0, math.sqrt(dx * dx + dy * dy) - 250), 0)  # most of the way by odometry
    # home in on the tag behind the zone
    for _ in range(100):
        tag = robot.vision.get_data(TAGS[colour])[0]
        if not tag.exists:
            robot.turn_for(RIGHT if colour == "blue" else LEFT, 15)
            continue
        if tag.width >= 60:  # about 23 cm from the wall: the barrel lands well inside the zone
            break
        robot.move_at(tag.bearing, 70)
        wait(40, MSEC)
    robot.stop_all_movement()
    robot.kicker.place()
    robot.move_for(120, 180)


robot.set_move_velocity(100)
robot.set_turn_velocity(150)
while True:
    if fetch():
        deliver()
    else:
        break
