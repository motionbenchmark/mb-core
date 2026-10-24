"""Games: defined once (games/<name>/game.json), then simulated, scored and compared.

Run a team's VEXcode AIM Python routine in simulation, play the same game with
simulated drivers, and get a verdict on whether the code beats people before it
runs on the real robot. See docs/games.md.
"""

from .driver import CHARACTERS, Driver, class_roster
from .run import RunResult, simulate, write_run
from .spec import Game, create_game

__all__ = ["CHARACTERS", "Driver", "class_roster", "RunResult", "simulate", "write_run", "Game", "create_game"]
