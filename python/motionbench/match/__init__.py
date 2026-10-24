"""Multi-robot matches: simulate, render broadcast-style video, report, and
benchmark trackers against ground truth. See docs/match.md."""

from .field import FIELD_M, Field, default_field
from .io import BROADCAST_CAMERA, MatchData, write_match
from .sim import PERIODS, MatchResult, simulate

__all__ = ["FIELD_M", "Field", "default_field", "BROADCAST_CAMERA", "MatchData", "write_match", "PERIODS", "MatchResult", "simulate"]
