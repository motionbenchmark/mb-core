"""Overhead-camera tracker: phone video + printed AprilTags -> `.mbtrace`.

Setup (VEX field):
  * four AprilTags (tag36h11, ids 0-3) taped flat in the field corners,
    their top edge facing the far (+y) wall;
  * one tag (id 10) on top of the robot, its top edge facing the robot's front;
  * a phone on a tripod or ceiling mount that sees the whole field.

    from motionbench.tracker import FieldConfig, track_video
    cfg = FieldConfig.vex()
    result = track_video("run.mp4", cfg)
    result.write("run.mbtrace", task="MB-VEX-NAV@1.0", seed="dev-01",
                 agent_id="driver-01", agent_class="human")

No tags? ``motionbench.tracker.untagged`` tracks a robot in an ordinary phone
video from four tapped floor corners (``motionbench calibrate`` makes the page).

Requires the `tracker` extra: ``pip install motionbench[tracker]``.
"""

from .field import FieldConfig
from .track import TrackResult, track_video, track_frames
from .untagged import Calibration, UntaggedResult, track_untagged

__all__ = ["FieldConfig", "TrackResult", "track_video", "track_frames", "Calibration", "UntaggedResult", "track_untagged"]
