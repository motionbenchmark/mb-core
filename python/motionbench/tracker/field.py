from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

VEX_FIELD_M = 3.6576  # 12 ft
AIM_ARENA_M = (1.8288, 1.2192)  # 6 ft x 4 ft, MotionBench-defined arena for VEX AIM tasks


@dataclass
class FieldConfig:
    """Where the reference tags sit on the floor, and which tag is the robot.

    Coordinates are metres in the task's arena frame (origin at the lower-left
    field corner, +x right, +y towards the far wall). Reference tags lie flat
    with their top edge facing +y.
    """

    width_m: float
    height_m: float
    tag_size_m: float = 0.15
    #: tag id -> [x, y] of the tag centre on the floor
    corner_tags: dict[int, list[float]] = field(default_factory=dict)
    robot_tag_id: int = 10
    robot_tag_size_m: float = 0.10
    #: added to the tag's top-edge direction to get the robot heading (rad)
    robot_tag_yaw_offset: float = 0.0
    #: height of the robot tag above the floor (m), for parallax correction
    robot_tag_height_m: float = 0.0
    #: camera lens position [x, y, z] in field metres, if measured (enables parallax correction)
    camera_xyz_m: Optional[list[float]] = None
    family: str = "tag36h11"

    @classmethod
    def vex(cls, inset_m: float = 0.15, tag_size_m: float = 0.15) -> "FieldConfig":
        """Default for a 12 ft x 12 ft VEX field: tags 0-3 in the corners."""
        w = h = VEX_FIELD_M
        return cls(
            width_m=w,
            height_m=h,
            tag_size_m=tag_size_m,
            corner_tags={0: [inset_m, inset_m], 1: [w - inset_m, inset_m], 2: [w - inset_m, h - inset_m], 3: [inset_m, h - inset_m]},
        )

    @classmethod
    def aim(cls, inset_m: float = 0.06, tag_size_m: float = 0.08) -> "FieldConfig":
        """6 ft x 4 ft table arena for the VEX AIM tasks.

        The AIM kit's own AprilTag signs use IDs 0-4, so the floor tags here are
        20-23 and the robot tag is 24: the tracker never confuses them with signs
        the robot itself is looking at. The robot tag sits on top of the robot
        (about 7.5 cm up), so measure the camera position for parallax correction.
        """
        w, h = AIM_ARENA_M
        return cls(
            width_m=w,
            height_m=h,
            tag_size_m=tag_size_m,
            corner_tags={20: [inset_m, inset_m], 21: [w - inset_m, inset_m], 22: [w - inset_m, h - inset_m], 23: [inset_m, h - inset_m]},
            robot_tag_id=24,
            robot_tag_size_m=0.045,
            robot_tag_height_m=0.075,
        )

    @classmethod
    def room(cls, width_m: float, height_m: float, inset_m: float = 0.15) -> "FieldConfig":
        """Generic rectangular room (e.g. for MB-COVER)."""
        return cls(
            width_m=width_m,
            height_m=height_m,
            corner_tags={0: [inset_m, inset_m], 1: [width_m - inset_m, inset_m], 2: [width_m - inset_m, height_m - inset_m], 3: [inset_m, height_m - inset_m]},
        )

    def tag_corners_field(self, tag_id: int) -> list[list[float]]:
        """Field positions of a reference tag's corners, in OpenCV's order (TL, TR, BR, BL)."""
        cx, cy = self.corner_tags[tag_id]
        s = self.tag_size_m / 2
        return [[cx - s, cy + s], [cx + s, cy + s], [cx + s, cy - s], [cx - s, cy - s]]

    def save(self, path: "str | Path") -> None:
        d = asdict(self)
        d["corner_tags"] = {str(k): v for k, v in self.corner_tags.items()}
        Path(path).write_text(json.dumps(d, indent=2))

    @classmethod
    def load(cls, path: "str | Path") -> "FieldConfig":
        d = json.loads(Path(path).read_text())
        d["corner_tags"] = {int(k): v for k, v in d["corner_tags"].items()}
        return cls(**d)
