# Capturing a real run (VEX field)

What you need: a phone, a tripod or a ceiling mount, a printer, tape.
Setup takes about 15 minutes; after that each run takes as long as the run.

## 1. Print tags

```bash
motionbench field --vex -o field.json
motionbench tags --config field.json --out tags/
```

Print `tag36h11-00..03.png` (15 cm) and `tag36h11-10.png` (10 cm) at **100% scale**.
Measure the black square: if it is not the size in the label, fix your printer
scaling and print again. Glue them to card so they stay flat.

## 2. Place tags

* Tags 0, 1, 2, 3: flat on the field, centre 15 cm in from each corner
  (0 = near-left, 1 = near-right, 2 = far-right, 3 = far-left as seen from the
  camera side). The **top edge** of every corner tag faces the far wall.
* Tag 10: flat on top of the robot, centred, **top edge facing the robot's front**.
  Measure its height above the floor and note it as `robot_tag_height_m` in
  `field.json`.

## 3. Place the camera

* As high as possible (≥ 2 m), on the near side, centred, the whole field and all
  four corner tags in frame. Lock focus and exposure; 30 fps or more; 1080p.
* For parallax correction, measure the lens position in field metres and set
  `camera_xyz_m` in `field.json` (e.g. `[1.83, -0.6, 2.4]`). Without it, a tag
  10 cm above the floor seen from 2.4 m shifts by up to ~4% of its distance from
  the point under the camera.
* Do not move the camera between runs. If it moves, re-check all four tags are visible.

## 4. Record and track

Start recording, start the run, stop recording after the robot stops.

```bash
motionbench track run.mp4 --config field.json --task MB-VEX-NAV@1.0 --seed dev-01 \
    --agent-id driver-07 --agent-class human --trial 0 -o runs/driver-07/t0
motionbench score runs/driver-07/t0
```

The tracker reports its detection rate and calibration error. Re-shoot if the
robot was found in fewer than 90% of frames or calibration RMS is above 1 cm.
Trim the video so it starts within a second of the robot starting to move; the
scorer re-bases time to the first pose sample.

## 5. Delivery tasks

After the run, record where every object ended up (`world_final.json`), either by
measuring or by a still photo with the tags visible. Verified-tier runs require it.

## Privacy

Frame the camera so people are not identifiable, or blur them before sharing.
Runs by minors need guardian consent before any video leaves the team.
