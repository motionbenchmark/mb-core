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

## 6. VEX AIM on a table

The AIM tasks (`MB-AIM-NAV@0.1`, `MB-AIM-SORT@0.1`) use a 6 ft × 4 ft
(1.83 m × 1.22 m) area. Tape the outline on a table or the floor; it is a
MotionBench arena, not an official VEX field.

```bash
motionbench field --aim -o aim-field.json   # floor tags 20-23, robot tag 24
motionbench tags --config aim-field.json --out tags/
```

* The AIM kit's AprilTag signs use IDs 0-4. The AIM preset uses 20-24 so the
  tracker and the robot's AI Vision never read each other's tags. Keep the kit's
  signs off the arena unless your program uses them, and tell us if it does.
* Print the floor tags at 8 cm and the robot tag at 4.5 cm, and stick the robot
  tag flat on top of the robot facing forward.
* The robot is small, so mount the phone higher and more directly overhead
  (about 1.5 m above the centre of the table), and measure the camera position
  into `camera_xyz_m` so the 7.5 cm tag height is corrected.
* For sorting, after the run note where each barrel ended up (or take a photo
  from the camera position) to fill in `world_final.json`.

## 7. No tags: an ordinary phone video

Printed tags give the most accurate results, but MotionBench can also measure a run
from a plain video, like a robot vacuum filmed from the end of a hallway.

1. **Film it.** Hold the phone still (a little hand shake is fine) high enough to see
   the whole floor area and the robot's whole path. Keep people out of the picture.
2. **Calibrate.** `motionbench calibrate run.mov -o calib.html`, open the page, and tap
   the four corners of a rectangle on the floor whose long sides run along the walls
   (far-left, far-right, near-right, near-left). Optionally trace the full floor
   outline (door nooks) and cut off the bottom of the picture. If you can, type in a
   tape-measured width (and length): one measurement makes the size exact, two also
   fix the lens. Download `calib.json`.
3. **Track and score.** `motionbench track run.mov --untagged --calib calib.json -o out/ --overlay`.
   Check `out/overlay.mp4`: the ring should sit on the robot's base all the way through.

How it works: frames are aligned to the first one, the median of all frames is the
empty floor, the robot is whatever is darker (or lighter) than it, and its centre is the
lowest point of its silhouette plus half its diameter. Without measurements the
robot's own size is the ruler. Floor the camera never saw is marked `unobserved` and
left out of the score.

Expected accuracy (rendered test video with known positions): about 2 cm with a
measured width and length, about 6 cm from the robot's size alone.

## Privacy

Frame the camera so people are not identifiable, or blur them before sharing.
Runs by minors need guardian consent before any video leaves the team.
