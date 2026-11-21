# Set a laser receiver on a grade rod to grade, the rod held plumb on its stake

On a grading crew the laser sets the height of everything: footings, slabs, trenches. A rotary laser sweeps a level
plane across the site; a laborer stands the grade rod's shoe on the grade stake, holds the rod plumb by its round
bubble, loosens the receiver's rod clamp, slides the receiver until its display reads on grade, and clamps it. Every
later grade check copies that reading, so a rod held off plumb, a shoe off the stake top, or a receiver that slips
after clamping puts the work at the wrong height.

Here the rodman has stood the rod (a collapsed two-section fibreglass grade rod) on the stake and leaned it on a 4x4
post. The receiver is still clamped where it read for the last stake: 12 to 22 mm too high or too low (the seed
decides). The robot must swing the rod plumb about its shoe and hold it there with one arm, while the other arm works
the receiver: loosen the clamp's wing knob a quarter turn, slide the receiver against the display's marks, retighten
and let go, all without knocking the rod off the stake.

## Files

- `task.py`: scene, seeded start, scoring (yam_arena task file). `score()` and `success()` are the rubric in
  `task.json`. The receiver rides on the rod by contact only: its clamp jaw is a sprung slider, pressed onto the rod
  as the wing knob turns; nothing snaps it to the rod.
- `robokit.py`: rigid parts built from boxes and cylinders with sprung joints (clamp jaw, wing knob), and lamps
  (the receiver's marks, the bubble) that show or hide. No change to yam_arena is needed.
- `demo.py`, `script.txt`: the scripted demo (below).
- `script_noop.txt`, `script_random.txt`, `script_reach.txt`: baselines (do nothing, random joint targets, random
  reaches and grasps); `BASELINE_SEED` seeds them.
- `media/checks.md`: demo and baseline runs, real-part sources, the observable-inputs check and the overlap check.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

The score is `task_score` in the log: 1.0 on success (held 1 s), otherwise `score()` from `task.py` at the end.

Scripted demo (one MP4 per camera, `setup.png` and `result.json` per seed):

    DEMO_CAM_W=640 DEMO_CAM_H=352 python demo.py task.py demo_out --seeds 0 1 2 --max-steps 2400

The demo uses only what a policy has: top-camera and right-wrist-camera images, the arms' joint angles (forward
kinematics) and the site layout in the setup text. It reads the receiver's marks and the round level's bubble from the top image
and finds the wing knob on the wrist image. It never reads simulator poses, the laser plane's height or the
clamp's joints.
