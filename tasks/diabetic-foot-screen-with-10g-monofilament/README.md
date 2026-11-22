# Diabetic foot sensation screen: press a 10 g monofilament until it bends at four plantar sites, beside a callus

Medical assistants (O*NET 31-9092.00, about 811,000 US jobs, BLS 2024) and registered nurses do the annual diabetic
foot exam in primary care. Its sensation test (IWGDF 2019, ADA) touches a 10 g (5.07 Semmes-Weinstein) nylon
monofilament to the plantar hallux and the 1st, 3rd and 5th metatarsal heads: perpendicular to the skin, pressed
until the filament bends, about 2 s from approach to removal, never sliding, never on an ulcer, callus or scar (test
at its edge). A filament pressed at an angle, jabbed past its bend, slid, or put on callus gives a false "can feel"
or "cannot feel", and a missed loss of protective sensation leads to undetected foot ulcers and amputations.

The sole is curved and tipped: each site has its own normal (28 to 51 deg from the bench's x axis, 16 to 26 deg from
each other), so the pen must be re-aimed for every site. The filament is a buckling column: it carries about 0.1 N
and then bends, so the barrel must stop in an 11 mm depth window (24 to 35 mm from the skin) and hold still for about
a second without the tip sliding more than 3 mm. One site carries a yellow callus, seen only on the camera image;
that site must be tested 2 to 8 mm outside the callus edge.

## Files

- `task.py`: scene, seeded start (callus site, foot stand shift along y, pen shift along its cradle) and scoring
  (yam_arena task file). `score()` and `success()` are the rubric in `task.json`. The filament is a chain of four
  rigid capsule links on sprung hinges: it buckles under contact physics; no latch, no proximity trigger.
- `robokit.py`: rigid parts built from boxes, cylinders and capsules, with sprung joints. This copy adds serial chains
  (a joint hung on another joint's link), per-link inertia and full shape orientations. It hooks `yam_arena.scene` and
  the embodiment from the task's import; no change to yam_arena is needed.
- `demo.py` and `script.txt`: the scripted demo (see below).
- `script_noop.txt`, `script_random.txt`, `script_reach.txt`: baselines (do nothing, random joint targets, random
  reaches and grasps); `BASELINE_SEED` seeds them. `script_naive.txt` (every site pressed straight along +x) and
  `script_soleplane.txt` (every site pressed along the sole's mean normal) are the demo with the press direction
  changed.
- `run_dream.sh`, `mkdream.py`: the Dream-1 run (after approval) and its videos.
- `media/checks.md`: sources, demo and baseline tables, the observable-inputs check and the overlap check.

## Scoring (same as `score()` in task.py)

Score 0 if at any point the filament touches the callus, the pen's barrel touches the foot, the pen falls off the
bench, or the foot model moves more than 20 mm. Otherwise 0.25 for each site whose first contact was a correct test:
the filament tip landed within 8 mm of the site (on the callus site: 2 to 8 mm outside the callus edge), the barrel
tip came within 35 mm of the skin (the filament bent) but never closer than 24 mm, the pen's axis stayed within 15 deg
of the skin normal and the tip within 3 mm of its landing point while bent, and the contact lasted 1.0 to 3.0 s. A
site's later contacts never count. Four sites score 0.75 until the pen lies back in its cradle with no gripper centre
within 5 cm of it: then 1.0 (success, held 1 s). Doing nothing scores 0.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo (writes one MP4 per camera, `setup.png` and `result.json` per seed; one seed per process):

    DEMO_CAM_W=640 DEMO_CAM_H=352 python demo.py task.py demo_out --seeds 0 --max-steps 2400

The demo uses only what a policy has: the top-camera image (the blue pen in its cradle; the yellow callus, whose
position also gives the foot stand's shift), the arms' joint angles (forward kinematics) and the fixed layout in the
setup text. It never reads simulator object poses, the filament's joints or the scorer.

## Real trial

Foot: a life-size rigid adult foot model on a sloped stand (any anatomical foot model, about $50 to $150), with an
18 mm disc of yellow moleskin glued on one site as the callus. Pen: Owen Mumford Neuropen with a 10 g monofilament
(about $40), in a two-post cradle. Mark each site with a 16 mm ring of washable ink around an 8 mm target dot and
film the trial with a side camera at each site's level: the bend, the dwell (1 to 3 s from touch to lift-off), the
angle (protractor overlay on the side view) and any slide are read from the video; ink on the moleskin is a callus
touch.
