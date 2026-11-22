# School scoliosis screen: slide a scoliometer down a manikin's back, notch on the spine, both feet on; refer at 7 deg

Registered nurses (O*NET 29-1141.00; 3,282,010 US jobs, BLS OEWS May 2024), school nurses among them, screen
adolescents for idiopathic scoliosis with the Adams forward-bend test and a scoliometer (Bunnell 1984). The student
bends forward; the nurse lays the scoliometer across the back with its central notch over the spinous processes and
both ends on the skin, without pressing, slides it from the upper thoracic spine to the lumbar spine, and notes the
largest angle of trunk rotation (ATR). An ATR of 7 deg or more is referred (Bunnell 1993; AAFP 2001). A notch off the
spine, a lifted end or a hand that tilts the instrument gives a false reading: missed curves progress untreated, false
referrals cost radiographs.

The skill is contact-rich and continuous. The instrument must stay seated on both feet over a curved back (the spine
rises and falls 2.2 cm along the sweep) while it rolls with the rib hump, so the hand must follow the surface and
shift sideways as the plate rolls to keep the notch on the spine; the reading is a ball in a tube, seen only by a
wrist camera; and the answer follows from that reading alone (the hump is not visible from the top camera at
320x176, and no back has a peak between 6 and 8 deg).

## Files

- `task.py`: scene, seeded start and scoring (yam_arena task file). `score()` and `success()` are the rubric below.
  Eight manikin backs are built (different hump size, side and level); the seed picks one (seed mod 8) and the rest
  are parked off the table. The back's skin is a smooth triangle mesh (spinous ridge, paraspinal muscles, ribs,
  rounded flanks, rib hump); its colliders are overlapping 12 mm capsules within 0.7 mm of the drawn skin where the
  instrument touches. The skin shell rides on a 1,500 N/m vertical spring that measures the load on the back.
  The scoliometer's ball is a damped pendulum (the ball in its tube); its two finger pads are point contacts, so the
  plate rolls freely about the spine axis when the gripper holds it by them, as between finger and thumb.
- `robokit.py`: rigid parts built from boxes, cylinders and capsules with sprung joints. This copy adds a slider
  centre of mass, per-capsule friction, collider-only capsules and smooth visual meshes.
- `demo.py`, `script.txt`: the scripted demo (below).
- `script_noop.txt`, `script_random.txt`, `script_reach.txt`: baselines (do nothing, random joint targets, random
  reaches and grasps); `BASELINE_SEED` seeds them. `script_naive.txt`: put the slip in REFER without measuring.
  `script_place.txt`: the demo with the instrument set down at mid-thoracic level and slid only 2 cm.
- `run_dream.sh`, `mkdream.py`: the Dream-1 run (after approval) and its videos.
- `media/checks.md`: sources, demo and baseline tables, observable-inputs check, overlap check.

## Scoring (same as `score()` in task.py)

Score 0 if at any point the load on the manikin's back exceeds 20 N, the scoliometer falls off the table, or the
manikin moves more than 20 mm. Contact: a foot within 1.5 mm of the skin. 0.25 when the control steps with both
feet in contact, the notch within 10 mm of the spine line and the plate within 20 deg of square to the spine cover
at least 60% of the 25 cm screening span (5 mm bins, upper thoracic to lumbar). 0.5 when, in addition, those steps
cover every bin within 20 mm of the hump's peak level and both feet were in contact for at least 90% of the steps
with a foot on the back inside the span. 0.75 when, in addition, the slip block rests wholly inside the correct
tray: REFER if the peak angle of trunk rotation is 7 deg or more, else PASS. 1.0 (success, held 1 s) when, in
addition, the scoliometer stands in its case and no gripper centre is within 5 cm of it or of the slip.

A single placement cannot earn the sweep rungs: 0.25 needs seated, centred contact over 15 cm of the 25 cm span,
and 0.5 also needs the 4 cm around the hump's peak. The tray rung counts only after the sweep, so a policy that puts
the slip in a fixed tray without measuring scores 0.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo (one MP4 per camera, `setup.png` and `result.json` per seed; one seed per process):

    DEMO_CAM_W=640 DEMO_CAM_H=352 python demo.py task.py demo_out --seeds 0 --max-steps 2400

The demo uses only what a policy has: the left wrist camera's image of the scale (ball and red zero mark), the arms'
joint angles (forward kinematics, also for the wrist camera's pose) and the fixed layout in the setup text. It never
reads simulator object poses, the ball's joint, the back's spring or the scorer.

## Real trial (under $300 of parts)

Back: a child torso form, e.g. Econoco Mondo K4 8-10 yr jersey torso form ($102.99, Zoro), laid face down on a
padded stand with the back up, plus a rib-hump overlay cut and heat-formed from 10 mm EVA foam ($17.26, Walmart) and
glued over the ribs on one side at the seeded level; its trunk rotation is set and checked by hand with the
scoliometer before the trial. Instrument: an OSI (Bunnell) Scoliometer ($66 to $130 at school-nurse suppliers) with
two 8 mm rubber bumpers glued at the top centre as finger pads; its open case, a slip block, and a red and a green
tray. Load: a bathroom or kitchen scale under the stand (gate at 20 N above the resting load). Contact and notch
position: a camera level with the back at the side and one behind the shoulders, marked spine line, 5 mm level
marks along the span.
