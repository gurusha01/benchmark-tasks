# Take a throat swab for a strep culture: hold the tongue down with a tongue blade and rub both tonsillar pillars without touching anything else

Medical assistants (O*NET 31-9092.00, about 811,000 US jobs, BLS 2024) collect throat swabs for rapid strep tests and
cultures many times a day. The procedure: hold the tongue down with a tongue blade, rub the swab over both tonsillar
pillars (or the back wall of the throat), keep it off the tongue, teeth, cheeks and uvula going in and coming out, take
the swab out, then the blade. A swab that touches anything else picks up mouth flora and gives a false result; a blade
pushed onto the back third of the tongue makes the patient gag.

One arm has to hold a 1.8 mm wooden blade on the tongue against its spring for the whole time the other arm is in the
mouth. The other arm steers a 2.5 mm swab through a 5 x 4 cm mouth opening, past the blade, into a 3.2 cm gap
between the pillars, rubs each one, crosses in front of the uvula and comes back out without touching anything.

## Files

- `task.py`: scene, seeded start pose and scoring (yam_arena task file). `score()` and `success()` are the rubric in
  `task.json`. The tongue is a block on a sprung slider: no latch, no proximity trigger; contact physics only.
- `robokit.py`: rigid parts built from boxes and cylinders, with sprung joints (the tongue). It hooks `yam_arena.scene`
  and the embodiment from the task's import; no change to yam_arena is needed.
- `demo.py` and `script.txt`: the scripted demo (see below).
- `script_noop.txt`, `script_random.txt`, `script_reach.txt`: baselines (do nothing, random joint targets, random
  reaches and grasps over the work area); `BASELINE_SEED` seeds them.
- `media/checks.md`: demo and baseline runs, sources, the observable-inputs check, the real-trial check and the
  overlap check.

## Run a trial

Inside the Isaac Lab-Arena environment with `yam-isaac-lab-arena` installed:

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

The score is `task_score` in the log: 1.0 on success (held 1 s), otherwise `score()` from `task.py` at the end of the
episode. After every reset the task file shifts the blade and the swab along their rests by a seeded 0 to 12 mm and
lets the scene settle before the first observation; `-E object_jitter=0` keeps the head where the setup puts it.

Scripted demo (writes one MP4 per camera, `setup.png` and `result.json` per seed):

    DEMO_CAM_W=640 DEMO_CAM_H=352 python demo.py task.py demo_out --seeds 0 --max-steps 2400

The demo uses only what a policy has: the first top-camera image (where the blade and the swab lie), the arms' own
joint angles (forward kinematics) and the head's fixed layout from the setup text. It never reads the simulator's
object poses or the tongue's joint.

## Real trial

Head: an adult oral and nasal swab simulator (CAE Medicor oral and nasal swab simulator, about $750-1,000; teeth,
tongue, uvula), blade: a 6 in x 3/4 in wooden tongue depressor; swab: BD BBL CultureSwab (or any rayon culture swab).
Before each trial dot the left pillar with blue and the right pillar with green washable marker, and dust the tongue,
uvula, cheeks, teeth, palate and the blade's upper face with UV tracer powder (e.g. Glo Germ). After the trial the
swab tip must show blue and green and no UV glow under a UV torch (a touch outside the pillars); the blade's inner end
must not have passed a line drawn across the tongue two thirds of the way back (gag). Film the trial with the robot's
cameras and a side camera.
