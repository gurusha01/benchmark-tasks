# Seat a pod coffee maker's water reservoir back on its hidden valve

**Instruction:** "Put the water reservoir back on the coffee maker, seated on its valve, and let go."

Single-serve pod coffee makers such as the Keurig K-Classic (K50) carry a removable 48 oz water reservoir on
their left side. It comes off to be rinsed or filled at the sink and goes back on the brewer's base ledge. Under
the reservoir's back end a spring valve sits in a short collar. When the collar drops into the valve well on the
ledge, a post in the well pushes the valve open and the reservoir's floor comes to rest flat on the ledge. A
reservoir that is set down a few millimetres off stands on its collar beside the well, high and tilted, with the
valve shut; the usual advice for a brewer that will not pump is to reseat the reservoir until it sits flat. The
well is hidden under the reservoir while it goes down, so the robot has to note where the well is before the
reservoir covers it, carry the reservoir level, and line the collar up blind.

## Running a trial

    inspect-robots run --embodiment yam_arena -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "Put the water reservoir back on the coffee maker, seated on its valve, and let go." \
        --scorer task_score --max-steps 1200

- Scene: `station`; 20 Hz control; run limit 1,200 steps (60 s). One fixed start, so seeds change nothing in the
  layout. `--scorer task_score` gives 1.0 on success, otherwise `score()` at the end of the run.
- Scripted demo (writes an MP4 per camera, `setup.png` and `result.json` per seed):

      python demo.py task.py demo_out --script script.txt --seeds 0 1 2 3 4 5 6 7 --max-steps 1200

  The demo uses the left arm only and only what the cameras show: the coffee maker's pose (with the well on its
  ledge, visible before the reservoir covers it) and the reservoir's pose. It pinches the reservoir's outer wall
  at the top band, lifts it, carries it over the ledge, maps its hand through the reservoir's measured pose to hold
  the reservoir level and square, lines the collar up over the well, lowers it in steps of 2 mm, presses 4 mm past
  the seat, lets go and backs away. It reaches success at steps 438-480 (22-24 s).
- Baselines (`media/checks.md`): `script_noop.txt`, `script_random.txt` (random joint targets) and
  `script_reach.txt` (random reaches and grasps), seeded by `BASELINE_SEED`; `NAIVE_DX=0.010` or `-0.006` runs
  the demo with the target shifted 10 mm or 6 mm along the ledge (a reservoir placed by its outline, not on the
  well).
- Wall time: about 4 minutes per trial, including simulator start-up, on one TITAN RTX at 320 x 176 per camera.

## Scoring

The scorer reads the reservoir's and the coffee maker's poses, the valve poppet's lift and both grippers'
positions (from the joint states).

| Rule | Value |
|---|---|
| Gate (zero for the rest of the run) | Coffee maker moved more than 10 mm or tilted more than 3 deg; reservoir falling faster than 0.9 m/s, off the table or upside down |
| Released and still | Each gripper's centre point 30 mm or more from the reservoir's bounding box; reservoir under 1 cm/s and 0.1 rad/s; both for 1 s. Every tier needs it. |
| Collar in the well | Collar centre within 2.5 mm of the well centre along both ledge axes (the 2 mm clearance plus 0.5 mm), collar mouth 3 mm or more below the ledge top |
| Seated | Collar in the well, reservoir floor within 1.5 mm of the ledge top, tilt under 1.5 deg, poppet lifted 3 mm or more (4 mm when seated) |
| Tiers | 1.0 seated; 0.5 collar in the well but not seated (wedged or tilted); 0 otherwise |
| Success | 1.0 held for 1 s |

**Real trial:** after the robot lets go, slide a 1.5 mm feeler gauge at each corner of the reservoir's floor:
seated reads no gap and the reservoir does not rock under a light touch on its top. A collar in the well holds
against a 5 N sideways push at the reservoir's base (spring scale); a collar beside the well slides. A phone video
shows the release, the 1 s of stillness and any fall.

## Layout (metres, world frame; table top z = 0.7475; arm bases at (0, +/-0.305))

| Object | Type | Material | Size | Mass | Pose |
|---|---|---|---|---|---|
| Coffee maker body | free rigid body (with the ledge) | black ABS | 338 x 170 x 330 mm | 4.4 kg | x 0.306-0.644, y -0.115-+0.055, front toward the robot |
| Base ledge | part of the coffee maker | dark ABS | 190 x 90 x 30 mm | - | x 0.380-0.570, y 0.055-0.145 |
| Valve well | part of the ledge | - | 26 x 26 mm, 10 mm deep | - | centred (0.520, 0.094) |
| Valve post | part of the coffee maker | steel | 6 mm across, 7 mm from the well floor | - | in the well centre |
| Reservoir | free articulation (tank plus valve poppet) | clear copolyester, 2.5 mm walls, 3 mm floor | 150 x 75 x 240 mm; 22 mm band 5 mm thick along both long sides at the top | 0.40 kg | start: bottom centre (0.255, 0.215), collar toward +x, leaning 4.2 deg on its collar |
| Valve collar | part of the reservoir | white plastic | 22 x 22 mm outside, 2 mm walls, 8 mm below the floor | - | 45 mm behind the reservoir's centre |
| Valve poppet | prismatic link, spring 300 N/m, 5 mm travel | - | 12 x 12 x 2 mm | 3 g | 1 mm inside the collar mouth |

Friction: coffee maker feet 0.9, reservoir 0.5.

## Real-world parts list

- A Keurig K-Classic K50 coffee maker with its 48 oz reservoir (about $100-120 on sale, $159.99 list, Target).
  Lid off, reservoir empty.
- A 1.5 mm feeler gauge, a 0-10 N spring scale and a phone for video.

## Sources and estimates

- Coffee maker: 13.0 in high, 9.8 in wide, 13.3 in deep, 10.6 lb, 48 oz removable reservoir (Keurig K50 product
  page; Target and Best Buy listings). The body here is 170 mm wide, the 9.8 in less the reservoir and its gap.
- Reservoir valve: a spring valve under the reservoir is pushed open when the reservoir is seated on the brewer;
  a reservoir not seated square leaves it shut (repair guides and owners' forums, for example iFixit Answers
  "reservoir problem", Barista Life "Keurig not pumping water").
- Estimates (not published): reservoir outline 150 x 75 x 240 mm and 0.40 kg empty (Keurig reservoirs are tall
  narrow tanks; the larger K-2550 reservoir is 4 1/8 x 14 1/4 x 13 in); ledge height; collar, well and post sizes;
  valve spring rate; reservoir position along the body.

## Simplifications

- The round collar and well are square here, with 2 mm clearance each side; the well has no lead-in chamfer.
- The poppet is a sprung slider; there is no water and no flow.
- The coffee maker body is a plain box with its ledge; the drip tray and brew head are drawn but not simulated.
- The reservoir is rigid and has no lid.

## Licences

All objects are built from boxes and cylinders by the task's own code (`robokit.py`). No third-party models,
meshes or textures are used.
