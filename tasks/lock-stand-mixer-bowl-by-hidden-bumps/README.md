# Lock a stand mixer's bowl onto its clamping plate

A tilt-head stand mixer (KitchenAid K45SS size) stands on the table with its head tilted back, and its empty 4.5 qt
stainless bowl stands in front of it. The bowl's foot carries three bumps; the clamping plate has three narrow slots,
each leading clockwise into a channel under a lip. Once the bowl is over the plate, the bumps and the slots are hidden
under it. The robot has to lower the bowl, held level by its rim, with its bumps over the slots (or turn it by feel
until it drops onto the plate), then turn it clockwise until the bumps are under the lips, and let go.

## Files

- `task.py`: scene and scoring (yam_arena task file). Mixer and bowl are each one rigid body built from boxes and
  cylinders; slots, lips, stops and bumps are colliders, so nothing is latched in code. `score()` and `success()` use
  the bowl's and the mixer's poses and the fingertip positions.
- `robokit.py`: multi-shape rigid parts for yam_arena task files (boxes may be turned about z, y and x).
- `demo.py`, `script.txt`: the scripted demo. `script_noop.txt`, `script_random.txt` (random joint targets) and
  `script_reach.txt` (random reaches and grasps) are the baselines; `BASELINE_SEED` seeds them. `MIX_NAIVE=1` runs
  the demo but lets go as soon as the bowl is on the plate, without turning it. `MIX_LIFTTEST=1` tries to lift the
  locked bowl 3 cm before letting go.

## Run a trial

    inspect-robots run --embodiment yam_arena --policy dream \
        -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
        --instruction "Put the mixing bowl on the stand mixer and lock it in place, then let go." \
        --scorer task_score --max-steps 2400 --store-frames --log-dir logs

Scripted demo (one MP4 per camera, `setup.png`, `result.json` per seed):

    python demo.py task.py demo_out --script script.txt --seeds 0 1 2 3 4 5 6 7 --max-steps 2400

The demo uses the poses of the two objects (what the cameras show: the plate's slots before the bowl covers them, the
bowl's turn from its handle) and its own joints. It grips the rim on both sides, at the bowl's own quarter points read from its handle, with the fingers across the wall,
lifts the bowl 10 cm, carries it over the plate turned so that its bumps are over the slots, lowers it to 2 mm above
the lips, then sets it down and loosens its grip so the bowl settles under its own weight. If the bowl rides on the lips (7 mm high), it turns it to and fro in 1.5 degree steps,
pressing lightly, until it drops. It then turns it 36.6 degrees clockwise in six steps, lets go and moves both arms
away.

## Real parts and sizes

- Mixer: KitchenAid Classic K45SS, 13.9 x 8.7 x 14.1 in, 22 lb (9.98 kg) net, about $280 to $300 (retailer
  listings). The pedestal height, column and head blocks are estimates within that envelope.
- Bowl: KitchenAid K45SBWH 4.5 qt polished stainless steel bowl with handle: 8.75 in rim, 7 in tall, 9.6 in across
  the handle (retailer listings). Its mass, 0.73 kg, is an estimate (listed shipping weights 2.0 to 2.2 lb include the
  carton).
- Locking: the maker's manuals say to set the bowl on the bowl clamping plate and turn it clockwise; its help page
  says the bowl's base has bumps that lock into the base plate. The clamping plate and bump sizes (three 20 degree
  slots, 30 degree channels, 3 mm bumps 6 mm proud) are estimates from photos: no drawing is published.

## Runtime and seeds

- Run limit 2,400 control steps (2 min at 20 Hz). The demo succeeds at steps 510 to 610 (26 to 31 s) on seeds 0 to 7.
- The bowl's start is drawn from the seed: x within 1.5 cm, y within 2 cm and its turn within 5 degrees of the
  nominal pose.
- Wall time: about 4 minutes per trial on one TITAN RTX at 320 x 176 per camera.
