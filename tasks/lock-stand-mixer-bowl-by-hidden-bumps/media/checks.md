# Checks

All runs in Isaac Lab-Arena with the yam_arena kit at 20 Hz, run limit 2,400 steps.

## Scripted demo (`script.txt`)

| seed | score | success at step |
|---|---|---|
| 0 | 1.0 | 572 |
| 1 | 1.0 | 530 |
| 2 | 1.0 | 570 |
| 3 | 1.0 | 530 |
| 4 | 1.0 | 607 |
| 5 | 1.0 | 610 |
| 6 | 1.0 | 570 |
| 7 | 1.0 | 510 |

On several seeds the bowl first came down resting on one lip, tilted 2 to 4 degrees and 2 to 5 mm high; it dropped
onto the plate only after the yaw was swept by a few degrees while pressing lightly. The bumps then turned 36.6
degrees clockwise to the stop (end yaw 0.6 to 0.9 degrees past it, inside the 15.9 to 35.1 degree lock window).

## Baselines (all 0)

| baseline | seeds | score | end state |
|---|---|---|---|
| no-op (`script_noop.txt`) | 0 | 0.0 | bowl on the table |
| random joint targets (`script_random.txt`, BASELINE_SEED = seed) | 0, 1, 2 | 0.0, 0.0, 0.0 | bowl on the table or knocked over |
| random reaches and grasps (`script_reach.txt`) | 0, 1, 2 | 0.0, 0.0, 0.0 | bowl on the table or knocked over; mixer moved on seed 2 |
| set the bowl on the plate and let go, no turn (`MIX_NAIVE=1`) | 0, 1, 2 (and 0, 1 again after the last script change) | 0.0 each | seated (on the plate, centred, level) but at the slots, 33.7 to 35.4 degrees from locked |

## Lock check (`MIX_LIFTTEST=1`, seed 0)

After locking, both grippers still on the rim were raised 3 cm: the bowl rose 0.5 mm (the fingers slid up the rim)
and the mixer did not move, so the bumps hold the bowl down under the lips. The same grip lifts the free bowl 9 cm
off the table at the start of every demo.

## Notes

- One fixed nominal start; the seed moves the bowl by up to 1.5 cm in x, 2 cm in y and 5 degrees in yaw.
- An earlier version of the demo gripped the rim at fixed world angles and failed seed 4 (the arms ran out of
  reach while turning); gripping at the bowl's own quarter points fixed it. The scorer did not change.
