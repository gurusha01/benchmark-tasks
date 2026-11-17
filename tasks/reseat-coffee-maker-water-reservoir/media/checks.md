# Checks

All runs: `station` scene, 20 Hz, zero object jitter, run limit 1,200 steps, `task_score` as the scorer reports it.

## Scripted demo (`script.txt`)

| Seed | Score | Steps | Collar offset from well centre (x, y) | Floor above ledge | Tilt | Poppet lift |
|---|---|---|---|---|---|---|
| 0 | 1.0 | 438 | -0.44, 0.35 mm | 0.32 mm | 0.19 deg | 3.80 mm |
| 1 | 1.0 | 438 | -0.33, 0.27 mm | 0.36 mm | 0.23 deg | 3.78 mm |
| 2 | 1.0 | 480 | 0.04, -0.07 mm | 0.00 mm | 0.00 deg | 3.96 mm |
| 3 | 1.0 | 472 | -0.09, 0.05 mm | 0.00 mm | 0.00 deg | 3.96 mm |
| 4 | 1.0 | 480 | -0.22, 0.14 mm | 0.00 mm | 0.00 deg | 3.96 mm |
| 5 | 1.0 | 438 | -0.54, 0.27 mm | 0.38 mm | 0.24 deg | 3.77 mm |
| 6 | 1.0 | 438 | -0.15, 0.20 mm | 0.32 mm | 0.19 deg | 3.77 mm |
| 7 | 1.0 | 438 | -0.40, 0.13 mm | 0.43 mm | 0.26 deg | 3.72 mm |

8/8 at 1.0, success at steps 438-480 (22-24 s) of 1,200. The layout is fixed, so seeds differ only by
simulator timing. Peak joint torque while carrying: 43% of the shoulder limit (joint 2), under 30% elsewhere.

## Baselines

| Baseline | Runs | Score | End state |
|---|---|---|---|
| Do nothing (`script_noop.txt`) | seeds 0-2 | 0, 0, 0 | reservoir still on the table at its start |
| Random joint targets (`script_random.txt`) | `BASELINE_SEED` 0-2 | 0, 0, 0 | reservoir knocked onto its side twice, left leaning 21 deg on the table once |
| Random reaches and grasps (`script_reach.txt`) | `BASELINE_SEED` 0-2 | 0, 0, 0 | coffee maker pushed (gate) in all three |
| Placed by outline 10 mm forward of the seat (`NAIVE_DX=0.010`) | 1 | 0 | collar on the ledge 10.3 mm off the well: floor 4.9 mm high, tilted 3.7 deg, valve shut |
| Placed by outline 6 mm back of the seat (`NAIVE_DX=-0.006`) | 1 | 0 | collar on the ledge 5.6 mm off the well: floor 5.5 mm high, tilted 4.2 deg, valve shut |

The two outline placements run the full demo, released level and still, with the target moved along the ledge.
A reservoir set down against the brewer's side but off the well scores 0: it rests on its collar, high and
tilted, with the valve shut.

## Development notes

- A straight-down grasp cannot reach the top of a 240 mm reservoir on a 30 mm ledge; the demo pinches the outer
  wall's top band with the hand pitched 40 deg down.
- The loaded arm sags about 30 mm at the seat, so open-loop placement misses the well by 4 mm; the demo closes the
  loop on the reservoir's measured pose (the reservoir is visible; the well's position was seen before it was
  covered).
