# Checks for the mower grass-bag task

All runs on one TITAN RTX, Isaac Lab-Arena 0.3.1 with yam_arena, 2,400-step limit, 20 Hz.

## 1. Real hardware and where each number comes from

| Item | Value in the scene | Source |
|---|---|---|
| Mower | Sun Joe MJ401E, 14 in corded electric push mower, about $140 | [Home Depot listing](https://www.homedepot.com/p/202554772) |
| Width, weight | 16.5 in (41.9 cm), 29 lb (13.2 kg) | [Sun Joe product page](https://shopjoe.com/products/sun-joe-14-inch-12-amp-electric-lawn-mower) (61.5 x 16.5 x 29 in assembled, 29 lb) |
| Grass bag | MJ401E-10, 10.6 gal (40 l), fabric, about $25 | [Sun Joe bag page](https://shopjoe.com/products/sun-joe-mj401e-mj401e-pro-lawn-mower-grass-bag), [Overstock listing](https://www.overstock.com/Home-Garden/Sun-Joe-MJ401E-MJ401E-PRO-Lawn-Mower-Grass-Bag-10.6-gallons/18258038/product.html) |
| Fitting procedure | raise the spring-loaded ejector flap, set the bag over the coupling hooks on the mower body, release the flap | MJ401E(-PRO) operator's manual, grass bag assembly ([manualmachine copy](https://manualmachine.com/sunjoe/mj401epro/20188921-operators-manual)) |
| Bag body 38 x 36 x 29 cm, 1.0 kg | estimate | sized to the published 40 l; mass not published |
| Hooks 8 cm either side, slot floor 32.5 cm up, 21 mm slot between the rear face and a flared lip | estimate | not published |
| Flap 24 x 15 cm, 0.15 kg, hinge 39 cm up, 0.25 N m/rad spring | estimate | not published |
| Wheels 7 in, handle 8 cm over the hard top | estimate | not published |

A real trial needs the mower (about $140) and its bag (about $25); nothing else.

## 2. Category

Societal relevance/relatability. A 14 in corded mower is homeowner equipment, not a landscaping crew's tool, so we do
not claim Impact on jobs. Bagging clippings with a walk-behind mower is one of the most common household outdoor
chores, and the bag is refitted every time it is emptied. The skill (one arm holds a sprung safety flap out of the way
while the other carries a bulky part in under it and drops a rod into two hooks it cannot see well) is the same on the
landscaping crews' commercial mowers.

## 3. The hooks hold by contact only

There is no latch, joint or proximity trigger between bag and mower. The rod is a 10 mm cylinder on the bag; each
hook is an open-topped slot (a floor plate standing out from the rear face, an outer lip with a flared top, and the
rear face itself). The rod stays in the hooks
only because gravity holds it on the slot floors, and the flap lies on the bag's hard top by contact. `seated()` in
`task.py` only measures where the rod's axis is.

## 4. Demo and baselines in the simulator

Scripted demo, final task and script, one trial per seed (seed draws the bag's start offset: dx, dy, yaw):

| Seed | Start offset (mm, mm, deg) | Camera estimate of rod x / centre y / heading | Score | Steps to success | Rod in hooks (x, z, mower frame, mm) | Flap at end |
|---|---|---|---|---|---|---|
| 0 | -2.3, -3.6, 0.25 | 237.5 / -5.0 / -0.87 | 1.0 | 869 | -16.0, 330.0 | 61.9 deg |
| 1 | -4.8, 8.6, 0.05 | 234.1 / 13.2 / 1.87 | 1.0 | 873 | -15.8, 330.0 | 61.9 deg |
| 2 | -2.2, 3.6, -0.31 | 237.5 / 3.9 / 0.23 | 1.0 | 863 | -15.7, 330.0 | 61.9 deg |
| 3 | -4.6, -1.7, -0.76 | 235.8 / -5.7 / -1.19 | 1.0 | 868 | -16.0, 330.0 | 61.9 deg |
| 4 | -2.3, 6.0, 1.83 | 236.7 / 16.4 / 3.13 | 1.0 | 864 | -15.8, 330.0 | 61.9 deg |
| 5 | -3.1, -2.2, 1.09 | 237.5 / 0.2 / 0.18 | 1.0 | 869 | -15.9, 330.0 | 61.9 deg |
| 6 | -0.3, -7.1, 1.14 | 239.1 / -8.0 / -1.24 | 1.0 | 870 | -15.9, 330.0 | 61.9 deg |
| 7 | -4.5, 9.1, 1.00 | 233.8 / 18.6 / 3.40 | 1.0 | 871 | -16.0, 330.0 | 61.9 deg |

8 of 8 seeds succeed, at 863 to 873 steps (43 to 44 s of the 120 s limit). The rod ends in the outer half of each
21 mm slot (slot spans x -21 to 0) resting on the floor (z 330 = resting height).

The camera estimate is good to about 3 mm along and 1 to 10 mm across; the heading estimate errs by up to 2.4 deg.
The demo does not need it closer: after the grasp it works from the measured joints, and the hooks' flared lips and
the rear face guide the rod. By the measured joints, the bag's descent stopped 3 to 4 cm above the slot floors on
every seed (the rod on the lips' flared tops), and the rod dropped into the slots when the fingers opened.

Baselines (2,400 steps each, `--no-record`):

| Run | Score | Gate | Rod at the end (x, z, mm) |
|---|---|---|---|
| Do nothing, seed 0 | 0.0 | - | -188, 284 (bag where it started) |
| Do nothing, seed 1 | 0.0 | - | -190, 284 |
| Random joint targets, seeds 0 / 1 / 2 | 0.0 / 0.0 / 0.0 | - | bag pushed about, never on the hooks |
| Random reaches and grasps, seeds 0 / 1 / 2 | 0.0 / 0.0 / 0.0 | mower moved (all three) | bag never on the hooks |

No baseline reached either rung. The random reaches knock the mower (13.2 kg, free on the bench) more than 5 mm,
which is itself a gate.

## 5. The demo uses only observable inputs

`script.txt` reads: the first top-camera image (`ctx.obs.images["top_cam"]`), the arms' joint angles through forward
kinematics (`fk`, `_actual_tcp`), and the public layout constants in `task.py` (mower position, hook height, flap
hinge and size, bag dimensions). It never calls `ctx.states()`, `task.seated`, `task.rod_in_hooks` or reads the
flap's joint: `grep -nE "states\(|seated|rod_in_hooks|sliders|_STATE" script.txt` finds nothing. The flap angle it
uses comes from where the left fingertip is.

## 6. Overlap with the board

Searched all PR titles and bodies and the accepted tasks for mower, grass, bag, catcher, ejector/discharge flap,
spring-loaded flap/guard/lid, hold open while inserting, and hook on/onto rod. Closest:

| Task | Status | How ours differs |
|---|---|---|
| #1403 Hold a spring-loaded flange joint open and slide a blind plate into the gap | closed | ours carries a bulky 1 kg bag in under a swinging flap and drops a rod into two hooks; theirs pulls a flange apart and slides a plate into a gap |
| #1837 Clip a spring-hinged pulse oximeter onto a finger while steadying the hand | merged | small spring clip opened by the gripper itself; no carried part |
| hang-a-blind-headrail-in-gated-brackets | accepted | hooks a rail into brackets; no sprung part to hold out of the way, nothing heavy or bulky |
| refit-drawer-past-rear-hooks | accepted | carries a drawer above riding height past hooks; no flap held by the other arm |
| #2263 Lift a fry basket out of the oil and hang it on the fryer's hanger | open | hangs a basket on a hanger; no sprung part, no second arm holding anything |

Also seen: #1670 "Tip a push mower onto its side to reach the blade" (closed; different action), #1699 piano lid prop
(closed; holding a lid while setting a prop).
