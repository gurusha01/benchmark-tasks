# Checks for the grade-rod laser receiver task

All runs on one TITAN RTX each, Isaac Lab-Arena 0.3.1 with yam_arena, 2,400-step limit, 20 Hz, one trial per process.

## 1. Real hardware and where each number comes from

| Item | Value in the scene | Source |
|---|---|---|
| Occupation | Construction laborers, O*NET 47-2061, 892,690 employed (OEWS May 2024) | [BLS OEWS 47-2061](https://www.bls.gov/oes/current/oes472061.htm) |
| Grade rod | two-section fibreglass rod with a rod-level bubble; a receiver clamps straight onto it | e.g. [AdirPro fibreglass grade rod with rod level bubble](https://homedepot.com/p/AdirPro-18-ft-Fiberglass-Oval-Shaped-Laser-Level-Grade-Rod-in-10ths-Scale-with-Rod-Level-Bubble-711-45-PKG/323100464); [All about grade rods](https://www.engineersupply.com/all-about-grade-rods.aspx) |
| Receiver | Spectra Precision HR320 class with its rod clamp; coarse on-grade band 6.0 mm (+-3 mm), 2 in reception window | [HR320 with C59 clamp](https://www.cansel.ca/product/spectra-precision-hr320-laser-receiver-with-c59-clamp-508588), [HR320 manual](https://www.manualslib.com/manual/3268041/Spectra-Hr320.html) |
| Procedure | rod on the stake, plumb by its bubble; slide the receiver until it reads on grade; clamp | [All about grade rods](https://www.engineersupply.com/all-about-grade-rods.aspx) |
| Rod section 25 x 44 mm, 1.3 kg, collapsed 1.32 m | estimate | not published per model |
| Receiver 16.5 x 7.5 x 4 cm, 0.38 kg | estimate | not published |
| Clamp: quarter-turn wing knob, sprung jaw | estimate | |

A real trial needs a rotary laser kit with receiver and tripod (about $250 to $300, e.g. [VEVOR rotary laser kit](https://www.bestbuy.com/product/vevor-rotary-laser-level-kit-1650ft-360-degree-self-leveling-red-horizontal-laser-transitwith-heavy-duty-tripod-receiver-red-black/JJGHK45KWF/sku/11964292)), a fibreglass grade rod with a bubble (about $60), a 2x2 stake and a post to lean the rod on.

## 2. Category and why it is robotic

Impact on jobs. Setting the receiver on the rod is the first step of every grade check on a site (footings, slabs,
trenches), and the reading copies into all later work. The difficulty is physical: one arm must hold a 1.3 m rod plumb
within 1 deg with its shoe on a 38 mm stake top while the other arm loosens a clamp, slides a 0.38 kg receiver along
the rod by feel against a +-3 mm display band, and retightens it, without pushing the rod off the stake or out of plumb.
The right hand's sliding force pushes the rod; the left hand must keep it plumb against that.

## 3. Contact only

The receiver rides on the rod by contact: the clamp jaw is a sprung slider pressed onto the rod as the wing knob turns,
and friction holds the receiver. The rod's shoe rests on the stake top by gravity and friction. No joint, latch or
proximity trigger connects the parts. The laser beam, the three display marks and the bubble are code-driven from the
receiver's height and the rod's tilt (they are indicators only).

## 4. Observable inputs

The demo uses the top-camera and right-wrist-camera images, the arms' joint angles (forward kinematics) and the site
layout from the setup text. It reads the receiver's display marks and the round level's bubble from the top image and
finds the wing knob on the wrist image. It never reads simulator object poses, the laser plane's height or the clamp's
joints (`grep` of `script.txt` finds no `states(`, `_STATE` or `sliders`). Demo cameras were 640 x 352.

## 5. Demo and baselines

Scripted demo, final task and script, one trial per process (seed draws the receiver's start error and the shoe's
offset on the stake):

| Seed | Receiver start (mm from grade) | Score | Steps to success | Receiver at end (mm from grade) | Rod tilt (deg) |
|---|---|---|---|---|---|
| 0 | -17.5 | 1.0 | 1037 | +1.3 | 0.66 |
| 1 | +13.3 | 1.0 | 1128 | +0.5 | 0.59 |
| 2 | -16.7 | 1.0 | 920 | +0.6 | 0.66 |
| 3 | -12.8 | 1.0 | 1751 | -0.1 | 0.71 |
| 4 | +16.4 | 1.0 | 1180 | +0.1 | 0.73 |
| 5 | +22.0 | 1.0 | 1446 | +2.2 | 0.58 |
| 6 | -21.0 | 1.0 | 1322 | -0.9 | 0.71 |
| 7 | +18.9 | 1.0 | 1155 | -0.2 | 0.99 |

Baselines (same task file, `BASELINE_SEED` per run):

| Baseline | Score | Gate |
|---|---|---|
| noop seed 0 | 0.0 | none |
| noop seed 1 | 0.0 | none |
| random seed 0 | 0.0 | none |
| random seed 1 | 0.0 | rod fell |
| random seed 2 | 0.0 | none |
| reach seed 0 | 0.0 | rod fell |
| reach seed 1 | 0.0 | rod fell |
| reach seed 2 | 0.0 | rod fell |

Dream-1 (seed 0, 2,400 steps, 320 x 176 cameras, instruction from task.json): 0.0.

Failure modes seen while building the demo (all fixed in the script, none in the scorer except the 1 mm seating
allowance): the receiver drops about 5 mm as the clamp takes its weight, so the demo now always reaches the green band
from below; the rod can turn in the left hand's pads while the right hand slides the receiver (one seed ended 1.27 deg
off plumb with the pads vertical), so the demo now finishes by reading the bubble and walking the rod's top toward it.

## 6. Overlap check (board, Nov 21)

| Task | Status | How ours differs |
|---|---|---|
| #2202 level a floor sleeper by pushing a shim until the bubble centres | accepted | one-arm push of a wedge under a resting part; ours holds a free 1.3 m rod plumb with one arm while the other works a clamp on it |
| #2065 mark a point with a plumb bob | accepted | steadies a hanging bob; no clamp, no display, no held rod |
| #2379 level an automatic level on foot screws | closed | screw-turning on an instrument base |
| #2392 level an analytical balance by its feet | closed | thumb-wheel feet on a lab balance |
| #880 raise a guyed mast plumb | closed | guy lines and pegs |
| #2341 set a sprinkler head to grade on its swing joint | closed | no laser, no rod; screws a head up or down |

No board task sets a receiver on a rod or uses a laser receiver display (`gh search` for "laser receiver", "grade rod",
"rotary laser", "laser level", "leveling rod": no hits).
