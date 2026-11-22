# Checks for the scoliometer screening task

All runs on edward, one TITAN RTX each (GPUs 2 and 3, two processes per GPU), Isaac Lab-Arena with yam_arena,
PhysX 240 Hz, 20 Hz control, 2,400-step limit, one seed per process.

## 1. Real job, real procedure, real objects

| Item | Value in the scene | Source |
|---|---|---|
| Who does it | Registered nurses, O*NET 29-1141.00 (school nurses screen students): 3,282,010 US jobs, BLS OEWS May 2024 | [BLS OEWS May 2024 release, Table 1](https://www.bls.gov/news.release/archives/ocwage_04022025.htm); [O*NET 29-1141.00](https://www.onetonline.org/link/summary/29-1141.00) |
| Method | Adams forward bend; scoliometer laid across the back, notch over the spine; ATR is the largest reading | Bunnell WP, An objective criterion for scoliosis screening, JBJS Am 1984;66(9):1381-7, [doi:10.2106/00004623-198466090-00010](https://doi.org/10.2106/00004623-198466090-00010) |
| Sweep | "moves the scoliometer along the vertebral column ... moving down the spine ... noting the highest degree of asymmetry"; referral rule ATR >= 7 deg (2003 rules) | [Minnesota Dept. of Health / Gillette school scoliosis screening manual](https://gillettechildrens.org/assets/uploads/documents/Scoliosis-Screening-Manual.pdf) (p.4, App. E) |
| Referral at 7 deg | Bunnell 1984 found 5 deg sensitive; 7 deg is the later referral criterion | Bunnell WP, Selective screening for scoliosis, Clin Orthop 1993;(288):40-45; [AAFP 2001](https://www.aafp.org/afp/2001/0701/p32.html) ("7 degrees ... cut-off for referral") |
| Instrument | OSI (Bunnell) Scoliometer: ball in a curved water-damped glass tube, 0-30 deg each way, semicircular indentation for the spinous processes; 19.0 x 8.9 cm (CME listing); 10 mm thick and 80 g are estimates | [US Patent 5,181,525](https://patents.google.com/patent/US5181525A/en); [Gillette manual photo p.14](https://gillettechildrens.org/assets/uploads/documents/Scoliosis-Screening-Manual.pdf); [CME Corp 12-1099, 3.5 x 7.5 in, $66.04](https://www.cmecorp.com/fei-scoliometer.html); [School Health, $129.99](https://www.schoolhealth.com/scoliometer) |
| Back | adolescent forward-bent back, 34 cm shoulders to waist, 27 cm wide, spinous ridge 5 mm proud, paraspinal muscles 5 mm proud 25 mm either side, ribs falling 5 deg, rib hump 3-11 deg ATR (Gaussian, sigma 5 cm) | anatomical estimates; ATR range from Bunnell 1984 (5-20 deg curves) |
| Real-trial back (< $300) | Econoco Mondo K4 8-10 yr child jersey torso form, $102.99; 10 mm EVA foam sheet for the rib-hump overlay, $17.26 | [Zoro K4](https://zoro.com/econoco-mondo-mannequins-8-10yr-old-child-crme-jersey-torso-form-wneckblock-k4/i/G8875940); [Walmart EVA foam](https://www.walmart.com/ip/534730696) |

## 2. Why it is robotic (and why a single touch earns nothing)

The decision is one threshold on one number; the work is keeping a 19 cm plate seated on both feet, notch on the
spine, while sliding it 25 cm over a back whose height changes by 2.2 cm and which rolls under it by up to 11 deg.
The plate rolls freely in the fingers, so it seats itself only if the hand follows the surface in height and moves
sideways as the plate rolls (the finger pads sit 8.1 cm above the feet: at 11 deg the notch swings 15 mm off the
spine unless the hand shifts). The reading is seen only on the plate's face by a wrist camera, which has to be held
behind the moving plate by the other arm. The score needs seated, centred contact over at least 15 cm of the span,
and over the 4 cm around the hump's peak, so a single placement or a touch at one level scores 0 (`script_place.txt`
below).

## 3. Contact only; how the back and the instrument are modelled

- Back. Drawn as a smooth triangle mesh: spinous ridge (Gaussian, 5.1 mm, sigma 4 mm), paraspinal muscles (5 mm,
  sigma 12 mm, 25 mm from the spine), ribs falling at 5 deg, rounded flanks, and the rib hump as a rotation of the
  cross-section about the spine that rises and fades along the spine (sigma 5 cm). No box corners. Colliders: per
  6 mm of spine, three overlapping 12 mm capsules each side (inner and outer paraspinal, ribs) whose tops follow the
  drawn profile within 0.7 mm (checked by bisection at u = 0 to 88 mm), plus a capsule ridge. The skin shell rides on
  a 1,500 N/m, 60 N s/m vertical spring (a padded manikin), preloaded for its 0.8 kg; the load gate reads the spring.
- Scoliometer. White plate with two feet (rounded pads, friction 0.15) at +-88 mm, a 30 x 24 mm notch, the scale on
  the face toward the robots (tube band, 1 deg ticks, red zero mark) and an 8 mm dark ball. The ball is a pendulum
  of 14 cm radius (the tube's), damped 1e-3 N m s/rad, 4 g; its angle relative to the plate is the reading. Two 8 mm
  sphere finger pads at the top centre: the gripper pinches them, point contacts carry no torque about the pinch
  axis, so the plate rolls freely about the spine axis and seats itself on both feet.
- Nothing is latched; feet contact, notch offset, squareness and coverage are measured from geometry every control
  step (feet: capsule-to-collider distance).

## 4. Observable inputs

`script.txt` reads the left wrist camera's image every control step during the sweep (red zero mark and dark ball,
each cast onto the plate's face plane using the camera pose from the left arm's joint angles and the plate's plane
from the right arm's joint angles; reading = asin(lateral offset / 127 mm), median of the last 9 frames), the arms'
joint angles, and the fixed layout from the setup text. It never calls `ctx.states()`, the scorer, the ball joint or
the back's spring: `grep -nE "states\(|_STATE|sliders|write_pose|_part_state|VARIANTS|atr_of" script.txt` finds
nothing. Demo cameras were 640 x 352.

## 5. Demo and baselines

Scripted demo, final task and script, one seed per process, 640 x 352 cameras. "Demo's largest camera reading" is what
the script read off the left wrist image (median of 9 frames); "largest ball reading" is the ball joint's angle
relative to the plate while both feet were down (scorer log). Span covered: share of the 50 five-mm bins of the
screening span with both feet down, notch within 10 mm of the spine and the plate square; peak bins: bins within
20 mm of the peak level.

| Seed | True peak ATR (deg) / answer | Demo's largest camera reading (deg) / tray | Largest ball reading, both feet down (deg) | Span covered | Peak bins | Both feet | Max load (N) | Score | Steps |
|---|---|---|---|---|---|---|---|---|---|
| 0 | +9.5 at x 31 cm / REFER | 8.70 / REFER | 9.47 | 100% | 8/8 | 100% | 2.1 | 1.0 | 972 |
| 1 | -4.0 at x 28 cm / PASS | 4.31 / PASS | 4.31 | 100% | 8/8 | 100% | 2.0 | 1.0 | 972 |
| 2 | -10.5 at x 35 cm / REFER | 10.50 / REFER | 10.64 | 100% | 8/8 | 100% | 2.0 | 1.0 | 972 |
| 3 | +5.0 at x 33 cm / PASS | 4.27 / PASS | 4.98 | 100% | 8/8 | 100% | 2.0 | 1.0 | 972 |
| 4 | +3.0 at x 29 cm / PASS | 2.61 / PASS | 2.96 | 100% | 8/8 | 100% | 2.0 | 1.0 | 972 |
| 5 | -8.5 at x 30 cm / REFER | 8.64 / REFER | 8.69 | 100% | 8/8 | 100% | 2.0 | 1.0 | 972 |
| 6 | +11.0 at x 27 cm / REFER | 10.38 / REFER | 11.01 | 100% | 8/8 | 100% | 2.2 | 1.0 | 972 |
| 7 | -5.5 at x 36 cm / PASS | 5.60 / PASS | 5.72 | 100% | 8/8 | 100% | 2.0 | 1.0 | 972 |

Baselines (same task file, 2,400 steps, `BASELINE_SEED`, `--no-record`). `naive` puts the slip in REFER without
measuring; `place` is the demo with the plate set down at x 31.5 cm and slid only 2 cm (a single placement).

| Run | Score | Gate | Span covered | Peak bins | Slip in |
|---|---|---|---|---|---|
| naive_0 | 0.0 | none | 0% | 0/8 | refer |
| noop_0 | 0.0 | none | 0% | 0/8 | none |
| noop_1 | 0.0 | none | 0% | 0/8 | none |
| place_0 | 0.0 | none | 12% | 3/8 | refer |
| random_0 | 0.0 | load on the back 20.5 N > 20 N | 0% | 0/8 | none |
| random_2 | 0.0 | load on the back 21.5 N > 20 N | 0% | 0/8 | none |
| reach_0 | 0.0 | none | 0% | 0/8 | none |
| reach_2 | 0.0 | load on the back 20.5 N > 20 N | 0% | 0/8 | none |

Failure modes seen while building, fixed in the model or the script, none by loosening a threshold:
- The first back was a staircase of rotated boxes (visible slats). It is now a smooth mesh over capsule colliders
  (reviewer point on anatomical form); the demo and scorer were re-run on it.
- Teleporting the seeded back into place left its sprung shell link behind for one control step and the joint
  dragged the back into the case; the reset now commits the teleport with one physics step.
- The left wrist camera first trailed only 13 cm behind the plate and its arm swept through the right gripper on the
  way in (the plate was knocked out of the fingers). The back was moved 5 cm further out and the camera now trails
  22 cm behind and above, and moves into place before the right arm.
- With an uncompensated hand the plate rolled about the finger pads and the notch swung 11 mm off the spine at a
  9.5 deg hump (coverage 68%, 0.25). The demo now shifts the pads sideways by 73 mm x sin(reading).
- Instantaneous readings overshot by up to 3 deg as the ball swung (13.5 deg read on a 10.5 deg back), and stray
  frames (another dark blob in the window) read 8-9 deg on a 5.0 deg back (seed 3 put the slip in REFER, 0.5). The
  demo now slides at 1.5 cm/s, takes the median of the last 9 frames, accepts only a round dark blob 5-20 mm below
  the red mark on the face plane, and drops a frame that jumps more than 2 deg from that median. All eight seeds
  were re-run with this final script.

## 6. Overlap check (live board search Nov 22: `gh search prs` for scoliometer, scoliosis, inclinometer, spine; board_titles.txt grep)

`gh search` hits: scoliometer none; scoliosis none; inclinometer #2415 tote lids, #2106 seedling (unrelated); spine
#3431, #3289, #1848, #3520, #1596, #2446, #2577, #3173, #2159, #2323, #298, #1421, #2769, #3376 (books, knives,
fish, binders; only #3520 is close, below). No board task screens a back or uses an inclinometer on a body.

| Task | Status | How ours differs |
|---|---|---|
| #1799 square a milling vise by sweeping its fixed jaw with an indicator | merged | a dial indicator on the machine spindle swept along a straight, flat jaw by a machine axis; ours is a hand-held two-foot inclinometer kept seated across a curved, rolling 3-D surface with the arm following height and roll, a second arm reading it by camera, and a referral decision from the reading |
| #3520 set a bookbinder's spine gauge on a rounded book block | closed | sets a gauge once on a book's rounded spine; ours is a continuous 25 cm sweep with contact kept at both feet throughout |
| #2392 level an analytical balance by its thumb-wheels / #2202 shim a floor sleeper until the bubble centres | closed / merged | adjust an object until a fixed bubble level reads zero; ours moves the level itself over a surface and records its peak |
| #1978 squeegee a wet upright glass pane, edge pressed at a working angle through each stroke | closed | sustained sliding contact of a blade on a flat pane; no instrument, no centring on a ridge, no reading or decision |
| #3198 temporal-artery thermometer sweep across a manikin's forehead | open | a short sweep of a probe tip across the forehead, one contact point; ours keeps a 19 cm two-foot plate seated and centred over a long curved path and reads a gauge |
| #2552 detector-wand sweep over a mannequin's arm | closed | non-contact sweep at a height; ours is contact at both feet |
| #2199 seat and level a surface-roughness drive and run a traverse | closed | the drive traverses by itself on a flat part; ours is the arm doing the traverse over a rolling surface |
| #3766 monofilament foot screen (ours) | pending | perpendicular presses at four discrete sites with a buckling filament; no sliding (sliding is forbidden there) |
| #3660 throat swab (ours) | closed | two tools, hold-aside plus a swab through a gap; nothing in common |
