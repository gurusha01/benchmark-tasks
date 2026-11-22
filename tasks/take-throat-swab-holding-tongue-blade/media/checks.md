# Checks for the throat-swab task

All runs on one TITAN RTX, Isaac Lab-Arena 0.3.1 with yam_arena, 2,400-step limit, 20 Hz, one seed per process.

## 1. Real job, real procedure, real objects

| Item | Value in the scene | Source |
|---|---|---|
| Who does it | Medical assistants, O*NET 31-9092.00: about 811,000 US jobs (2024), 57% in physician offices | [BLS OOH, medical assistants](https://www.bls.gov/ooh/healthcare/medical-assistants.htm); O*NET task "collect blood, tissue, or other laboratory specimens" |
| Procedure | depress the tongue, rub both tonsillar pillars or the back of the throat, keep the swab off the tongue, teeth, cheeks and uvula; other sites give false negatives | [Pennsylvania DoH, How to collect a throat swab for culture](https://www.health.pa.gov/topics/Documents/Programs/HAIP-AS/How%20to%20Collect%20a%20Throat%20Swab%20for%20Culture.pdf); [CDC clinical specimen guidelines (oropharyngeal swab)](https://www.cdc.gov/covid/hcp/clinical-care/clinical-specimen-guidelines.html) |
| Training head | adult oral and nasal swab simulator with teeth, tongue and uvula, about $750 to $1,000 | [CAE Medicor oral and nasal swab simulator](https://medicalskillstrainers.cae.com/oral-and-nasal-swab-simulator/p); [3B Scientific listing](https://3bscientific.com/cr/oral-and-nasal-swab-simulator-medium-3017144-b-tp-001-m,p_153_32623.html) |
| Mouth opening 5 x 4 cm, incisor gap 3.2 cm, pillars 7.5 cm in and 3.2 cm apart, uvula 8.4 cm in | estimates from adult oral anatomy | not published for the simulator |
| Tongue spring 0.6 to 1.3 N over 16 mm | estimate | not published |
| Tongue depressor 6 x 3/4 in (15.2 x 1.8 cm), 1.8 mm thick, 4 g | standard wooden depressor; mass estimate | e.g. Puritan or McKesson 6 in senior depressors |
| Culture swab 17 cm, 12 mm cap, 2.5 mm shaft, 5 mm rayon tip | BD BBL CultureSwab class; sizes and 4 g mass estimated | BD product line |

## 2. The tongue and the scoring are physical

There is no latch or proximity trigger. The tongue is a block on a prismatic joint with a spring: it rises 2 mm above
the mouth's centre and closes the way to the pillars unless the blade holds it down. Pillar touches and forbidden
touches are measured from geometry each control step: the swab tip (a 5 mm cylinder, 15 mm long) counts as touching a
surface when it is within 0.8 mm of it (the contact offset is 0.5 mm). A pillar counts as swabbed after 0.1 s in
contact. The gag gate is the blade's inner end more than 66 mm behind the lips (the back third of a tongue that runs
from 16 to 90 mm).

## 3. Demo and baselines in the simulator

Scripted demo, final task and script, one process per seed (the seed shifts the blade and the swab along their rests):

| Seed | Start offset blade / swab (mm) | Camera: blade end / cap middle x (mm; true) | Blade tip held at (head frame, mm, from joints) | Score | Steps to success |
|---|---|---|---|---|---|
| 0 | 6.6 / 9.4 | 267.8 / 270.2 (266.6 / 271.9) | 40.3, -12.1, -12.7 | 1.0 | 1191 |
| 1 | 1.6 / 4.1 | 262.4 / 264.8 (261.6 / 266.6) | 39.8, -13.2, -13.5 | 1.0 | 1178 |
| 2 | 5.6 / 11.8 | 267.8 / 272.9 (265.6 / 274.3) | 39.5, -12.8, -13.6 | 1.0 | 1158 |
| 3 | 1.0 / 10.6 | 262.4 / 270.2 (261.0 / 273.1) | 40.1, -12.6, -12.7 | 1.0 | 1185 |
| 4 | 5.3 / 3.7 | 267.8 / 264.8 (265.3 / 266.2) | 40.1, -12.5, -13.0 | 1.0 | 1163 |
| 5 | 11.9 / 1.5 | 273.2 / 262.1 (271.9 / 264.0) | 40.3, -12.4, -13.1 | 1.0 | 1172 |
| 6 | 10.8 / 11.2 | 273.2 / 272.9 (270.8 / 273.7) | 40.2, -12.3, -12.3 | 1.0 | 1188 |
| 7 | 8.2 / 3.3 | 270.5 / 262.1 (268.2 / 265.8) | 40.3, -11.9, -13.9 | 1.0 | 1158 |

8 of 8 seeds succeed at 1158 to 1191 steps (58 to 60 s of the 120 s limit). In every run the tongue was pressed its
full 16 mm while the swab was in the mouth, both pillars were swabbed, no forbidden touch occurred, and the head did
not move.

Baselines (2,400 steps each, `--no-record`):

| Run | Score | Pillars touched | Tongue pressed (mm) |
|---|---|---|---|
| Do nothing, seed 0 / 1 | 0.0 / 0.0 | none | 0 |
| Random joint targets, seeds 0 / 1 / 2 | 0.0 / 0.0 / 0.0 | none | 0 |
| Random reaches and grasps, seeds 0 / 1 / 2 | 0.0 / 0.0 / 0.0 | none | 1.0 / 0 / 0.9 |

No baseline reached any rung. One random-reach run shoved the 6 kg head 33 cm across the bench (seed 1); the head
is free on the bench, and moving it is not scored. (The first attempts of random reaches seeds 0 and 1 ended without
writing a result and with no error in the log; both were re-run and completed normally.)

## 4. The demo uses only observable inputs

`script.txt` reads: the first top-camera image (`ctx.obs.images["top_cam"]`, to find the blade's handle end and the
swab cap's middle on their rests), the arms' joint angles through forward kinematics (`fk`), and the public layout
constants in `task.py` (lip plane, mouth height, rest height, tool sizes). It never calls `ctx.states()`, the scorer
or the tongue's joint: `grep -nE "states\(|tip_contacts|sliders|_STATE|_part_state" script.txt` finds nothing. The arm
sags 2 to 5 mm under load, so the demo re-aims on each tool's tip worked out from the measured joints; and because how
the light swab sits in the fingers is not observable, it runs the swab where it has about 10 mm of room above and
below rather than relying on millimetre accuracy.

## 5. Real-trial check

Dot the left pillar blue and the right pillar green with washable marker; dust the tongue, uvula, cheeks, teeth,
palate and the upper face of the blade with UV tracer powder (Glo Germ). Score from the swab tip after the trial: blue
and green present means both pillars swabbed; any UV glow means a forbidden touch (0). Draw a line across the tongue
two thirds of the way back: the blade's end past it is a gag (0). A side camera shows the order (swab out before the
blade) and that the swab ends out of the mouth.

## 6. Overlap with the board

Searched all 3,272 PR titles and bodies for throat, tonsil, tongue depressor/blade, pharynx, strep, swab, oral care
and manikin mouth tasks; read the closest by hand.

| Task | Status | How ours differs |
|---|---|---|
| #1951 Wring a strep-test swab in its tube | merged | the lab step after collection; no mouth, no tongue blade |
| #2235 Snap a specimen swab's handle at its breakpoint in a transport tube | merged | transport tube handling; no mouth |
| #2297 Take an anterior nasal swab | open, not accepted | nostril, one arm, no tongue held down |
| #3162 Give a child liquid medicine with an oral syringe | open | syringe into the cheek pocket; no blade, no pillars |
| #1160 Dental rinse and suction in a typodont mouth | open | bulb syringe and saliva ejector; no tongue blade, no swab |
| #2892 Floss a dental training head's lower front teeth | open | floss pick between incisors |
