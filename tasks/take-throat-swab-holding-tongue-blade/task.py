"""Take a throat swab for a strep culture from an oral swab training head, holding the tongue down with a tongue blade.

An adult oral and nasal swab training head (CAE Medicor class: teeth, tongue, uvula, tonsillar pillars) stands on
the bench facing the robots, mouth open. A wooden tongue blade and a culture swab (BD BBL CultureSwab class: rayon
tip on a plastic shaft, held by its cap) lie on an instrument rest between the arms.

The procedure (throat culture collection): depress the tongue with the blade, pass the swab over the tongue without
touching it, rub both tonsillar pillars (or the back wall of the throat), then withdraw the swab without touching the
tongue, teeth, cheeks or uvula, and only then take the blade out. A swab tip that touches anything else in or on the
mouth is contaminated: the culture is void. The tongue is a sprung body: unless the blade holds it down it rises
and blocks the way to the pillars.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import _ENV, Slider, _part_state, on_reset, on_step, part, set_target, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Take a throat swab from the manikin: hold its tongue down with the tongue blade, rub both tonsillar "
               "pillars with the swab without touching anything else, then take the swab out, then the blade.")


def srgb(r, g, b):
    return (r / 255, g / 255, b / 255)


SKIN, MUCOSA, TONGUE_C, PILLAR, TOOTH, GREY, DARK = (srgb(224, 172, 145), srgb(214, 120, 128), srgb(201, 104, 114),
                                                     srgb(176, 38, 52), srgb(238, 236, 226), srgb(120, 124, 128),
                                                     srgb(45, 47, 50))
WOOD, SWAB_TIP, SHAFT, CAP = srgb(222, 199, 158), srgb(246, 246, 240), srgb(236, 236, 236), srgb(30, 90, 190)

# ---- training head (frame: origin at the centre of the mouth opening on the lip plane; +x into the mouth) -------
XF = 0.46                 # lip plane, world x
ZM = TOP + 0.23           # centre of the mouth opening, world z
MOUTH_W, MOUTH_H = 0.050, 0.040
UPPER_TEETH_Z, LOWER_TEETH_Z = 0.016, -0.016          # edges of the incisors
CAV_Y = 0.028             # inner cheek
PILLAR_X = (0.075, 0.100)                             # anterior tonsillar pillar (palatoglossal arch) and tonsil bed
PILLAR_Y = 0.016          # inner face of each pillar, |y|; the oropharynx between them is 32 mm wide
BACK_X = 0.100            # posterior pharyngeal wall
UVULA = ((0.084, 0.0), 0.004, (0.004, 0.018))         # (x, y), radius, z range
TONGUE_TOP_UP, TONGUE_TRAVEL = 0.002, 0.016           # tongue's dorsum at rest; it can be pressed 16 mm down
TONGUE_RATE = 40.0        # N/m (estimate): about 0.6 N to start it moving, 1.3 N to hold it flat
TONGUE_PRELOAD = 0.016    # m of spring wind-up at rest
X_GAG = 0.066             # the posterior third of the tongue (dorsum x 0.016-0.090) starts here: blade beyond it gags

_D = ZM - TOP             # mouth centre above the bench
HEAD_BOXES = [
    # face around the mouth opening (the lips)
    ((-0.006, 0.0, 0.075), (0.012, 0.16, 0.11), SKIN),
    ((-0.006, 0.0, -0.055), (0.012, 0.16, 0.07), SKIN),
    ((-0.006, 0.0525, 0.0), (0.012, 0.055, 0.04), SKIN),
    ((-0.006, -0.0525, 0.0), (0.012, 0.055, 0.04), SKIN),
    # incisors
    ((0.007, 0.0, UPPER_TEETH_Z + 0.004), (0.010, 0.044, 0.008), TOOTH),
    ((0.007, 0.0, LOWER_TEETH_Z - 0.005), (0.010, 0.044, 0.010), TOOTH),
    # hard and soft palate, cheeks, floor
    ((0.036, 0.0, 0.028), (0.072, 0.080, 0.012), MUCOSA),
    ((0.0885, 0.0, 0.026), (0.033, 0.080, 0.016), MUCOSA),
    ((0.0375, 0.034, -0.003), (0.075, 0.012, 0.074), MUCOSA),
    ((0.0375, -0.034, -0.003), (0.075, 0.012, 0.074), MUCOSA),
    ((0.0525, 0.0, -0.046), (0.105, 0.080, 0.012), MUCOSA),
    # tonsillar pillars (left = +y, right = -y) and the back wall of the throat
    ((0.0875, 0.028, -0.003), (0.025, 0.024, 0.074), PILLAR),
    ((0.0875, -0.028, -0.003), (0.025, 0.024, 0.074), PILLAR),
    ((0.106, 0.0, -0.003), (0.012, 0.080, 0.074), MUCOSA),
    # rest of the head, neck and base
    ((0.079, 0.0, 0.082), (0.182, 0.16, 0.096), SKIN),
    ((0.085, 0.06, -0.028), (0.17, 0.04, 0.124), SKIN),
    ((0.085, -0.06, -0.028), (0.17, 0.04, 0.124), SKIN),
    ((0.141, 0.0, -0.028), (0.058, 0.08, 0.124), SKIN),
    ((0.05, 0.0, -0.071), (0.124, 0.08, 0.038), SKIN),
    ((0.07, 0.0, (-0.09 - _D + 0.012) / 2), (0.08, 0.08, _D - 0.012 - 0.09), GREY),
    ((0.08, 0.0, -_D + 0.006), (0.18, 0.20, 0.012), DARK),
]
HEAD_CYL = [((UVULA[0][0], UVULA[0][1], sum(UVULA[2]) / 2), UVULA[1], UVULA[2][1] - UVULA[2][0], MUCOSA, "Z")]
# the tongue's dorsum at rest; the slider frame sits at its top centre
_TZ = TONGUE_TOP_UP
TONGUE = Slider("tongue", rest=(0.053, 0.0, _TZ), kind="prismatic", axis="Z", sign=-1, travel=TONGUE_TRAVEL,
                rate=TONGUE_RATE, damping=2.0, mass=0.03,
                boxes=(((0.044, 0.0, _TZ - 0.019), (0.056, 0.044, 0.038), TONGUE_C),
                       ((0.081, 0.0, _TZ - 0.019), (0.018, 0.029, 0.038), TONGUE_C)))
HEAD = part("head", pos=(XF, 0.0, ZM), mass=6.0, color=SKIN, friction=1.0, boxes=HEAD_BOXES, cylinders=HEAD_CYL,
            sliders=(TONGUE,))

# forbidden and allowed surfaces for the swab tip (head frame boxes: centre, size); the tongue moves with its slider
FORBID = [b[:2] for b in HEAD_BOXES[:11]]
PILLAR_L, PILLAR_R, BACK = HEAD_BOXES[11][:2], HEAD_BOXES[12][:2], HEAD_BOXES[13][:2]
CAVITY = (np.array([-0.012, -CAV_Y, -0.040]), np.array([BACK_X, CAV_Y, 0.034]))

# ---- tongue blade and culture swab ---------------------------------------------------------------------------------
BLADE_L, BLADE_W, BLADE_T = 0.152, 0.018, 0.0018      # 6 x 3/4 in wooden tongue depressor
SWAB_TIP_X = (0.070, 0.085)                            # rayon tip, 5 mm across, on a 2.5 mm shaft; cap 12 mm across
SWAB_TIP_R = 0.0025
RACK_Z = 0.140                                         # top of the instrument stand's posts above the bench
BLADE_Y, SWAB_Y = 0.12, -0.09
BLADE_END_X = 0.260                                    # handle end of the blade, world x
CAP_END_X = 0.250                                      # end of the swab's cap, world x
RACK_X = 0.325                                         # centre of the instrument rest, world x

BLADE = part("blade", pos=(BLADE_END_X + BLADE_L / 2, BLADE_Y, TOP + RACK_Z + BLADE_T / 2 + 0.0005), mass=0.004,
             color=WOOD, friction=0.9, boxes=[((0.0, 0.0, 0.0), (BLADE_L, BLADE_W, BLADE_T), WOOD)])
# the cap is a hollow tube, so the swab balances near the middle of its shaft: centre of mass at the body origin
SWAB = part("swab", pos=(CAP_END_X + 0.085, SWAB_Y, TOP + RACK_Z + 0.00125 + 0.0005), mass=0.004, color=SHAFT,
            friction=0.9, inertia=(8e-8, 1.0e-5, 1.0e-5), ang_damping=2.0, boxes=[((-0.0725, 0.0, 0.0), (0.0005, 0.0005, 0.0005), CAP, None, None, "visual")],
            cylinders=[((-0.0725, 0.0, 0.0), 0.006, 0.025, CAP, "X"),
                       ((0.005, 0.0, 0.0), 0.00125, 0.130, SHAFT, "X"),
                       ((0.0775, 0.0, 0.0), SWAB_TIP_R, 0.015, SWAB_TIP, "X")])


def _rack_boxes():
    b = [((0.0, 0.0, 0.004), (0.20, 0.36, 0.008), DARK)]
    for x in (0.335, 0.405):        # blade posts with side lips (the blade's middle, 34 cm, lies between them)
        b += [((x - RACK_X, BLADE_Y, RACK_Z / 2), (0.012, 0.030, RACK_Z), GREY),
              ((x - RACK_X, BLADE_Y + 0.0125, RACK_Z + 0.002), (0.012, 0.004, 0.004), GREY),
              ((x - RACK_X, BLADE_Y - 0.0125, RACK_Z + 0.002), (0.012, 0.004, 0.004), GREY)]
    for x in (0.31, 0.39):          # swab posts with a 4 mm slot
        b += [((x - RACK_X, SWAB_Y, RACK_Z / 2), (0.012, 0.020, RACK_Z), GREY),
              ((x - RACK_X, SWAB_Y + 0.004, RACK_Z + 0.002), (0.012, 0.004, 0.004), GREY),
              ((x - RACK_X, SWAB_Y - 0.004, RACK_Z + 0.002), (0.012, 0.004, 0.004), GREY)]
    return b


RACK = part("rack", pos=(RACK_X, 0.0, TOP), mass=1.5, color=DARK, friction=1.0, boxes=_rack_boxes())
OBJECTS = [HEAD, BLADE, SWAB, RACK]

# ---- scoring -------------------------------------------------------------------------------------------------------
TOUCH = 0.0008            # tip surface within 0.8 mm of a surface = touching (contact offset 0.5 mm)
HOLD_STEPS = 2            # a pillar counts as swabbed after 2 control steps (0.1 s) in contact
_STATE: dict = {}


def _box_dist(p, c, s):
    d = np.abs(np.asarray(p) - np.asarray(c)) - np.asarray(s) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)) + min(max(d[0], d[1], d[2]), 0.0))


def _tip_pts(head, swab):
    return [head.to_local(swab.to_world((x, 0.0, 0.0))) for x in np.linspace(SWAB_TIP_X[0], SWAB_TIP_X[1], 7)]


def _blade_pts(head, blade):
    return [head.to_local(blade.to_world((sx * BLADE_L / 2, sy * BLADE_W / 2, 0.0)))
            for sx in (-1, 1) for sy in (-1, 0, 1)]


def _tongue_boxes(head):
    dz = head.sliders.get("tongue", 0.0)
    return [((TONGUE.rest[0] + b[0][0] - TONGUE.rest[0], b[0][1], b[0][2] + dz), b[1]) for b in TONGUE.boxes]


def tip_contacts(objects) -> dict:
    """Clearance (m) from the swab tip's surface to each surface group: forbidden (lips, teeth, palate, cheeks,
    floor, uvula, tongue, blade, bench), the left and right pillars, the back wall."""
    head, swab, blade = objects["head"], objects["swab"], objects["blade"]
    pts = _tip_pts(head, swab)
    def near(boxes):
        return min(_box_dist(p, c, s) for p in pts for c, s in boxes) - SWAB_TIP_R
    (ux, uy), ur, (uz0, uz1) = UVULA
    uv = min(math.hypot(max(math.hypot(p[0] - ux, p[1] - uy) - ur, 0.0), max(uz0 - p[2], p[2] - uz1, 0.0))
             for p in pts) - SWAB_TIP_R
    bl = min(_box_dist(blade.to_local(swab.to_world((x, 0.0, 0.0))), (0, 0, 0), (BLADE_L, BLADE_W, BLADE_T))
             for x in np.linspace(SWAB_TIP_X[0], SWAB_TIP_X[1], 7)) - SWAB_TIP_R
    bench = min(float(swab.to_world((x, 0.0, 0.0))[2]) - TOP for x in np.linspace(*SWAB_TIP_X, 7)) - SWAB_TIP_R
    return {"forbid": min(near(FORBID), near(_tongue_boxes(head)), uv, bl, bench),
            "left": near([PILLAR_L]), "right": near([PILLAR_R]), "back": near([BACK])}


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(gate=None, left=0, right=0, left_ok=False, right_ok=False, start=None, h0=None, max_press=0.0)
    if environment is None:
        return
    set_target(environment, "head", "tongue", TONGUE_PRELOAD)
    seed = _ENV.get("seed")
    if seed is None:
        return
    # where each tool lies along its rest varies per trial: 0 to 12 mm further from the robots (seeded)
    rng = np.random.default_rng(int(seed) + 4243)
    db, ds = rng.uniform(0.0, 0.012), rng.uniform(0.0, 0.012)
    _STATE["start"] = (round(db, 4), round(ds, 4))
    write_pose(environment, "blade", (BLADE.pos[0] + db, BLADE.pos[1], BLADE.pos[2]), (0.0, 0.0, 0.0, 1.0))
    write_pose(environment, "swab", (SWAB.pos[0] + ds, SWAB.pos[1], SWAB.pos[2]), (0.0, 0.0, 0.0, 1.0))


_reset()


@on_step
def _watch(environment) -> None:
    set_target(environment, "head", "tongue", TONGUE_PRELOAD)
    objs = {o.name: _part_state(environment, o) for o in (HEAD, BLADE, SWAB)}
    head, blade = objs["head"], objs["blade"]
    if _STATE["h0"] is None:
        _STATE["h0"] = head.pos.copy()
    _STATE["max_press"] = min(_STATE["max_press"], head.sliders.get("tongue", 0.0))
    if _STATE["gate"]:
        return
    c = tip_contacts(objs)
    if c["forbid"] <= TOUCH:
        _STATE["gate"] = "swab tip touched the mouth outside the pillars (contaminated)"
        return
    for side in ("left", "right"):
        _STATE[side] = _STATE[side] + 1 if c[side] <= TOUCH else 0
        if _STATE[side] >= HOLD_STEPS:
            _STATE[f"{side}_ok"] = True
    # the blade on the back third of the tongue, or on the uvula or throat, triggers the gag reflex
    lo, hi = CAVITY
    for p in _blade_pts(head, blade):
        if p[0] > X_GAG and np.all(p >= lo) and np.all(p <= hi):
            _STATE["gate"] = "tongue blade on the back third of the tongue (gag)"
            return


def _in_cavity(p) -> bool:
    lo, hi = CAVITY
    return bool(np.all(np.asarray(p) >= lo) and np.all(np.asarray(p) <= hi))


def _update(objects):
    if _STATE.get("gate"):
        return 0.0, False
    head, swab, blade = objects["head"], objects["swab"], objects["blade"]
    n = int(_STATE.get("left_ok", False)) + int(_STATE.get("right_ok", False))
    if n == 0:
        return 0.0, False
    if n == 1:
        return 0.25, False
    tip = _tip_pts(head, swab)
    held_up = min(float(swab.to_world((x, 0.0, 0.0))[2]) for x in SWAB_TIP_X) - TOP > 0.03
    swab_out = all(p[0] < -0.02 for p in tip) and held_up and tip_contacts(objects)["forbid"] > TOUCH
    if not swab_out:
        return 0.5, False
    blade_out = not any(_in_cavity(p) for p in _blade_pts(head, blade))
    if not blade_out:
        return 0.75, False
    return 1.0, True


def success(objects) -> bool:
    return _update(objects)[1]


def score(objects) -> float:
    """0 after a gate: the swab tip touched the lips, teeth, palate, cheeks, floor of the mouth, uvula, tongue, the
    tongue blade or the bench (contaminated), or the blade reached the back third of the tongue (gag). Otherwise, at
    the end: 0.25 when the swab tip has touched one tonsillar pillar (0.1 s in contact), 0.5 both; 0.75 both and the
    swab is out of the mouth (tip 2 cm or more in front of the lips, 3 cm or more above the bench, touching nothing);
    1.0 (success, held 1 s) that, and no part of the blade left in the mouth. Doing nothing scores 0."""
    return _update(objects)[0]


def report(objects) -> dict:
    c = tip_contacts(objects)
    return {"start_offset_m": _STATE.get("start"), "gate": _STATE.get("gate"),
            "left_pillar": _STATE.get("left_ok"), "right_pillar": _STATE.get("right_ok"),
            "tip_clearance_mm": {k: round(v * 1000, 2) for k, v in c.items()},
            "max_tongue_press_mm": round(-_STATE.get("max_press", 0.0) * 1000, 1),
            "head_shift_mm": (round(float(np.linalg.norm(objects["head"].pos - _STATE["h0"])) * 1000, 1)
                              if _STATE.get("h0") is not None else None),
            "score": _update(objects)[0]}


def debug_line(objects) -> str:
    c = tip_contacts(objects)
    h = objects["head"]
    tip = _tip_pts(h, objects["swab"])[-1]
    bt = h.to_local(objects["blade"].to_world((BLADE_L / 2, 0.0, 0.0)))
    return (f"tongue {h.sliders.get('tongue', 0)*1000:.1f}mm tip {np.round(np.array(tip)*1000,1)} "
            f"clear {{{', '.join(f'{k}:{v*1000:.1f}' for k, v in c.items())}}} blade_tip {np.round(bt*1000,1)} "
            f"L{_STATE.get('left_ok')} R{_STATE.get('right_ok')} gate {_STATE.get('gate')} score {_update(objects)[0]}")
