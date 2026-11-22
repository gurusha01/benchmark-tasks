"""Diabetic foot sensation screen with a 10 g monofilament pen on a life-size foot model.

A rigid adult right-foot model stands on a sloped stand on the bench, its sole facing the robots and tipped back 30
deg. A Neuropen-class monofilament pen (16 mm barrel, 140 mm long, a 38 mm 5.07 / 10 g nylon filament out of its
nose) lies in a two-post cradle in front of the right arm. One of the four test sites carries a yellow callus.

The procedure (IWGDF 2019 / ADA annual foot exam): touch the filament to the plantar hallux and the 1st, 3rd and
5th metatarsal heads, perpendicular to the skin, pressing until the filament bends (about 1.5 to 2 s in all),
without sliding it; on a callus, test next to the callus, never on it. The sole is curved, so each site has its own
normal. The filament is a chain of four rigid links on sprung hinges: it buckles under a near-constant load.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import _ENV, Slider, _np, _part_state, on_reset, on_step, part, set_visible, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Screen the training manikin's foot with the monofilament: press it perpendicular to the sole until it bends, about "
               "1.5 s, at the big toe and 1st, 3rd and 5th metatarsal heads, beside any callus; then put the pen back.")


def srgb(r, g, b):
    return (r / 255, g / 255, b / 255)


SKIN, SKIN2, CALLUS_C, DARK, GREY = (srgb(224, 172, 145), srgb(212, 158, 132), srgb(226, 196, 84), srgb(45, 47, 50),
                                     srgb(120, 124, 128))
BARREL_C, NOSE_C, FIL_C = srgb(30, 90, 190), srgb(236, 236, 232), srgb(250, 250, 246)


def _nrm(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def _rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat_wxyz(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return tuple((q / np.linalg.norm(q)).tolist())


# ---- foot (frame F: u heel -> toes, v toward the little toe, w out of the sole toward the robots) -------------------
# The origin of F is the skin over the 3rd metatarsal head. The sole is tipped back ALPHA from vertical and the foot
# is turned FOOT_YAW about the vertical (the sole faces the robots' left front).
ALPHA = math.radians(30.0)
FOOT_YAW = math.radians(-30.0)
RF = _rz(FOOT_YAW) @ np.column_stack([[math.sin(ALPHA), 0.0, math.cos(ALPHA)], [0.0, 1.0, 0.0],
                                      [-math.cos(ALPHA), 0.0, math.sin(ALPHA)]])
FOOT_POS = (0.47, -0.03, TOP + 0.21)        # world position of F's origin (seed shifts it along y)


def _pad_rot(n):
    n = _nrm(n)
    a = _nrm(np.array([1.0, 0.0, 0.0]) - n[0] * n)
    return np.column_stack([a, np.cross(n, a), n])


_D22, _D25, _D20, _D10 = (math.radians(d) for d in (22, 25, 20, 10))
# site: (skin point in F, outward normal in F, pad length along the foot, pad width, pad thickness)
SITES = {
    "hallux": ((0.052, -0.028, -0.004), _nrm((math.tan(_D20), -math.tan(_D10), 1.0)), 0.040, 0.027, 0.020),
    "mth1": ((0.0, -0.034, -0.0069), _nrm((0.0, -math.sin(_D22), math.cos(_D22))), 0.050, 0.038, 0.030),
    "mth3": ((0.0, 0.0, 0.0), np.array([0.0, 0.0, 1.0]), 0.050, 0.036, 0.030),
    "mth5": ((-0.015, 0.036, -0.0089), _nrm((0.0, math.sin(_D25), math.cos(_D25))), 0.044, 0.042, 0.030),
}
SITE_NAMES = list(SITES)
CALLUS_R, CALLUS_H = 0.009, 0.0015          # yellow callus disc on the seeded site, centred on it


def _site_world_frame(name):
    """(skin point, rotation [a, b, n]) of a site in the foot part's frame (world axes, origin at F)."""
    s, n, *_ = SITES[name]
    return RF @ np.asarray(s, float), RF @ _pad_rot(n)


def _obox(c_f, size, color, R_f=None, visual=False):
    """A box given in F (centre, size, rotation in F) as a robokit box spec in the part frame."""
    R = RF @ (np.eye(3) if R_f is None else R_f)
    spec = (tuple((RF @ np.asarray(c_f, float)).tolist()), tuple(size), color, ("Q", _quat_wxyz(R)), None)
    return spec + (("visual",) if visual else ())


FOOT_BOXES, FOOT_COLL = [], []              # robokit specs; colliders (centre, rotation, size) for the scorer
for _nm, (_s, _n, _la, _lb, _t) in SITES.items():
    _R = _pad_rot(_n)
    _c = np.asarray(_s) - _R[:, 2] * _t / 2
    FOOT_BOXES.append(_obox(_c, (_la, _lb, _t), SKIN, _R))
for _c, _sz, _col in [((-0.005, 0.003, -0.048), (0.060, 0.090, 0.050), SKIN),       # forefoot core
                      ((-0.085, 0.008, -0.045), (0.100, 0.075, 0.060), SKIN),       # midfoot (arch recessed 15 mm)
                      ((-0.160, 0.006, -0.042), (0.060, 0.062, 0.072), SKIN),       # heel
                      ((0.050, -0.028, -0.030), (0.040, 0.026, 0.024), SKIN2),      # big toe behind its pad
                      ((-0.055, 0.005, -0.095), (0.230, 0.120, 0.020), DARK)]:      # sloped foot board
    FOOT_BOXES.append(_obox(_c, _sz, _col))
FOOT_COLL = [(RF @ np.asarray(b[0]) if False else np.asarray(b[0]), None, np.asarray(b[1])) for b in FOOT_BOXES]
FOOT_COLL = []
for _b in FOOT_BOXES:
    w, x, y, z = _b[3][1]
    _Rb = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    FOOT_COLL.append((np.asarray(_b[0]), _Rb, np.asarray(_b[1])))

# stand under the board: a column from the base plate up to the board's back face, and the base plate on the bench
_BACK = RF @ np.array([-0.055, 0.005, -0.105])
_ZB = TOP - FOOT_POS[2]                      # bench top in the part frame
_COL_H = _BACK[2] - (_ZB + 0.012) + 0.01
FOOT_BOXES += [((float(_BACK[0]), float(_BACK[1]), float(_ZB + 0.012 + _COL_H / 2)), (0.05, 0.05, _COL_H), DARK),
               ((float(_BACK[0]), float(_BACK[1]), float(_ZB + 0.006)), (0.22, 0.18, 0.012), DARK)]

# visual shapes: the four callus discs (only the seeded one is shown) first, so they are cyl_0 .. cyl_3
FOOT_CYL = []
for _nm in SITE_NAMES:
    _p, _R = _site_world_frame(_nm)
    FOOT_CYL.append((tuple((_p + _R[:, 2] * CALLUS_H / 2).tolist()), CALLUS_R, CALLUS_H, CALLUS_C, "Z", "visual",
                     _quat_wxyz(_R)))
_QF = _quat_wxyz(RF)
_QFX = _quat_wxyz(RF @ np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]))   # capsule Z axis along u
# lesser toes (colliders, capsules along u), then rounded skin over the boxes (visual)
TOES = [((0.040, 0.000, -0.019), 0.0075, 0.018), ((0.038, 0.017, -0.020), 0.0070, 0.015),
        ((0.034, 0.031, -0.021), 0.0065, 0.012), ((0.028, 0.044, -0.022), 0.0060, 0.010)]
for _c, _r, _l in TOES:
    FOOT_CYL.append((tuple((RF @ np.asarray(_c)).tolist()), _r, _l, SKIN, "Z", "capsule", _QFX))
for _c, _sc in [((-0.158, 0.006, -0.036), (0.034, 0.033, 0.036)),         # heel
                ((-0.060, 0.006, -0.055), (0.115, 0.046, 0.026)),         # dorsum
                ((0.060, -0.028, -0.022), (0.022, 0.0135, 0.012))]:       # tip of the big toe
    FOOT_CYL.append((tuple((RF @ np.asarray(_c)).tolist()), 1.0, 2.0, SKIN, "Z", "sphere_visual", _QF, _sc))

FOOT = part("foot", pos=FOOT_POS, mass=3.0, color=SKIN, friction=1.0, boxes=FOOT_BOXES, cylinders=FOOT_CYL)

# ---- monofilament pen in its cradle ---------------------------------------------------------------------------------
BARREL_L, BARREL_R = 0.140, 0.008
NOSE_X = 0.076                                # barrel tip (end of the nose) along the pen's axis from its centre
FIL_N, FIL_L, FIL_R = 4, 0.038, 0.0008        # 38 mm filament: 4 links, drawn and collided 1.6 mm thick
LINK = FIL_L / FIL_N
FIL_K = 1.0e-3                                # N m/rad per hinge: linear buckling load 0.098 N (10 g)
FIL_DAMP = 2.0e-4
FIL_PREBEND = 0.030                           # rad per hinge: the slight set a real filament has; it curves toward
                                              # the barrel's white stripe, tip 2.9 mm off the barrel's axis
CRADLE_X, CRADLE_Y = 0.28, -0.16
POST_DY = 0.058                               # posts at the cradle centre +-58 mm along the pen
RACK_Z = 0.037                                # post tops above the bench; the barrel's axis rests 45 mm up
PEN_Z = TOP + RACK_Z + BARREL_R + 0.0005

FILAMENT = tuple(
    Slider(f"f{i}", rest=(NOSE_X + i * LINK, 0.0, 0.0), kind="revolute", axis="Z", travel=1.4, both_ways=True,
           rate=FIL_K, damping=FIL_DAMP, mass=1.0e-4, inertia=(1e-8, 1e-8, 1e-8), start=FIL_PREBEND,
           parent="" if i == 0 else f"f{i - 1}",
           cylinders=(((NOSE_X + (i + 0.5) * LINK, 0.0, 0.0), FIL_R, LINK - 2 * FIL_R, FIL_C, "X", "capsule"),))
    for i in range(FIL_N))
PEN = part("pen", pos=(CRADLE_X, CRADLE_Y, PEN_Z), yaw=math.pi / 2, mass=0.015, color=BARREL_C, friction=1.0,
           boxes=[((0.0, BARREL_R + 0.0002, 0.0), (0.120, 0.0006, 0.004), NOSE_C, None, None, "visual")],   # stripe
           cylinders=[((0.0, 0.0, 0.0), BARREL_R, BARREL_L, BARREL_C, "X"),
                      ((0.073, 0.0, 0.0), 0.004, 0.006, NOSE_C, "X")],
           sliders=FILAMENT)


def _cradle_boxes():
    b = [((0.0, 0.0, 0.003), (0.040, 0.150, 0.006), DARK)]
    for dy in (-POST_DY, POST_DY):
        b += [((0.0, dy, RACK_Z / 2), (0.020, 0.010, RACK_Z), GREY)]
        for sx in (-1, 1):   # side lips: an 19 mm slot for the 16 mm barrel
            b += [((sx * 0.0115, dy, RACK_Z + 0.004), (0.004, 0.010, 0.008), GREY)]
    return b


CRADLE = part("cradle", pos=(CRADLE_X, CRADLE_Y, TOP), mass=0.6, color=GREY, friction=0.8, boxes=_cradle_boxes())
OBJECTS = [FOOT, PEN, CRADLE]

# ---- scoring --------------------------------------------------------------------------------------------------------
CONTROL_DT = 0.05
TOUCH = 0.0008                # filament tip within 0.8 mm of the skin = touching
SITE_TOL = 0.008              # landing point within 8 mm of the site
CALLUS_BAND = (CALLUS_R + 0.002, CALLUS_R + 0.008)   # on the callus site: 2 to 8 mm outside the callus edge
D_WINDOW = (0.024, 0.035)     # barrel tip to skin when buckled (straight filament: 38 mm)
ANGLE_MAX = 15.0              # deg between the pen's axis and the skin normal while buckled
SLIDE_MAX = 0.003
DURATION = (1.0, 3.0)
GAP_STEPS = 3                 # 0.15 s off the skin ends a contact
FOOT_MOVE_MAX = 0.020
HANDS_CLEAR = 0.05
_STATE: dict = {}


def _box_dist(p, s):
    d = np.abs(np.asarray(p)) - np.asarray(s) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)) + min(max(d[0], d[1], d[2]), 0.0))


def _link_pose(environment, name, link):
    base = environment.unwrapped
    asset = base.scene[name]
    names = list(asset.body_names)
    i = next(k for k, nm in enumerate(names) if nm == link or nm.endswith("/" + link))
    pose = _np(asset.data.body_link_pose_w)[0, i]
    origin = _np(base.scene.env_origins[0])
    x, y, z, w = pose[3:7]
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return pose[:3] - origin, R


def _tcps(environment):
    from yam_arena import ik, station

    out = {}
    for side in ("left", "right"):
        rob = environment.unwrapped.scene[f"{side}_robot"]
        q = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
        out[side] = ik.tcp_pose(q, station.BASES[side])[0]
    return out


def _seg_dist(p, a, b):
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def site_readings(foot, pen, tip):
    """Per site: the filament tip in the site's frame (along the foot, across, height of the tip's surface above the
    skin), the barrel tip's height above the skin plane and the angle (deg) between the pen's axis and the normal."""
    out = {}
    tip_l = foot.to_local(tip)
    bt_l = foot.to_local(pen.to_world((NOSE_X, 0.0, 0.0)))
    ax_l = foot.rotation().T @ pen.rotation()[:, 0]
    for nm in SITE_NAMES:
        p, R = _site_world_frame(nm)
        t = R.T @ (tip_l - p)
        bt = R.T @ (bt_l - p)
        ang = math.degrees(math.acos(float(np.clip(-np.dot(ax_l, R[:, 2]), -1.0, 1.0))))
        _, _, la, lb, _ = SITES[nm]
        inside = abs(t[0]) <= la / 2 + FIL_R and abs(t[1]) <= lb / 2 + FIL_R
        out[nm] = dict(a=float(t[0]), b=float(t[1]), gap=float(t[2]) - FIL_R, inside=inside, D=float(bt[2]), ang=ang)
    return out


def barrel_clearance(foot, pen) -> float:
    """Smallest distance (m) from the barrel's surface (and its nose) to the foot model's colliders."""
    best = 1.0
    for x in np.linspace(-BARREL_L / 2, NOSE_X, 16):
        r = BARREL_R if x <= BARREL_L / 2 else 0.004
        p = foot.to_local(pen.to_world((x, 0.0, 0.0)))
        for c, Rb, s in FOOT_COLL:
            best = min(best, _box_dist(Rb.T @ (p - c), s) - r)
    return best


def _finish(ep):
    nm = ep["site"]
    if nm in _STATE["results"]:
        return
    r = math.hypot(*ep["land"])
    if nm == _STATE["callus"]:
        land_ok = CALLUS_BAND[0] <= r <= CALLUS_BAND[1]
    else:
        land_ok = r <= SITE_TOL
    dur = ep["steps"] * CONTROL_DT
    checks = dict(land=land_ok, buckled=ep["minD"] <= D_WINDOW[1], not_jabbed=ep["minD"] >= D_WINDOW[0],
                  angle=ep["buckled_steps"] > 0 and ep["max_ang"] <= ANGLE_MAX, slide=ep["slide"] < SLIDE_MAX,
                  duration=DURATION[0] <= dur <= DURATION[1], one_site=not ep["switched"])
    _STATE["results"][nm] = dict(passed=all(checks.values()), land_mm=round(r * 1000, 1),
                                 min_barrel_to_skin_mm=round(ep["minD"] * 1000, 1),
                                 max_angle_deg=round(ep["max_ang"], 1), slide_mm=round(ep["slide"] * 1000, 2),
                                 duration_s=round(dur, 2), max_hinge_deg=round(ep["max_hinge"], 1),
                                 failed=[k for k, v in checks.items() if not v])


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(gate=None, results={}, ep=None, foot0=None, start=None, callus="mth1", tcp={}, last=None)
    if environment is None:
        return
    seed = _ENV.get("seed")
    rng = np.random.default_rng((int(seed) if seed is not None else 0) + 7151)
    callus = SITE_NAMES[int(rng.integers(4))]
    dy, dp = float(rng.uniform(-0.012, 0.012)), float(rng.uniform(-0.006, 0.006))
    _STATE.update(callus=callus, start=dict(callus=callus, foot_dy_mm=round(dy * 1000, 1),
                                            pen_dy_mm=round(dp * 1000, 1)))
    write_pose(environment, "foot", (FOOT_POS[0], FOOT_POS[1] + dy, FOOT_POS[2]), (0.0, 0.0, 0.0, 1.0))
    s2 = math.sin(math.pi / 4)
    write_pose(environment, "pen", (CRADLE_X, CRADLE_Y + dp, PEN_Z), (0.0, 0.0, s2, s2))
    for i, nm in enumerate(SITE_NAMES):
        set_visible("foot", f"body/cyl_{i}", nm == callus)


_reset()


@on_step
def _watch(environment) -> None:
    foot, pen = _part_state(environment, FOOT), _part_state(environment, PEN)
    if _STATE["foot0"] is None:
        _STATE["foot0"] = foot.pos.copy()
    _STATE["tcp"] = _tcps(environment)
    lp, lR = _link_pose(environment, "pen", f"f{FIL_N - 1}")
    tip = lp + lR @ np.array([LINK - FIL_R, 0.0, 0.0])
    rd = site_readings(foot, pen, tip)
    hinge = max(abs(v) for v in pen.sliders.values()) if pen.sliders else 0.0
    _STATE["last"] = dict(tip=tip, rd=rd, hinge=hinge)
    if _STATE["gate"]:
        return
    # gates: callus touched, barrel on the foot, pen off the bench, foot shoved
    c = rd[_STATE["callus"]]
    if math.hypot(c["a"], c["b"]) <= CALLUS_R + FIL_R and c["gap"] <= CALLUS_H + TOUCH:
        _STATE["gate"] = "filament touched the callus"
    elif barrel_clearance(foot, pen) <= 0.001:
        _STATE["gate"] = "pen barrel touched the foot"
    elif pen.pos[2] < TOP - 0.03:
        _STATE["gate"] = "pen fell off the bench"
    elif float(np.linalg.norm(foot.pos - _STATE["foot0"])) > FOOT_MOVE_MAX:
        _STATE["gate"] = "foot model moved more than 20 mm"
    if _STATE["gate"]:
        return
    touching = [nm for nm in SITE_NAMES if rd[nm]["inside"] and rd[nm]["gap"] <= TOUCH]
    ep = _STATE["ep"]
    if touching:
        nm = min(touching, key=lambda k: rd[k]["gap"])
        r = rd[nm]
        if ep is None:
            ep = _STATE["ep"] = dict(site=nm, steps=0, gap=0, land=(r["a"], r["b"]), slide=0.0, minD=1.0, max_ang=0.0,
                                     buckled_steps=0, switched=False, max_hinge=0.0)
        if nm != ep["site"]:
            ep["switched"] = True
        r = rd[ep["site"]]
        ep["steps"] += 1 + ep["gap"]
        ep["gap"] = 0
        ep["minD"] = min(ep["minD"], r["D"])
        ep["max_hinge"] = max(ep["max_hinge"], math.degrees(hinge))
        if r["D"] <= D_WINDOW[1]:   # while the filament is bent: slide from the landing point, and the angle
            ep["slide"] = max(ep["slide"], math.hypot(r["a"] - ep["land"][0], r["b"] - ep["land"][1]))
            ep["buckled_steps"] += 1
            ep["max_ang"] = max(ep["max_ang"], r["ang"])
    elif ep is not None:
        ep["gap"] += 1
        if ep["gap"] >= GAP_STEPS:
            _finish(ep)
            _STATE["ep"] = None


def _pen_in_cradle(pen) -> bool:
    a = pen.rotation()[:, 0]
    if abs(a[2]) > math.sin(math.radians(10)) or abs(a[1]) < 1e-6:
        return False
    if float(np.linalg.norm(pen.lin_vel)) > 0.01:
        return False
    cy = _STATE.get("cradle_y", CRADLE_Y)
    for dy in (-POST_DY, POST_DY):
        t = (cy + dy - pen.pos[1]) / a[1]
        if abs(t) > BARREL_L / 2 - 0.005:
            return False
        p = pen.pos + t * a
        if abs(p[0] - CRADLE_X) > 0.004 or abs(p[2] - PEN_Z) > 0.004:
            return False
    return True


def _hands_clear(pen) -> bool:
    a = pen.to_world((-BARREL_L / 2, 0.0, 0.0))
    b = pen.to_world((NOSE_X + FIL_L, 0.0, 0.0))
    return all(_seg_dist(np.asarray(t), a, b) >= HANDS_CLEAR for t in (_STATE.get("tcp") or {}).values())


def _update(objects):
    if _STATE.get("gate"):
        return 0.0, False
    n = sum(1 for r in _STATE.get("results", {}).values() if r["passed"])
    if n < 4:
        return 0.25 * n, False
    pen = objects["pen"]
    if _pen_in_cradle(pen) and _hands_clear(pen):
        return 1.0, True
    return 0.75, False


def success(objects) -> bool:
    return _update(objects)[1]


def score(objects) -> float:
    """Score 0 if at any point the filament touches the callus, the pen's barrel touches the foot, the pen falls off
    the bench, or the foot model moves more than 20 mm. Otherwise 0.25 for each site whose first contact was a correct
    test: the filament tip landed within 8 mm of the site (on the callus site: 2 to 8 mm outside the callus edge), the
    barrel tip came within 35 mm of the skin (the filament bent) but never closer than 24 mm, the pen's axis stayed
    within 15 deg of the skin normal and the tip within 3 mm of its landing point while bent, and the contact lasted
    1.0 to 3.0 s. A site's later contacts never count. Four sites score 0.75 until the pen lies back in its cradle
    with no gripper centre within 5 cm of it: then 1.0 (success, held 1 s). Doing nothing scores 0."""
    return _update(objects)[0]


def report(objects) -> dict:
    foot = objects["foot"]
    return {"start": _STATE.get("start"), "gate": _STATE.get("gate"), "callus_site": _STATE.get("callus"),
            "sites": _STATE.get("results"), "open_contact": _STATE.get("ep"),
            "pen_in_cradle": _pen_in_cradle(objects["pen"]), "hands_clear": _hands_clear(objects["pen"]),
            "foot_shift_mm": (round(float(np.linalg.norm(foot.pos - _STATE["foot0"])) * 1000, 1)
                              if _STATE.get("foot0") is not None else None),
            "score": _update(objects)[0]}


def debug_line(objects) -> str:
    L = _STATE.get("last")
    if not L:
        return "no reading"
    rd = L["rd"]
    near = min(SITE_NAMES, key=lambda k: math.hypot(rd[k]["a"], rd[k]["b"]))
    pen = objects["pen"]
    tc = (_STATE.get("tcp") or {}).get("right")
    r = rd[near]
    sl = objects["pen"].sliders
    return (f"near {near} tip a{r['a']*1000:.1f} b{r['b']*1000:.1f} gap{r['gap']*1000:.1f} D{r['D']*1000:.1f} "
            f"ang{r['ang']:.1f} hinges {[round(math.degrees(v),1) for v in sl.values()]} "
            f"ep {None if not _STATE.get('ep') else (_STATE['ep']['site'], _STATE['ep']['steps'])} "
            f"done {[(k, v['passed']) for k, v in _STATE.get('results', {}).items()]} gate {_STATE.get('gate')} "
            f"score {_update(objects)[0]} pen {np.round(pen.pos*1000,0).tolist()} ax {np.round(pen.rotation()[:,0],2).tolist()} "
            f"tcp {None if tc is None else np.round(tc*1000,0).tolist()}")
