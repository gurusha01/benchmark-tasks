"""Diabetic foot sensation screen with a 10 g monofilament pen on a life-size foot model.

A rigid life-size adult right-foot model (a smooth sculpted triangle mesh: heel, medial arch, metatarsal-head pads,
five toes with pulps; built by make_foot.py) is bolted at its ankle cut to a sloped stand on the bench, its sole
facing the robots and tipped back 30 deg. A Neuropen-class monofilament pen (16 mm barrel, 140 mm long, a 38 mm 5.07 / 10 g nylon filament out of its
nose) lies in a two-post cradle in front of the right arm. One of the four test sites carries a yellow callus.

The procedure (IWGDF 2019 / ADA annual foot exam): touch the filament to the plantar hallux and the 1st, 3rd and
5th metatarsal heads, perpendicular to the skin, pressing until the filament bends (about 1.5 to 2 s in all),
without sliding it; on a callus, test next to the callus, never on it. The sole is curved, so each site has its own
normal. The filament is a chain of four rigid links on sprung hinges: it buckles under a near-constant load.
Touches, depths and angles are measured against the foot's skin mesh.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scipy.spatial import ConvexHull, cKDTree  # noqa: E402
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
    """Site frame [a, b, n]: a along the foot (heel -> toes) in the skin's tangent plane, n the outward normal."""
    n = _nrm(n)
    a = _nrm(np.array([1.0, 0.0, 0.0]) - n[0] * n)
    return np.column_stack([a, np.cross(n, a), n])


_FM = np.load(pathlib.Path(__file__).resolve().parent / "foot_mesh.npz")   # built by make_foot.py (in F, metres)
SITE_NAMES = [str(k) for k in _FM["site_names"]]                          # hallux, mth1, mth3, mth5
SITES = {k: (np.asarray(p, float), _nrm(n)) for k, p, n in zip(SITE_NAMES, _FM["site_p"], _FM["site_n"])}
SITE_ZONE = 0.018            # a touch belongs to the nearest site within 18 mm (along the skin's tangent plane)
CALLUS_R, CALLUS_H = 0.009, 0.0015          # yellow callus patch on the seeded site, centred on it
FOOT_V = _FM["verts"].astype(float) @ RF.T  # skin mesh in the foot part's frame (world axes, origin at F)
FOOT_N = _FM["normals"].astype(float) @ RF.T
_TREE = cKDTree(FOOT_V)


def _site_world_frame(name):
    """(skin point, rotation [a, b, n]) of a site in the foot part's frame (world axes, origin at F)."""
    s, n = SITES[name]
    return RF @ s, RF @ _pad_rot(n)


def skin_dist(p_local):
    """Signed distance (m, negative inside) from a point in the foot part's frame to the skin mesh (1.2 mm vertex
    spacing), and the skin's outward normal at the nearest vertex. Near the skin the distance is taken along that
    normal, which is exact to about 0.05 mm on this mesh."""
    p_local = np.asarray(p_local, float)
    _, i = _TREE.query(p_local)
    dv = p_local - FOOT_V[i]
    s = float(np.dot(dv, FOOT_N[i]))
    e = float(np.linalg.norm(dv))
    return (s if e < 0.004 else math.copysign(e, s)), FOOT_N[i]


def skin_point(p_local, iters=4):
    """The skin point nearest ``p_local`` (foot part frame) and the skin's outward normal there."""
    p = np.asarray(p_local, float)
    for _ in range(iters):
        d, n = skin_dist(p)
        p = p - d * n
    return p, skin_dist(p)[1]


def _obox(c_f, size, color, R_f=None, visual=False):
    """A box given in F (centre, size, rotation in F) as a robokit box spec in the part frame."""
    R = RF @ (np.eye(3) if R_f is None else R_f)
    spec = (tuple((RF @ np.asarray(c_f, float)).tolist()), tuple(size), color, ("Q", _quat_wxyz(R)), None)
    return spec + (("visual",) if visual else ())


# stand: the sloped board the foot's ankle cut is bolted to (in F), a column under the board, and the base plate
FOOT_BOXES = [_obox((-0.055, 0.005, -0.095), (0.230, 0.120, 0.020), DARK)]
_BACK = RF @ np.array([-0.055, 0.005, -0.105])
_ZB = TOP - FOOT_POS[2]                      # bench top in the part frame
_COL_H = _BACK[2] - (_ZB + 0.012) + 0.01
FOOT_BOXES += [((float(_BACK[0]), float(_BACK[1]), float(_ZB + 0.012 + _COL_H / 2)), (0.05, 0.05, _COL_H), DARK),
               ((float(_BACK[0]), float(_BACK[1]), float(_ZB + 0.006)), (0.22, 0.18, 0.012), DARK)]
FOOT_COLL = []                               # the stand's boxes (centre, rotation, size) for the barrel check
for _b in FOOT_BOXES:
    _q = _b[3][1] if len(_b) > 3 and _b[3] else (1.0, 0.0, 0.0, 0.0)
    w, x, y, z = _q
    _Rb = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    FOOT_COLL.append((np.asarray(_b[0]), _Rb, np.asarray(_b[1])))


def _hull(pts_f):
    P = np.asarray(pts_f, float) @ RF.T
    return P, ConvexHull(P).simplices


# meshes: the skin (drawn), the four callus patches (drawn; only the seeded one is shown), then the colliders: convex
# tiles on the plantar skin and convex core slabs (not drawn)
FOOT_MESHES = [("skin", FOOT_V, _FM["faces"], SKIN, "visual", FOOT_N)]
for _i, _nm in enumerate(SITE_NAMES):
    FOOT_MESHES.append((f"callus_{_i}", _FM[f"callus_v_{_nm}"].astype(float) @ RF.T, _FM[f"callus_f_{_nm}"], CALLUS_C,
                        "visual"))
for _i, _t in enumerate(_FM["tiles"]):
    FOOT_MESHES.append((f"tile_{_i}", *_hull(_t), SKIN, "convex"))
_o = 0
for _i, _k in enumerate(_FM["core_len"]):
    FOOT_MESHES.append((f"core_{_i}", *_hull(_FM["core"][_o:_o + _k]), SKIN, "convex"))
    _o += _k

FOOT = part("foot", pos=FOOT_POS, mass=3.0, color=SKIN, friction=1.0, boxes=FOOT_BOXES, meshes=FOOT_MESHES)

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
    """The filament tip's gap to the skin and the skin normal under it, the barrel tip's distance to the skin, the
    pen's axis, and per site the tip's position in the site's tangent plane (along the foot, across, radius)."""
    tip_l = foot.to_local(tip)
    gap, n_tip = skin_dist(tip_l)
    D = skin_dist(foot.to_local(pen.to_world((NOSE_X, 0.0, 0.0))))[0]
    ax_l = foot.rotation().T @ pen.rotation()[:, 0]
    out = {}
    for nm in SITE_NAMES:
        p, R = _site_world_frame(nm)
        t = R.T @ (tip_l - p)
        out[nm] = dict(a=float(t[0]), b=float(t[1]), r=float(math.hypot(t[0], t[1])))
    return out, dict(gap=gap - FIL_R, n=n_tip, D=D, ax=ax_l)


def barrel_clearance(foot, pen) -> float:
    """Smallest distance (m) from the barrel's surface (and its nose) to the foot's skin and its stand."""
    best = 1.0
    for x in np.linspace(-BARREL_L / 2, NOSE_X, 16):
        r = BARREL_R if x <= BARREL_L / 2 else 0.004
        p = foot.to_local(pen.to_world((x, 0.0, 0.0)))
        best = min(best, skin_dist(p)[0] - r)
        for c, Rb, sz in FOOT_COLL:
            best = min(best, _box_dist(Rb.T @ (p - c), sz) - r)
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
        set_visible("foot", f"body/callus_{i}", nm == callus)


_reset()


@on_step
def _watch(environment) -> None:
    foot, pen = _part_state(environment, FOOT), _part_state(environment, PEN)
    if _STATE["foot0"] is None:
        _STATE["foot0"] = foot.pos.copy()
    _STATE["tcp"] = _tcps(environment)
    lp, lR = _link_pose(environment, "pen", f"f{FIL_N - 1}")
    tip = lp + lR @ np.array([LINK - FIL_R, 0.0, 0.0])
    rd, g = site_readings(foot, pen, tip)
    hinge = max(abs(v) for v in pen.sliders.values()) if pen.sliders else 0.0
    _STATE["last"] = dict(tip=tip, rd=rd, g=g, hinge=hinge)
    if _STATE["gate"]:
        return
    # gates: callus touched, barrel on the foot, pen off the bench, foot shoved
    if rd[_STATE["callus"]]["r"] <= CALLUS_R + FIL_R and g["gap"] <= CALLUS_H + TOUCH:
        _STATE["gate"] = "filament touched the callus"
    elif barrel_clearance(foot, pen) <= 0.001:
        _STATE["gate"] = "pen barrel touched the foot"
    elif pen.pos[2] < TOP - 0.03:
        _STATE["gate"] = "pen fell off the bench"
    elif float(np.linalg.norm(foot.pos - _STATE["foot0"])) > FOOT_MOVE_MAX:
        _STATE["gate"] = "foot model moved more than 20 mm"
    if _STATE["gate"]:
        return
    near = min(SITE_NAMES, key=lambda k: rd[k]["r"])
    touching = g["gap"] <= TOUCH and rd[near]["r"] <= SITE_ZONE
    ep = _STATE["ep"]
    if touching:
        if ep is None:   # landing: the skin normal under the tip is the normal the angle is checked against
            ep = _STATE["ep"] = dict(site=near, steps=0, gap=0, land=(rd[near]["a"], rd[near]["b"]), n=g["n"].copy(),
                                     slide=0.0, minD=1.0, max_ang=0.0, buckled_steps=0, switched=False, max_hinge=0.0)
        if near != ep["site"]:
            ep["switched"] = True
        r = rd[ep["site"]]
        ep["steps"] += 1 + ep["gap"]
        ep["gap"] = 0
        ep["minD"] = min(ep["minD"], g["D"])
        ep["max_hinge"] = max(ep["max_hinge"], math.degrees(hinge))
        if g["D"] <= D_WINDOW[1]:   # while the filament is bent: slide from the landing point, and the angle
            ep["slide"] = max(ep["slide"], math.hypot(r["a"] - ep["land"][0], r["b"] - ep["land"][1]))
            ep["buckled_steps"] += 1
            ang = math.degrees(math.acos(float(np.clip(-np.dot(g["ax"], ep["n"]), -1.0, 1.0))))
            ep["max_ang"] = max(ep["max_ang"], ang)
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
    test (a touch belongs to the nearest site within 18 mm): the filament tip landed within 8 mm of the site (on the
    callus site: 2 to 8 mm outside the callus edge), the barrel tip came within 35 mm of the skin (the filament bent)
    but never closer than 24 mm, the pen's axis stayed within 15 deg of the skin normal at the landing point and the
    tip within 3 mm of its landing point while bent, and the contact lasted 1.0 to 3.0 s. A site's later contacts never count. Four sites score 0.75 until the pen lies back in its cradle
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
    rd, g = L["rd"], L["g"]
    near = min(SITE_NAMES, key=lambda k: rd[k]["r"])
    pen = objects["pen"]
    tc = (_STATE.get("tcp") or {}).get("right")
    r = rd[near]
    sl = objects["pen"].sliders
    ang = math.degrees(math.acos(float(np.clip(-np.dot(g["ax"], g["n"]), -1.0, 1.0))))
    return (f"near {near} tip a{r['a']*1000:.1f} b{r['b']*1000:.1f} gap{g['gap']*1000:.1f} D{g['D']*1000:.1f} "
            f"ang{ang:.1f} hinges {[round(math.degrees(v),1) for v in sl.values()]} "
            f"ep {None if not _STATE.get('ep') else (_STATE['ep']['site'], _STATE['ep']['steps'])} "
            f"done {[(k, v['passed']) for k, v in _STATE.get('results', {}).items()]} gate {_STATE.get('gate')} "
            f"score {_update(objects)[0]} pen {np.round(pen.pos*1000,0).tolist()} ax {np.round(pen.rotation()[:,0],2).tolist()} "
            f"tcp {None if tc is None else np.round(tc*1000,0).tolist()}")
