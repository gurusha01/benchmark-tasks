"""Set a laser receiver on a grade rod to the laser plane, with the rod held plumb on the grade stake.

A rotary laser on a tripod sweeps a level plane across the site. A collapsed fibreglass grade rod (SECO 8 ft class,
two sections, 1.32 m collapsed) leans on a sawhorse beside a 2x2 grade stake driven into a tray of compacted soil. A
laser receiver (Spectra Precision HR320 class) is clamped on the rod at the wrong height. Its back display shows
three LED marks: the top mark lights when the receiver is above the laser plane, the bottom mark when it is below,
the middle (green) mark when it is on grade (within 3 mm, the receiver's coarse dead band). The rod carries a round
bubble level at the top of its lower section.

The job, as on any grading crew: stand the rod's foot on the stake top, hold the rod plumb by its bubble, loosen the
receiver clamp, slide the receiver until it reads on grade, tighten the clamp and let go of the receiver. The rod
then reads the stake's grade mark for everything set later.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import (_ENV, Slider, _part_state, on_reset, on_step, part, set_target, set_visible,  # noqa: E402
                     write_pose)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Hold the grade rod plumb on the stake by its bubble and clamp the laser receiver on the rod where it "
               "reads on grade.")

SOIL, WOOD, DARK, STEEL, YELLOW, ORANGE, GREEN, RED, BLUE, OFFLED, BLACK, WHITE, FIBRE = (
    (0.42, 0.33, 0.24), (0.76, 0.60, 0.40), (0.12, 0.12, 0.13), (0.55, 0.56, 0.58), (0.95, 0.78, 0.10),
    (0.95, 0.45, 0.08), (0.10, 0.85, 0.20), (0.95, 0.10, 0.08), (0.15, 0.35, 0.95), (0.22, 0.23, 0.24),
    (0.03, 0.03, 0.03), (0.95, 0.95, 0.92), (0.93, 0.90, 0.80))

# ---- site: tray of compacted soil with a 2x2 grade stake (frame: origin on the table under the stake) -----------
SX, SY = 0.30, 0.0
TRAY = 0.28                # 28 x 28 cm tray
SOIL_H = 0.04              # soil surface 4 cm above the table
STAKE = 0.038              # nominal 2x2 (1.5 in = 38 mm square)
STAKE_UP = 0.025           # stake top 2.5 cm proud of the soil
Z_STAKE = SOIL_H + STAKE_UP
SITE_BOXES = [
    ((0.0, 0.0, SOIL_H / 2), (TRAY, TRAY, SOIL_H), SOIL),
    ((0.0, 0.0, Z_STAKE - 0.03), (STAKE, STAKE, 0.06), WOOD),                     # stake (driven in)
    ((0.0, 0.0, Z_STAKE + 0.0005), (0.006, 0.006, 0.001), BLACK, None, None, "visual"),   # tack on the hub
    ((0.0, 0.0, Z_STAKE + 0.0004), (STAKE - 0.004, 0.004, 0.0008), BLUE, None, None, "visual"),  # paint line
]
SITE = part("site", pos=(SX, SY, TOP), mass=25.0, color=SOIL, friction=1.0, boxes=SITE_BOXES)

# ---- grade rod (frame: origin at the centre of the bottom of its brass shoe, +z up the rod) ----------------------
ROD_L = 1.32               # SECO 8 ft two-section fibreglass rod, collapsed (52 in)
ROD_T, ROD_W = 0.025, 0.044  # cross-section, x (thickness) by y (face width), estimate
ROD_MASS = 1.3             # estimate
SHOE_H = 0.012
BUBBLE_U = 0.50            # round level on a bracket at the top of the lower section (estimate: within reach)
BUBBLE_X = -ROD_T / 2 - 0.022
BUBBLE_TOL = math.radians(1.0)   # the vial's circle
_tick = []
for k in range(1, 26):     # tenths-of-a-foot style graduation marks on the face toward the robot (visual)
    _tick.append(((-ROD_T / 2 - 0.0004, -ROD_W / 4, SHOE_H + k * 0.0508), (0.0008, ROD_W / 2, 0.004), BLACK,
                  None, None, "visual"))
ROD_BOXES = [
    ((0.0, 0.0, SHOE_H / 2), (ROD_T + 0.004, ROD_W + 0.004, SHOE_H), (0.70, 0.55, 0.25)),   # brass shoe
    ((0.0, 0.0, SHOE_H + (ROD_L - SHOE_H) / 2), (ROD_T, ROD_W, ROD_L - SHOE_H), FIBRE),
    ((-ROD_T / 2 - 0.011, 0.0, BUBBLE_U - 0.004), (0.022, 0.020, 0.004), STEEL),           # bubble bracket
    ((BUBBLE_X, 0.0, BUBBLE_U + 0.004), (0.020, 0.020, 0.012), DARK),                       # vial housing
    ((BUBBLE_X, 0.0, BUBBLE_U + 0.0102), (0.016, 0.016, 0.0004), (0.80, 0.95, 0.80), None, None, "visual"),
    *_tick,
]
N_ROD_FIXED = len(ROD_BOXES)
# bubble: 9 painted positions on the vial top, one shown at a time (centre when the rod is within 1 deg of plumb,
# otherwise the one toward the rod's high side); shown/hidden from the rod's tilt every step
_BUB = [(0.0, 0.0)] + [(0.0055 * math.cos(k * math.pi / 4), 0.0055 * math.sin(k * math.pi / 4)) for k in range(8)]
for bx, by in _BUB:
    ROD_BOXES.append(((BUBBLE_X + bx, by, BUBBLE_U + 0.0108), (0.004, 0.004, 0.0008), BLACK, None, None, "visual"))
ROD_BOXES.append(((BUBBLE_X, 0.0, BUBBLE_U + 0.0106), (0.009, 0.009, 0.0004), BLACK, None, None, "visual"))  # circle
ROD_BOXES.append(((BUBBLE_X, 0.0, BUBBLE_U + 0.0108), (0.0075, 0.0075, 0.0005), (0.80, 0.95, 0.80), None, None,
                  "visual"))

# ---- laser receiver with its rod clamp (frame: origin on the rod axis at the receiver's photocell centre) --------
RCV_W, RCV_D, RCV_H = 0.075, 0.040, 0.165      # HR320 class body, estimate
RCV_MASS = 0.38
RCV_Y = -(ROD_W / 2 + 0.016 + RCV_W / 2)        # body beside the rod, toward the robot's right
CLAMP_H = 0.045
GAP = 0.001                                     # clamp frame clearance around the rod
FRONT_X = ROD_T / 2 + GAP                       # fixed jaw (front plate) inner face, rod frame x
JAW_REST = -ROD_T / 2 - 0.002                   # moving jaw face when loose: 2 mm off the rod
JAW_TRAVEL = 0.004                              # 2 mm to contact + 2 mm of spring wind-up when tight
JAW_RATE = 15000.0                              # 30 N clamp force when tight (estimate)
KNOB_TIGHT = math.pi / 2                        # the wing knob locks in a quarter turn (estimate)
KNOB_X = JAW_REST - 0.008 - 0.012               # wing knob behind the jaw, toward the robot
RCV_BOXES = [
    ((0.0, RCV_Y, 0.0), (RCV_D, RCV_W, RCV_H), YELLOW),                                    # receiver body
    ((-RCV_D / 2 - 0.0005, RCV_Y, 0.050), (0.001, 0.060, 0.056), DARK, None, None, "visual"),  # back display
    ((-RCV_D / 2 - 0.0005, RCV_Y + RCV_W / 2 - 0.004, 0.0), (0.001, 0.008, 0.003), BLACK, None, None, "visual"),  # notch
    ((-RCV_D / 2 - 0.0005, RCV_Y - RCV_W / 2 + 0.004, 0.0), (0.001, 0.008, 0.003), BLACK, None, None, "visual"),
    ((RCV_D / 2 + 0.0005, RCV_Y, 0.0), (0.001, 0.055, 0.050), (0.35, 0.10, 0.08), None, None, "visual"),  # cell
    # clamp frame around the rod: plate on the receiver side, front plate, far side plate, back bridge
    ((-0.004, -ROD_W / 2 - GAP - 0.008, 0.0), (0.052, 0.016, CLAMP_H), DARK),
    ((FRONT_X + 0.004, 0.0, 0.0), (0.008, ROD_W + 2 * GAP, CLAMP_H), DARK),
    ((-0.004, ROD_W / 2 + GAP + 0.004, 0.0), (0.052, 0.008, CLAMP_H), DARK),
    ((JAW_REST - 0.008 - 0.003, 0.0, 0.0), (0.006, ROD_W + 2 * GAP + 0.024, CLAMP_H), DARK),
]
# LED marks on the back display (lit caps shown over dark ones): high (top, red), on grade (middle, green),
# low (bottom, blue). Index of the first lit cap in RCV_BOXES:
LED_Z = {"high": 0.066, "on": 0.050, "low": 0.034}   # display window near the top of the back face
LED_COL = {"high": RED, "on": GREEN, "low": BLUE}
for k in ("high", "on", "low"):
    RCV_BOXES.append(((-RCV_D / 2 - 0.0012, RCV_Y, LED_Z[k]), (0.001, 0.034, 0.012), OFFLED, None, None, "visual"))
LED_FIRST = len(RCV_BOXES)
for k in ("high", "on", "low"):
    RCV_BOXES.append(((-RCV_D / 2 - 0.0020, RCV_Y, LED_Z[k]), (0.001, 0.034, 0.012), LED_COL[k], None, None,
                      "visual"))
JAW = Slider("jaw", rest=(JAW_REST - 0.004, 0.0, 0.0), axis="X", sign=1, travel=JAW_TRAVEL, rate=JAW_RATE,
             damping=200.0, mass=0.02, start=JAW_TRAVEL,
             boxes=(((JAW_REST - 0.004, 0.0, 0.0), (0.008, ROD_W - 0.004, CLAMP_H - 0.006), STEEL),))
KNOB = Slider("knob", rest=(KNOB_X, 0.0, 0.0), kind="revolute", axis="X", sign=1, travel=KNOB_TIGHT, rate=0.05,
              damping=0.02, mass=0.01, start=KNOB_TIGHT,
              boxes=(((KNOB_X + 0.001, 0.0, 0.0), (0.012, 0.012, 0.012), DARK),            # hub
                     ((KNOB_X - 0.006, 0.0, 0.0), (0.006, 0.050, 0.010), ORANGE)))         # wings across
RECEIVER = part("receiver", pos=(0.0, 0.0, 0.0), mass=RCV_MASS, color=YELLOW, friction=0.6, boxes=RCV_BOXES,
                sliders=(JAW, KNOB), ang_damping=5.0)

# ---- sawhorse rail the rod leans on, and the rotary laser on its tripod (props) ---------------------------------
LEAN = math.radians(14.2)  # the rod's lean against the post as placed (it settles to LEAN_REST)
LEAN_REST = math.radians(12.7)   # about 13 deg: how it rests on the post (measured in the sim)
FOOT_Y0 = SY               # the rodman has set the shoe on the stake top and leaned the rod on the post
POST = 0.089               # a 4x4 post set in a bucket of concrete, beyond the stake: the rod leans on it
POST_H = 1.00
_uc = 0.88                 # the rod touches the post 88 cm up from its shoe, above its centre of mass
HORSE_X = SX + _uc * math.sin(LEAN) + (ROD_T / 2) / math.cos(LEAN) + POST / 2
HORSE_BOXES = [((0.0, 0.0, 0.11), (0.20, 0.20, 0.22), (0.35, 0.36, 0.38)),             # bucket of concrete
               ((0.0, 0.0, 0.22 + (POST_H - 0.22) / 2), (POST, POST, POST_H - 0.22), WOOD)]
HORSE = part("horse", pos=(HORSE_X, FOOT_Y0, TOP), mass=30.0, color=WOOD, friction=1.0, boxes=HORSE_BOXES)

U_GRADE = 0.215            # receiver centre height above the stake top when on grade: the laser plane's height
Z_PLANE = TOP + Z_STAKE + U_GRADE
LASER_X, LASER_Y = 0.62, -0.30
LASER_BOXES = [((0.0, 0.0, 0.004), (0.30, 0.30, 0.008), DARK),                            # tripod foot plate
               ((0.0, 0.0, (Z_PLANE - TOP - 0.10) / 2), (0.03, 0.03, Z_PLANE - TOP - 0.10), STEEL),
               ((0.0, 0.0, Z_PLANE - TOP - 0.07), (0.16, 0.16, 0.06), RED),
               ((0.0, 0.0, Z_PLANE - TOP), (0.06, 0.06, 0.03), (0.10, 0.10, 0.12))]
LASER = part("laser", pos=(LASER_X, LASER_Y, TOP), mass=6.0, color=RED, friction=1.0, boxes=LASER_BOXES)

ROD = part("rod", pos=(SX, SY, TOP + Z_STAKE), mass=ROD_MASS, color=FIBRE, friction=0.9, boxes=ROD_BOXES)
OBJECTS = [SITE, HORSE, LASER, ROD, RECEIVER]

# ---- scoring -----------------------------------------------------------------------------------------------------
ON_GRADE = 0.003           # +-3 mm: the receiver's coarse dead band (what the display shows)
SET_TOL = 0.004            # scored along the rod: the 3 mm band plus 1 mm for how the shoe seats on the stake
WINDOW = 0.025             # +-25 mm reception window (2 in): outside it no mark is lit
FOOT_TOL = STAKE / 2       # the whole shoe centre over the 38 mm stake top
HANDS_CLEAR = 0.03
STILL_STEPS = 10           # "still": rod and receiver each moved less than 5 mm over the last 0.5 s (10 steps)
STILL_MOVE = 0.005
_STATE: dict = {}


def _still() -> bool:
    h = _STATE.get("hist") or []
    if len(h) < STILL_STEPS:
        return False
    a = np.array([p for p, _ in h])
    b = np.array([p for _, p in h])
    return bool(np.linalg.norm(a - a[-1], axis=1).max() < STILL_MOVE and np.linalg.norm(b - b[-1], axis=1).max() < STILL_MOVE)


def rod_tilt(rod) -> float:
    return math.acos(max(-1.0, min(1.0, float(rod.rotation()[2, 2]))))


def receiver_u(rod, rcv) -> float:
    """Receiver photocell centre along the rod, above the bottom of the shoe (m)."""
    return float(rod.to_local(rcv.pos)[2])


def receiver_on_rod(rod, rcv) -> bool:
    p = rod.to_local(rcv.pos)
    return abs(p[0]) < 0.02 and abs(p[1]) < 0.02 and 0.0 < p[2] < ROD_L


def foot_on_stake(rod, site) -> bool:
    f = site.to_local(rod.pos)
    return abs(f[0]) <= FOOT_TOL and abs(f[1]) <= FOOT_TOL and Z_STAKE - 0.003 <= f[2] <= Z_STAKE + 0.004


def clamp_tight(rcv) -> bool:
    return rcv.sliders.get("knob", 0.0) >= KNOB_TIGHT - math.radians(8)


def display(rod, rcv) -> str | None:
    """What the receiver's marks show: the photocell height against the laser plane (world z)."""
    if not receiver_on_rod(rod, rcv):
        return None
    d = float(rcv.pos[2]) - Z_PLANE
    if abs(d) > WINDOW:
        return None
    return "on" if abs(d) <= ON_GRADE else ("high" if d > 0 else "low")


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(gate=None, tcp={}, start=None, shown="init", bubble=-1, trace=[], hist=[])
    if environment is None:
        return
    seed = _ENV.get("seed")
    rng = np.random.default_rng((int(seed) if seed is not None else 0) + 4243)
    off = rng.uniform(0.012, 0.022) * (1 if rng.random() < 0.5 else -1)   # receiver start error along the rod
    dy = rng.uniform(-0.004, 0.004)
    _STATE["start"] = (round(off, 4), round(dy, 4))
    # rod leaning on the post, its shoe on the stake top (up to 4 mm off centre), top toward the post (+x)
    foot = np.array([SX, FOOT_Y0 + dy, TOP + Z_STAKE + 0.0005])
    q = (0.0, math.sin(LEAN / 2), 0.0, math.cos(LEAN / 2))   # about +y: top leans toward +x
    write_pose(environment, "rod", tuple(foot), q)
    c, s = math.cos(LEAN), math.sin(LEAN)
    u = U_GRADE + off                       # clamped where it read for the last stake: wrong for this one
    _STATE["u0"] = u + 0.0056               # it settles ~5.6 mm as the clamp takes the receiver's weight
    rp = foot + np.array([s * u, 0.0, c * u])
    write_pose(environment, "receiver", tuple(rp), q)
    set_target(environment, "receiver", "jaw", JAW_TRAVEL)     # clamped: the knob starts tight
    set_target(environment, "receiver", "knob", KNOB_TIGHT)
    for i in range(3):                      # nothing lit until the first step reads the display
        set_visible("receiver", f"body/box_{LED_FIRST + i}", False)
    for i in range(9):
        set_visible("rod", f"body/box_{N_ROD_FIXED + i}", False)


_reset()


def _tcps(environment):
    from yam_arena import ik, station

    base = environment.unwrapped
    out = {}
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        qq = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
        out[side] = ik.tcp_pose(qq, station.BASES[side])[0]
    return out


def _bubble_index(rod) -> int:
    t = rod_tilt(rod)
    if t <= BUBBLE_TOL:
        return 0
    R = rod.rotation()
    h = -R[:2, 2]                                  # the bubble runs to the high side, opposite the rod's lean
    lx, ly = float(R[:2, 0] @ h), float(R[:2, 1] @ h)
    k = int(round(math.atan2(ly, lx) / (math.pi / 4))) % 8
    return 1 + k


@on_step
def _watch(environment) -> None:
    rod = _part_state(environment, ROD)
    rcv = _part_state(environment, RECEIVER)
    if _STATE.get("u0") is not None:
        # the post-reset settle steps run before the clamp spring is applied, so the receiver is put at its seeded
        # height on the rod (as the rod now rests) once, on the first control step
        write_pose(environment, "receiver", tuple(rod.to_world((0.0, 0.0, _STATE["u0"]))), tuple(rod.quat_xyzw))
        set_target(environment, "receiver", "jaw", JAW_TRAVEL)
        _STATE["u0"] = None
        return
    if rod_tilt(rod) > math.radians(45) or rod.pos[2] < TOP - 0.05:
        _STATE["gate"] = _STATE["gate"] or "rod fell"
    if not receiver_on_rod(rod, rcv):
        _STATE["gate"] = _STATE["gate"] or "receiver off the rod"
    # the wing knob stays where it is turned (thread friction); it drives the moving jaw
    kq = rcv.sliders.get("knob", 0.0)
    set_target(environment, "receiver", "knob", kq)
    set_target(environment, "receiver", "jaw", JAW_TRAVEL * max(0.0, min(1.0, kq / KNOB_TIGHT)))
    shown = display(rod, rcv)
    if shown != _STATE.get("shown"):
        for i, k in enumerate(("high", "on", "low")):
            set_visible("receiver", f"body/box_{LED_FIRST + i}", shown == k)
        _STATE["shown"] = shown
    b = _bubble_index(rod)
    if b != _STATE.get("bubble"):
        for i in range(9):
            set_visible("rod", f"body/box_{N_ROD_FIXED + i}", i == b)
        _STATE["bubble"] = b
    _STATE["tcp"] = _tcps(environment)
    h = _STATE.setdefault("hist", [])
    h.append((rcv.pos.copy(), rod.pos.copy()))
    del h[:-STILL_STEPS]


def _near(rcv, t) -> bool:
    p = rcv.to_local(t)
    return bool(-0.06 <= p[0] <= 0.05 and RCV_Y - RCV_W / 2 - HANDS_CLEAR <= p[1] <= ROD_W / 2 + 0.02 + HANDS_CLEAR
                and abs(p[2]) <= RCV_H / 2 + HANDS_CLEAR)


def _update(objects):
    if _STATE.get("gate"):
        return 0.0, False
    rod, rcv, site = objects["rod"], objects["receiver"], objects["site"]
    tcp = list((_STATE.get("tcp") or {}).values())
    still = _still()
    set_ok = (abs(receiver_u(rod, rcv) - U_GRADE) <= SET_TOL and clamp_tight(rcv)
              and receiver_on_rod(rod, rcv) and not any(_near(rcv, t) for t in tcp) and still)
    if not set_ok:
        return 0.0, False
    if foot_on_stake(rod, site) and rod_tilt(rod) <= BUBBLE_TOL:
        return 1.0, True
    return 0.5, False


def success(objects) -> bool:
    return _update(objects)[1]


def score(objects) -> float:
    """0 after a gate (rod tipped past 45 deg or dropped, receiver off the rod). Otherwise, at the end: 0.5 when the
    receiver is clamped on the rod at the grade mark (photocell within 4 mm of the laser plane's height above the
    stake top, measured along the rod from its shoe: the 3 mm band plus 1 mm for the shoe's seating), the clamp knob is tight, no fingertip is within 3 cm of the
    receiver and rod and receiver each moved less than 5 mm over the last 0.5 s; 1.0 (success, held 1 s) when, in addition, the rod's shoe stands on the
    stake top and the rod is plumb within its bubble's 1 deg circle. Anything else scores 0."""
    return _update(objects)[0]


def report(objects) -> dict:
    rod, rcv, site = objects["rod"], objects["receiver"], objects["site"]
    return {"start": _STATE.get("start"), "gate": _STATE.get("gate"), "score": _update(objects)[0],
            "receiver_u_mm": round(receiver_u(rod, rcv) * 1000, 1), "grade_u_mm": round(U_GRADE * 1000, 1),
            "display": display(rod, rcv), "knob_deg": round(math.degrees(rcv.sliders.get("knob", 0.0)), 1),
            "jaw_mm": round(rcv.sliders.get("jaw", 0.0) * 1000, 2),
            "hands_near": [bool(_near(rcv, t)) for t in (_STATE.get("tcp") or {}).values()],
            "still": _still(),
            "tilt_deg": round(math.degrees(rod_tilt(rod)), 2), "foot_on_stake": foot_on_stake(rod, site),
            "foot_mm": [round(float(v) * 1000, 1) for v in site.to_local(rod.pos)]}


def debug_line(objects) -> str:
    r = report(objects)
    return " ".join(f"{k}={v}" for k, v in r.items())
