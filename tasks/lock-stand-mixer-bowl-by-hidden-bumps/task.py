"""Lock a stand mixer's bowl onto its clamping plate (yam_arena task file).

A tilt-head stand mixer (4.5 qt class, KitchenAid K45SS size) stands on the station table with its head tilted back.
Its empty stainless bowl stands on the table in front of it. The bowl's foot carries three bumps that are hidden under
the bowl; the clamping plate has three narrow slots where the bumps can go down, each leading clockwise into a channel
under a lip that ends at a stop. The bowl locks only when it is lowered with its bumps over the slots, so it drops onto
the plate, and is then turned clockwise until the bumps are under the lips. Everything is geometry and contact:
nothing is latched in code.

Mixer frame (metres): origin on the table under the plate centre, x away from the robot (toward the column), y to the
robot's left, z up. Angles about z are counter-clockwise seen from above; clockwise turning lowers them.
"""
from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import _ENV, _part_state, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Put the mixing bowl on the stand mixer and lock it in place, then let go."

WHITE, CHROME, STEEL, GREY, DARK = (0.86, 0.86, 0.84), (0.72, 0.73, 0.75), (0.80, 0.81, 0.83), (0.55, 0.56, 0.58), (0.2, 0.2, 0.21)

# ---- mixer (one rigid body, free on the table) ----------------------------------------------------------------
MIXER_XY = (0.40, 0.0)
MIXER_MASS = 9.98            # K45SS net weight 22 lb (retailer listings)
BASE_H = 0.045               # pedestal height at the plate (estimate)
PLATE_R, PLATE_T = 0.076, 0.003
PT = BASE_H + PLATE_T        # plate top, mixer frame
R_IN, R_WALL, R_OUT = 0.066, 0.072, 0.076
LIP_LO, LIP_HI = PT + 0.0055, PT + 0.0085       # lip underside / top (2 mm channel clearance over the bumps)
SLOT_W, CHAN_W = 20.0, 30.0  # degrees: slot where a bump goes down; channel under the lip, clockwise from the slot
STEP = 5.0                   # degrees per ring segment
SLOTS = (93.6, 213.6, 333.6) # slot centres, degrees (mixer frame)


def _arc_boxes(r0, r1, z0, z1, a0, a1, color):
    """Boxes approximating the ring sector r0..r1, z0..z1, a0..a1 degrees."""
    out = []
    n = max(1, round((a1 - a0) / STEP))
    da = (a1 - a0) / n
    rm, dr = (r0 + r1) / 2, (r1 - r0)
    for i in range(n):
        a = a0 + (i + 0.5) * da
        L = 2 * r1 * math.sin(math.radians(da) / 2) + 0.0006
        out.append(((rm * math.cos(math.radians(a)), rm * math.sin(math.radians(a)), (z0 + z1) / 2),
                    (L, dr, z1 - z0), color, (a - 90.0, 0.0, 0.0)))
    return out


def _mixer_boxes():
    b = []
    # pedestal: 221 mm wide, 350 mm deep in all with the column (estimates within the 8.7 x 14.1 in footprint)
    b.append(((0.055, 0.0, BASE_H / 2), (0.350, 0.221, BASE_H), WHITE))
    # column at the back, head hinge on top
    b.append(((0.175, 0.0, 0.150), (0.090, 0.120, 0.210), WHITE))
    # head tilted back 60 deg about the hinge (y axis) at the column's top rear
    hinge = np.array([0.225, 0.0, 0.255])
    rel = np.array([-0.165, 0.0, 0.060])
    t = math.radians(60.0)
    rel2 = np.array([rel[0] * math.cos(t) + rel[2] * math.sin(t), 0.0, -rel[0] * math.sin(t) + rel[2] * math.cos(t)])
    b.append((tuple(hinge + rel2), (0.330, 0.130, 0.120), WHITE, (0.0, 60.0, 0.0)))
    # clamping plate: outer wall all round, then per sector: open slot, lip over a channel, solid block
    b += _arc_boxes(R_WALL, R_OUT, PT, LIP_HI, 0.0, 360.0, CHROME)
    for s in SLOTS:
        b += _arc_boxes(R_IN, R_WALL + 0.0005, LIP_LO, LIP_HI, s - SLOT_W / 2 - CHAN_W, s - SLOT_W / 2, CHROME)
        b += _arc_boxes(R_IN, R_WALL + 0.0005, PT, LIP_HI, s + SLOT_W / 2 - 120.0, s - SLOT_W / 2 - CHAN_W, CHROME)
    return b


MIXER = part("mixer", pos=(*MIXER_XY, TOP), mass=MIXER_MASS, color=WHITE, friction=0.9, boxes=_mixer_boxes(),
             cylinders=(((0.0, 0.0, BASE_H + PLATE_T / 2), PLATE_R, PLATE_T, CHROME),))

# ---- bowl (bowl frame: origin at the foot's bottom centre) --------------------------------------------------------
BOWL_MASS = 0.73             # estimate (shipping weights 2.0-2.2 lb include the carton)
FOOT_R, FOOT_H = 0.064, 0.012
BUMP_R0, BUMP_R1, BUMP_Z0, BUMP_Z1, BUMP_W = 0.0635, 0.070, 0.0015, 0.0045, 0.0150
BUMP_HALF = math.degrees(BUMP_W / 2 / 0.067)    # 6.4 deg
BUMPS = tuple(s - SLOT_W / 2 - CHAN_W + BUMP_HALF for s in SLOTS)   # locked (against the stop) at bowl yaw 0
R_BOT, R_TOP, RIM_Z = 0.075, 0.108, 0.178       # 8.75 in rim (222 mm), 7 in tall (listings)
HANDLE_AZ = 225.0            # handle azimuth in the bowl frame: front right when locked (estimate)
WALL_T = 0.003


def _bowl_boxes():
    b = [((0.0, 0.0, FOOT_H + 0.001), (0.13, 0.13, 0.002), STEEL)]   # bottom (a square plate inside the round wall)
    n = 24
    slope = math.degrees(math.atan2(R_TOP - R_BOT, RIM_Z - FOOT_H))
    slant = math.hypot(R_TOP - R_BOT, RIM_Z - FOOT_H)
    rm, zm = (R_BOT + R_TOP) / 2, (FOOT_H + RIM_Z) / 2
    for i in range(n):
        a = 360.0 * i / n
        ca, sa = math.cos(math.radians(a)), math.sin(math.radians(a))
        L = 2 * R_TOP * math.sin(math.pi / n) + 0.001
        b.append(((rm * ca, rm * sa, zm), (L, WALL_T, slant), STEEL, (a - 90.0, 0.0, -slope)))
        b.append(((0.1095 * ca, 0.1095 * sa, RIM_Z - 0.003), (2 * 0.111 * math.sin(math.pi / n) + 0.001, 0.005, 0.006),
                  STEEL, (a - 90.0, 0.0, 0.0)))
    for az in BUMPS:
        ca, sa = math.cos(math.radians(az)), math.sin(math.radians(az))
        rr = (BUMP_R0 + BUMP_R1) / 2
        b.append(((rr * ca, rr * sa, (BUMP_Z0 + BUMP_Z1) / 2), (BUMP_W, BUMP_R1 - BUMP_R0, BUMP_Z1 - BUMP_Z0), STEEL,
                  (az - 90.0, 0.0, 0.0)))
    # handle: a riveted steel loop on the side, out to 9.6 in overall width (listings), grip bar 18 mm wide
    ca, sa = math.cos(math.radians(HANDLE_AZ)), math.sin(math.radians(HANDLE_AZ))
    for (r, z, dr, dz) in ((0.129, 0.130, 0.006, 0.075), (0.114, 0.168, 0.030, 0.006), (0.112, 0.094, 0.034, 0.006)):
        b.append(((r * ca, r * sa, z), (0.018, dr, dz), STEEL, (HANDLE_AZ - 90.0, 0.0, 0.0)))
    return b


BOWL0 = (0.14, 0.0, 20.0)   # start: x, y, yaw (deg) on the table, handle toward the front right
BOWL = part("bowl", pos=(BOWL0[0], BOWL0[1], TOP), mass=BOWL_MASS, color=STEEL, friction=0.45, boxes=_bowl_boxes(),
            cylinders=(((0.0, 0.0, FOOT_H / 2), FOOT_R, FOOT_H, STEEL),))
OBJECTS = [MIXER, BOWL]

# ---- scoring ------------------------------------------------------------------------------------------------------
LOCK_YAW = (-1.5, CHAN_W - 2 * BUMP_HALF + 0.5)   # bumps wholly under the lips: 0 at the stop .. 17.7 deg
_S: dict = {}


def bowl_start(seed):
    rng = np.random.default_rng(2000 + int(seed or 0))
    return (BOWL0[0] + rng.uniform(-0.015, 0.015), BOWL0[1] + rng.uniform(-0.02, 0.02),
            BOWL0[2] + rng.uniform(-5.0, 5.0))


def _yaw(st):
    r = st.rotation()
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def _tilt(st):
    return math.degrees(math.acos(max(-1.0, min(1.0, st.rotation()[2, 2]))))


def rel_yaw(bowl, mixer):
    d = (_yaw(bowl) - _yaw(mixer) + 180.0) % 360.0 - 180.0
    return d


def _plate_centre(mixer):
    return mixer.to_world((0.0, 0.0, PT))


@on_reset
def _reset(environment=None) -> None:
    _S.update(dropped=False, mixer_moved=False, near=True, still=0, k=0, m0=None, vmax=0.0)
    if environment is not None:
        x, y, yaw = bowl_start(_ENV.get("seed"))
        h = math.radians(yaw) / 2
        write_pose(environment, "bowl", (x, y, TOP + 0.0005), (0, 0, math.sin(h), math.cos(h)))
        write_pose(environment, "mixer", (*MIXER_XY, TOP), (0, 0, 0, 1))


_reset()


def _tcps(environment):
    from yam_arena import ik, station
    out = []
    base = environment.unwrapped
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        q = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
        out.append(ik.tcp_pose(q, station.BASES[side])[0])
    return out


def near_bowl(bowl, tcps) -> bool:
    """A fingertip within about 3 cm of the bowl (inside its rim-plus-handle cylinder, up to 3 cm above the rim)."""
    for p in tcps:
        q = bowl.to_local(p)
        if math.hypot(q[0], q[1]) < 0.165 and -0.03 < q[2] < RIM_Z + 0.03:
            return True
    return False


@on_step
def _watch(environment) -> None:
    bowl = _part_state(environment, BOWL)
    mixer = _part_state(environment, MIXER)
    _S["k"] += 1
    if _S["m0"] is None:
        _S["m0"] = (mixer.pos.copy(), _yaw(mixer))
    v = float(np.linalg.norm(bowl.lin_vel)) if getattr(bowl, "lin_vel", None) is not None else 0.0
    _S["vmax"] = max(_S["vmax"], v)
    if v > 0.8 or bowl.pos[2] < TOP - 0.05:
        _S["dropped"] = True
    p0, y0 = _S["m0"]
    if (np.linalg.norm(mixer.pos[:2] - p0[:2]) > 0.02 or abs(_yaw(mixer) - y0) > 5.0 or _tilt(mixer) > 5.0):
        _S["mixer_moved"] = True
    try:
        _S["near"] = near_bowl(bowl, _tcps(environment))
    except Exception as exc:  # noqa: BLE001
        _S["err"] = repr(exc)
    slow = v < 0.01 and float(np.linalg.norm(getattr(bowl, "ang_vel", np.zeros(3)))) < 0.05
    _S["still"] = _S["still"] + 1 if slow else 0


def locked(bowl, mixer) -> dict:
    c = _plate_centre(mixer)
    foot = bowl.pos
    d = {"down_mm": round(1000 * (foot[2] - c[2]), 2),
         "axis_mm": round(1000 * float(np.linalg.norm(foot[:2] - c[:2])), 2),
         "tilt_deg": round(_tilt(bowl), 2), "rel_yaw_deg": round(rel_yaw(bowl, mixer), 2)}
    d["seated"] = abs(d["down_mm"]) <= 1.5 and d["axis_mm"] <= 3.0 and d["tilt_deg"] <= 2.0
    d["locked"] = d["seated"] and LOCK_YAW[0] <= d["rel_yaw_deg"] <= LOCK_YAW[1]
    return d


def _update(objects):
    if _S["dropped"] or _S["mixer_moved"]:
        return 0.0, False
    lk = locked(objects["bowl"], objects["mixer"])
    if not lk["locked"]:
        return 0.0, False
    if not _S["near"] and _S["still"] >= 20:
        return 1.0, True
    return 0.5, False


def success(objects) -> bool:
    return _update(objects)[1]


def score(objects) -> float:
    return _update(objects)[0]


def report(objects) -> dict:
    return {**locked(objects["bowl"], objects["mixer"]), "dropped": _S["dropped"], "mixer_moved": _S["mixer_moved"],
            "near": _S["near"], "still_steps": _S["still"], "vmax": round(_S["vmax"], 3),
            "score": _update(objects)[0], "err": _S.get("err")}


def debug_line(objects) -> str:
    return " ".join(f"{k}={v}" for k, v in locked(objects["bowl"], objects["mixer"]).items())
