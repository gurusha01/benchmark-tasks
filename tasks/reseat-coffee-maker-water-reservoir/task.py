"""Seat a single-serve coffee brewer's water reservoir back on the brewer, over its hidden base valve.

A pod coffee brewer of the size of a 48 oz-reservoir household model stands on the counter (the station table),
its front toward the robot. Its removable water reservoir has just been rinsed at the sink and stands empty and
upright on the counter in front of the brewer, to the robot's left, lid off. The reservoir's place is on the
brewer's left side (the robot's left), on a base ledge 30 mm above the counter, against the brewer's side wall.
Under the reservoir, near its back, a square valve collar holds a spring-loaded valve poppet. The ledge has a
square valve well with a post in it. When the collar drops into the well, the post pushes the poppet up and opens
the valve; the reservoir's bottom then rests flat on the ledge. The well is hidden under the reservoir while it
goes down. A collar that misses the well lands on the ledge: the reservoir then stands 8 mm high and tilted, with
the valve shut.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import Slider, _part_state, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
MM = 0.001
INSTRUCTION = "Put the water reservoir back on the coffee maker, seated on its valve, and let go."

BLACK, DARK, GREY, CLEARBLUE, WHITE, STEEL = ((0.05, 0.05, 0.06), (0.14, 0.14, 0.15), (0.32, 0.33, 0.35),
                                              (0.62, 0.74, 0.84), (0.85, 0.86, 0.86), (0.60, 0.61, 0.63))

# ---- reservoir (frame: origin at the centre of its bottom face, x toward the brewer's back, y away from the
#      brewer's side wall, z up) ----------------------------------------------------------------------------------
RL, RW, RH = 150.0 * MM, 75.0 * MM, 240.0 * MM     # outside length (x), width (y), height (z)
WALL, FLOOR = 2.5 * MM, 3.0 * MM                   # moulded copolyester walls and floor
RIM_T, RIM_H = 5.0 * MM, 22.0 * MM                 # thickened band along the top edge, where the lid sits
RES_MASS = 0.40                                    # kg, empty, lid off
VX = 45.0 * MM                                     # valve collar centre, 45 mm behind the reservoir's centre
COL_O, COL_T, COL_H = 22.0 * MM, 2.0 * MM, 8.0 * MM  # square collar: 22 mm outside, 2 mm walls, 8 mm below the floor
POP_W, POP_T, POP_TRAVEL = 12.0 * MM, 2.0 * MM, 5.0 * MM
POP_RATE = 300.0                                   # N/m: 1.5 N at full travel (estimate for a small check valve)
POP_REST_Z = -COL_H + 1.0 * MM + POP_T / 2         # poppet face 1 mm inside the collar mouth


def _reservoir_boxes():
    h, t = RH, WALL
    b = [((0.0, 0.0, FLOOR / 2), (RL, RW, FLOOR), CLEARBLUE),                                   # floor
         ((-(RL - t) / 2, 0.0, h / 2), (t, RW, h), CLEARBLUE),                                  # front wall
         (((RL - t) / 2, 0.0, h / 2), (t, RW, h), CLEARBLUE),                                   # back wall
         ((0.0, -(RW - t) / 2, h / 2), (RL - 2 * t, t, h), CLEARBLUE),                          # inner wall (brewer side)
         ((0.0, (RW - t) / 2, h / 2), (RL - 2 * t, t, h), CLEARBLUE),                           # outer wall
         ((0.0, (RW - RIM_T) / 2, h - RIM_H / 2), (RL - 2 * t, RIM_T, RIM_H), CLEARBLUE),       # outer rim
         ((0.0, -(RW - RIM_T) / 2, h - RIM_H / 2), (RL - 2 * t, RIM_T, RIM_H), CLEARBLUE)]      # inner rim
    zc = -COL_H / 2
    o, w = COL_O / 2, COL_T
    b += [((VX - o + w / 2, 0.0, zc), (w, COL_O, COL_H), WHITE),
          ((VX + o - w / 2, 0.0, zc), (w, COL_O, COL_H), WHITE),
          ((VX, -o + w / 2, zc), (COL_O - 2 * w, w, COL_H), WHITE),
          ((VX, o - w / 2, zc), (COL_O - 2 * w, w, COL_H), WHITE)]
    return b


POPPET = Slider("poppet", rest=(VX, 0.0, POP_REST_Z), axis="Z", sign=1, travel=POP_TRAVEL, rate=POP_RATE,
                damping=2.0, mass=0.003, boxes=(((VX, 0.0, POP_REST_Z), (POP_W, POP_W, POP_T), DARK),))

# ---- brewer (frame: origin on the counter at the reservoir's seated centre line: x = 0 at the reservoir's seated
#      centre, y = 0 at the brewer's left side wall, z = 0 on the counter) -----------------------------------------
LEDGE = 30.0 * MM                  # base ledge top above the counter
GAP = 1.5 * MM                     # seated reservoir's inner wall to the brewer's side wall
BODY_D, BODY_W, BODY_H = 338.0 * MM, 170.0 * MM, 330.0 * MM   # 13.3 in deep, 9.8 in wide with the reservoir, 13.0 in tall
BODY_X0 = -0.169                   # body front face (the reservoir sits level with the middle of the body)
LEDGE_X0, LEDGE_X1 = -0.095, 0.095
LEDGE_Y1 = 0.090
WELL = 26.0 * MM                   # square valve well, 2 mm clearance around the collar each side
WELL_D = 10.0 * MM
POST_R, POST_H = 3.0 * MM, 7.0 * MM  # post in the well, pushes the poppet 4 mm up when the reservoir is seated
SEAT_Y = GAP + RW / 2              # seated reservoir centre, y
WX, WY = VX, SEAT_Y                # well centre in the brewer frame
BREWER_MASS = 4.4                  # 10.6 lb listed with the reservoir, less 0.4 kg for the reservoir


def _brewer_shapes():
    z0, z1 = LEDGE - WELL_D, LEDGE
    wx0, wx1, wy0, wy1 = WX - WELL / 2, WX + WELL / 2, WY - WELL / 2, WY + WELL / 2
    zc, hz = (z0 + z1) / 2, z1 - z0
    bx = [
        ((BODY_X0 + BODY_D / 2, -BODY_W / 2, BODY_H / 2), (BODY_D, BODY_W, BODY_H), BLACK),          # body
        # ledge under the reservoir, built around the open valve well
        (((LEDGE_X0 + LEDGE_X1) / 2, LEDGE_Y1 / 2, z0 / 2), (LEDGE_X1 - LEDGE_X0, LEDGE_Y1, z0), DARK),
        (((LEDGE_X0 + wx0) / 2, LEDGE_Y1 / 2, zc), (wx0 - LEDGE_X0, LEDGE_Y1, hz), DARK),
        (((wx1 + LEDGE_X1) / 2, LEDGE_Y1 / 2, zc), (LEDGE_X1 - wx1, LEDGE_Y1, hz), DARK),
        ((WX, wy0 / 2, zc), (WELL, wy0, hz), DARK),
        ((WX, (wy1 + LEDGE_Y1) / 2, zc), (WELL, LEDGE_Y1 - wy1, hz), DARK),
        # drip tray and brew head (look only)
        ((BODY_X0 - 0.035, -BODY_W / 2, 0.010), (0.070, 0.130, 0.020), GREY, None, None, "visual"),
        ((BODY_X0 + 0.004, -BODY_W / 2, 0.110), (0.008, 0.120, 0.150), DARK, None, None, "visual"),
    ]
    cy = [((WX, WY, z0 + POST_H / 2), POST_R, POST_H, STEEL)]
    return bx, cy


_bb, _bc = _brewer_shapes()
BREWER_XY = (0.475, 0.055)         # world position of the brewer frame origin
BREWER = part("brewer", pos=(*BREWER_XY, TOP), mass=BREWER_MASS, color=BLACK, friction=0.9, boxes=_bb, cylinders=_bc)

RES_START = (0.255, 0.215)         # reservoir bottom centre at the start, world x, y
RESERVOIR = part("reservoir", pos=(*RES_START, TOP + COL_H), mass=RES_MASS, color=CLEARBLUE, friction=0.5,
                 boxes=_reservoir_boxes(), sliders=(POPPET,))
OBJECTS = [BREWER, RESERVOIR]

# ---- scoring ----------------------------------------------------------------------------------------------------
SEAT_Z_TOL = 1.5 * MM       # seated: reservoir bottom within 1.5 mm of the ledge top
TILT_SEATED = 1.5           # deg
OPEN_MIN = 3.0 * MM         # valve open: poppet pushed up at least 3 mm of its 5 mm travel (4 mm when seated)
ENGAGE_DEPTH = 3.0 * MM     # collar in the well: collar mouth at least 3 mm below the ledge top, inside the well
RELEASE_CLEAR = 0.030
STILL_V, STILL_W = 0.01, 0.10
BREWER_MOVE = 0.010
BREWER_TILT = 3.0
DROP_V = 0.9
_STATE: dict = {}


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(brewer0=None, gate=None, tcp={}, still_since=None, t=0.0)


_reset()


def _tcps(environment):
    from yam_arena import ik, station

    base = environment.unwrapped
    out = {}
    for side in ("left", "right"):
        rob = base.scene[f"{side}_robot"]
        q = np.asarray(getattr(rob.data.joint_pos, "torch", rob.data.joint_pos)[0].detach().cpu())[:6]
        out[side] = ik.tcp_pose(q, station.BASES[side])[0]
    return out


def _tilt(p) -> float:
    return math.degrees(math.acos(float(np.clip(p.rotation()[2, 2], -1.0, 1.0))))


@on_step
def _watch(environment) -> None:
    br = _part_state(environment, BREWER)
    rs = _part_state(environment, RESERVOIR)
    if _STATE["brewer0"] is None:
        _STATE["brewer0"] = br.pos.copy()
    if np.linalg.norm(br.pos[:2] - _STATE["brewer0"][:2]) > BREWER_MOVE or _tilt(br) > BREWER_TILT:
        _STATE["gate"] = _STATE["gate"] or "brewer moved or tipped"
    if rs.pos[2] < TOP - 0.05 or rs.rotation()[2, 2] < 0.0:
        _STATE["gate"] = _STATE["gate"] or "reservoir off the table or upside down"
    if float(rs.lin_vel[2]) < -DROP_V:
        _STATE["gate"] = _STATE["gate"] or "reservoir dropped"
    _STATE["tcp"] = _tcps(environment)
    _STATE["t"] += 0.05
    still = float(np.linalg.norm(rs.lin_vel)) < STILL_V and float(np.linalg.norm(rs.ang_vel)) < STILL_W
    _STATE["still_since"] = (_STATE["still_since"] if _STATE["still_since"] is not None else _STATE["t"]) if still else None


def valve(objects) -> dict:
    """Where the collar is relative to the well (in the brewer frame), the reservoir's height and tilt, and the
    poppet lift."""
    br, rs = objects["brewer"], objects["reservoir"]
    mouth = rs.to_world((VX, 0.0, -COL_H))                 # collar mouth centre
    m = br.to_local(mouth)
    floor = br.to_local(rs.to_world((0.0, 0.0, 0.0)))
    lift = abs(float(rs.sliders.get("poppet", 0.0)))
    return {"dx": float(m[0] - WX), "dy": float(m[1] - WY), "mouth_below_ledge": float(LEDGE - m[2]),
            "bottom_above_ledge": float(floor[2] - LEDGE), "tilt": _tilt(rs), "lift": lift}


def _released(objects) -> bool:
    rs = objects["reservoir"]
    tcp = _STATE.get("tcp") or {}
    if not tcp:
        return True
    lo, hi = np.asarray(rs.low), np.asarray(rs.high)
    return all(np.linalg.norm(np.maximum(np.maximum(lo - t, t - hi), 0.0)) >= RELEASE_CLEAR for t in tcp.values())


def _update(objects):
    if _STATE.get("gate"):
        return 0.0
    v = valve(objects)
    inside = abs(v["dx"]) <= (WELL - COL_O) / 2 + 0.5 * MM and abs(v["dy"]) <= (WELL - COL_O) / 2 + 0.5 * MM
    engaged = inside and v["mouth_below_ledge"] >= ENGAGE_DEPTH
    seated = engaged and abs(v["bottom_above_ledge"]) <= SEAT_Z_TOL and v["tilt"] <= TILT_SEATED and v["lift"] >= OPEN_MIN
    rest = _released(objects) and _STATE.get("still_since") is not None and _STATE["t"] - _STATE["still_since"] >= 1.0
    if not rest:
        return 0.0
    if seated:
        return 1.0
    if engaged:
        return 0.5
    return 0.0


def success(objects) -> bool:
    return _update(objects) >= 1.0


def score(objects) -> float:
    """0 after a gate (brewer moved over 10 mm or tipped over 3 deg; reservoir dropped, off the table or upside
    down) and 0 unless the reservoir is released (both grippers 30 mm clear) and still for 1 s. 1.0: seated (collar
    in the well, bottom within 1.5 mm of the ledge, tilt under 1.5 deg, valve poppet up 3 mm or more). 0.5: collar
    in the well (its mouth 3 mm or more below the ledge top) but not seated. Otherwise 0."""
    return _update(objects)


def report(objects) -> dict:
    v = valve(objects)
    return {"valve_mm": {k: round(val * 1000, 2) if k != "tilt" else round(val, 2) for k, val in v.items()},
            "gate": _STATE.get("gate"), "score": _update(objects), "released": _released(objects),
            "reservoir_pos_mm": [round(float(x) * 1000, 1) for x in objects["reservoir"].pos],
            "brewer_pos_mm": [round(float(x) * 1000, 1) for x in objects["brewer"].pos]}


def debug_line(objects) -> str:
    v = valve(objects)
    return (" ".join(f"{k} {val*1000:.1f}" if k != "tilt" else f"tilt {val:.2f}" for k, val in v.items())
            + f" gate {_STATE.get('gate')} score {_update(objects)}")
