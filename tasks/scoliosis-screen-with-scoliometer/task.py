"""School scoliosis screen with a scoliometer (Adams forward-bend test) on a training manikin's back.

A rigid adolescent back manikin (the forward-bent trunk, back horizontal and facing up, shoulders toward the robots)
lies on a dark stand on the bench, its spine ridge along x. Its skin shell rides on a stiff vertical spring (a padded
manikin) so the load on the back can be measured. One thoracic level carries a rib hump: the back is rotated about
the spine by up to 3 to 11 deg there (the angle of trunk rotation, ATR), fading over a few centimetres. An OSI
(Bunnell) Scoliometer stands in its open case in front of the right arm: a 19 x 8.9 cm plate with a ball in a curved
tube read against a 0-30 deg scale each way, a semicircular notch in the middle of its bottom edge for the spinous
processes, and two finger pads at its top centre. A referral slip block lies at the left with a red REFER tray and
a green PASS tray.

Procedure (Bunnell 1984; Minnesota school screening manual): lay the scoliometer across the back with the notch over
the spine and both ends on the skin, without pressing, and slide it from the upper thoracic spine to the lumbar spine,
noting the largest reading. Refer if the angle of trunk rotation is 7 deg or more (Bunnell 1993).
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from robokit import _ENV, Slider, _np, _part_state, on_reset, on_step, part, write_pose  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Screen the training manikin's back with the scoliometer: notch on the spine, both feet on the back, "
               "slide it from the shoulders to the waist; put the slip in REFER if it reads 7 degrees or more, else "
               "PASS. Then put the scoliometer back in its case.")


def srgb(r, g, b):
    return (r / 255, g / 255, b / 255)


SKIN, SKIN2, DARK, GREY = srgb(222, 178, 150), srgb(205, 160, 134), srgb(45, 47, 50), srgb(120, 124, 128)
WHITE, TICK, RED, TUBE, BALL_C = srgb(238, 238, 232), srgb(20, 20, 22), srgb(210, 30, 30), srgb(178, 214, 226), srgb(70, 72, 78)
CASE_C, SLIP_C, CARD_C = srgb(30, 50, 110), srgb(236, 236, 230), srgb(240, 200, 40)
REFER_C, PASS_C = srgb(200, 45, 40), srgb(40, 150, 70)


def _quat_wxyz(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([w, x, y, z])
    return tuple((q / np.linalg.norm(q)).tolist())


def _rx(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


# ---- the back (torso frame: origin on the bench under the spine at x TORSO_X; x along the spine, shoulders at -x) -----
TORSO_X, SPINE_Y = 0.35, -0.12
X0, X1 = 0.18, 0.52                 # world x of the shoulder and waist ends of the skin shell
SWEEP = (0.22, 0.47)                # screening span: upper thoracic (T2) to lumbar (L3), 25 cm
H0, H_RISE = 0.080, 0.022           # spine line (the paraspinal skin level) 8.0 cm above the bench at the ends, 10.2 cm mid-back
BETA = math.radians(5.0)            # the rib cage falls away from the spine at 5 deg each side
HALF_W, PITCH, CAP_R = 0.120, 0.006, 0.012
SIGMA = 0.050                       # rib hump: ATR(x) = A exp(-(x - xp)^2 / (2 sigma^2))
RIDGE_UP, RIDGE_S = 0.0045, 0.004   # spinous processes: a ridge 4.5 mm proud, about 1.5 cm wide at its foot
PARA_UP, PARA_U, PARA_S = 0.005, 0.025, 0.012   # paraspinal muscles: 5 mm bulges centred 25 mm either side
RIDGE_R = 0.005
K_SHELL, C_SHELL, M_SHELL = 1500.0, 60.0, 0.8
SHELL_PRELOAD = M_SHELL * 9.81 / K_SHELL
# eight manikin backs (peak ATR in deg, + = the robot-left (+y) side high; world x of the peak); the seed picks one
VARIANTS = [(9.5, 0.31), (-4.0, 0.28), (-10.5, 0.35), (5.0, 0.33), (3.0, 0.29), (-8.5, 0.30), (11.0, 0.27),
            (-5.5, 0.36)]
REFER_DEG = 7.0
PARK = [(3.0, -1.75 + 0.5 * k, TOP) for k in range(len(VARIANTS))]


def h_of(xw):
    return H0 + H_RISE * math.sin(math.pi * (xw - X0) / (X1 - X0))


def dh_of(xw):
    return H_RISE * math.pi / (X1 - X0) * math.cos(math.pi * (xw - X0) / (X1 - X0))


def atr_of(xw, A, xp):
    return A * math.exp(-((xw - xp) ** 2) / (2 * SIGMA ** 2))


def profile(u):
    """Skin height across the back (m) at lateral distance u from the spine, before the hump's rotation."""
    u = abs(u)
    return (RIDGE_UP * math.exp(-u * u / (2 * RIDGE_S ** 2)) + PARA_UP * math.exp(-((u - PARA_U) ** 2) / (2 * PARA_S ** 2))
            - u * math.tan(BETA))


def section_rot(xw, A, xp):
    return _ry(-math.atan(dh_of(xw))) @ _rx(math.radians(atr_of(xw, A, xp)))


FLANK_U, FLANK_K = 0.110, 60.0      # the flanks round over beyond 11 cm from the spine (drawn skin only)


def skin_point(xw, u, A, xp):
    """A point of the drawn skin in the torso frame."""
    w = profile(u) - FLANK_K * max(0.0, abs(u) - FLANK_U) ** 2
    p = np.array([xw - TORSO_X, 0.0, h_of(xw)]) + section_rot(xw, A, xp) @ np.array([0.0, u, w])
    p[2] = max(p[2], 0.034)
    return p


N_SLICE = int(round((X1 - X0) / PITCH))
SLICE_X = [X0 + (i + 0.5) * PITCH for i in range(N_SLICE)]
# collider segments across each side (u from, u to): inner and outer paraspinal muscle, rib cage; each a 12 mm capsule whose top
# runs along the chord of the skin profile, lifted to touch its highest point
SEGS = ((0.006, 0.025), (0.025, 0.052), (0.052, 0.118))


def _seg_line(u0, u1):
    a = np.array([u0, profile(u0)])
    b = np.array([u1, profile(u1)])
    d = (b - a) / np.linalg.norm(b - a)
    n = np.array([-d[1], d[0]])
    lift = max(float(np.dot(np.array([u, profile(u)]) - a, n)) for u in np.linspace(u0, u1, 40))
    return a + n * lift, b + n * lift, n


def _capsules(A, xp):
    """Colliders of one back in the torso frame: (end a, end b, radius) per capsule."""
    out = []
    for xw in SLICE_X:
        R = section_rot(xw, A, xp)
        o = np.array([xw - TORSO_X, 0.0, h_of(xw)])
        for s in (1.0, -1.0):
            for u0, u1 in SEGS:
                a, b, n = _seg_line(u0, u1)
                pa = o + R @ np.array([0.0, s * a[0], a[1] - CAP_R * n[1]]) + R @ np.array([0.0, -s * CAP_R * n[0], 0.0])
                pb = o + R @ np.array([0.0, s * b[0], b[1] - CAP_R * n[1]]) + R @ np.array([0.0, -s * CAP_R * n[0], 0.0])
                out.append((pa, pb, CAP_R))
    for xc in np.arange(X0 + 0.010, X1 - 0.009, 0.020):   # the spinous ridge, along the spine
        R = section_rot(xc, A, xp)
        c = np.array([xc - TORSO_X, 0.0, h_of(xc)]) + R @ np.array([0.0, 0.0, profile(0.0) - RIDGE_R])
        ax = R @ np.array([1.0, 0.0, 0.0])
        out.append((c - ax * 0.010, c + ax * 0.010, RIDGE_R))
    return out


CAPS = [_capsules(A, xp) for A, xp in VARIANTS]


def _mesh(A, xp):
    """Drawn skin: a smooth triangle mesh of the back with skirts down to the stand."""
    xs = np.linspace(X0, X1, 69)
    us = np.linspace(-0.135, 0.135, 109)
    P = np.array([[skin_point(x, u, A, xp) for u in us] for x in xs])
    low = 0.034
    G = np.zeros((len(xs) + 2, len(us) + 2, 3))
    G[1:-1, 1:-1] = P
    G[1:-1, 0] = P[:, 0]
    G[1:-1, -1] = P[:, -1]
    G[1:-1, 0, 2] = G[1:-1, -1, 2] = low
    G[0] = G[1]
    G[-1] = G[-2]
    G[0, :, 2] = G[-1, :, 2] = low
    nx, nu = G.shape[:2]
    idx = np.arange(nx * nu).reshape(nx, nu)
    f = []
    for i in range(nx - 1):
        for j in range(nu - 1):
            a, b, c, d = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
            f += [(a, d, c), (a, c, b)]
    return G.reshape(-1, 3), np.array(f)


def _quat_from_axis(d, axis="X"):
    """Rotation (w, x, y, z) taking the capsule's local axis onto direction d."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    e = np.array({"X": [1.0, 0.0, 0.0], "Y": [0.0, 1.0, 0.0], "Z": [0.0, 0.0, 1.0]}[axis])
    v = np.cross(e, d)
    c = float(np.dot(e, d))
    if np.linalg.norm(v) < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    R = np.eye(3) + K + K @ K / (1 + c)
    return _quat_wxyz(R)


def _torso(k):
    cyl = [(tuple(((a + b) / 2).tolist()), r, float(np.linalg.norm(b - a)), SKIN, "X", "capsule_hidden",
            _quat_from_axis(b - a), None, 0.3) for a, b, r in CAPS[k]]
    shell = Slider("shell", rest=(0.0, 0.0, 0.0), axis="Z", travel=0.04, both_ways=True, rate=K_SHELL,
                   damping=C_SHELL, mass=M_SHELL, start=SHELL_PRELOAD, cylinders=tuple(cyl))
    root = [((0.0, 0.0, 0.015), (X1 - X0 + 0.02, 2 * HALF_W + 0.02, 0.030), DARK)]
    pts, faces = _mesh(*VARIANTS[k])
    return part(f"torso{k}", pos=PARK[k], mass=20.0, color=DARK, friction=0.8, boxes=root, sliders=(shell,),
                meta={"meshes": [(pts, faces, SKIN)]})


TORSOS = [_torso(k) for k in range(len(VARIANTS))]
PARKING = Box(name="parking", pos=(3.0, 0.0, TOP - 0.01), size=(0.8, 4.6, 0.02), color=GREY)

# ---- scoliometer (gauge frame: origin between the two finger pads at the top centre; x through the plate, the scale
# on the -x face; y along the plate; z up) ---------------------------------------------------------------------------
G_LEN, G_H, G_T = 0.190, 0.089, 0.010
PIVOT_UP = 0.081                    # finger pads 81 mm above the foot tips
FOOT_Y, FOOT_R = 0.088, 0.003       # foot pads at +-88 mm: capsules across the plate, 3 mm radius
NOTCH_W, NOTCH_H = 0.030, 0.024
BODY_LOW = 0.016                    # plate's lower edge 16 mm above the foot tips (outside the notch)
BALL_R_ARC, BALL_LOW = 0.140, 0.030  # tube: 140 mm radius, lowest ball centre 30 mm above the foot tips
BALL_PIVOT_Z = BALL_LOW + BALL_R_ARC - PIVOT_UP
TICK_R = BALL_R_ARC - 0.009         # tick marks just above the tube
BALL_RAD = 0.004
FACE_X = -G_T / 2
G_MASS = 0.080


def _fz(z):   # height above the foot tips -> gauge frame z
    return z - PIVOT_UP


def _gauge_shapes():
    half = G_LEN / 2
    side_w = half - NOTCH_W / 2
    boxes = [((0.0, s * (NOTCH_W / 2 + side_w / 2), _fz((BODY_LOW + G_H) / 2)), (G_T, side_w, G_H - BODY_LOW), WHITE)
             for s in (-1, 1)]
    boxes.append(((0.0, 0.0, _fz((NOTCH_H + G_H) / 2)), (G_T, NOTCH_W, G_H - NOTCH_H), WHITE))
    for s in (-1, 1):   # legs under the two ends
        boxes.append(((0.0, s * FOOT_Y, _fz((FOOT_R + BODY_LOW) / 2)), (G_T, 0.012, BODY_LOW - FOOT_R), WHITE, None,
                      0.15))
    # scale on the -x face: tube band, ticks every deg (long every 5 deg), red zero mark
    cz = _fz(BALL_LOW + BALL_R_ARC)
    for a in range(-32, 33, 2):
        t = math.radians(a)
        boxes.append(((FACE_X - 0.0004, BALL_R_ARC * math.sin(t), cz - BALL_R_ARC * math.cos(t)),
                      (0.0008, 0.0052, 0.010), TUBE, ("Q", _quat_wxyz(_rx(t))), None, "visual"))
    for a in range(-30, 31):
        t = math.radians(a)
        L = 0.008 if a % 5 == 0 else 0.004
        col, w = (RED, 0.0030) if a == 0 else (TICK, 0.0010 if a % 5 else 0.0014)
        r = TICK_R - L / 2
        boxes.append(((FACE_X - 0.0006, r * math.sin(t), cz - r * math.cos(t)), (0.0008, w, L), col,
                      ("Q", _quat_wxyz(_rx(t))), None, "visual"))
    cyl = [((0.0, s * FOOT_Y, _fz(FOOT_R)), FOOT_R, 0.008, WHITE, "Y", "capsule", None, None, 0.15) for s in (-1, 1)]
    cyl += [((s * (G_T / 2 + 0.001), 0.0, 0.0), 0.004, 0.0, srgb(60, 60, 64), "Z", "sphere") for s in (-1, 1)]
    return boxes, cyl


_GB, _GC = _gauge_shapes()
BALL = Slider("ball", rest=(0.0, 0.0, BALL_PIVOT_Z), kind="revolute", axis="X", travel=0.60, both_ways=True, rate=0.0,
              damping=1.0e-3, mass=0.004, inertia=(1e-7, 1e-7, 1e-7), com=(0.0, 0.0, -BALL_R_ARC),
              cylinders=(((FACE_X - 0.0035, 0.0, BALL_PIVOT_Z - BALL_R_ARC), BALL_RAD, 0.0, BALL_C, "Z",
                          "sphere_visual"),))
CASE_X, CASE_Y = 0.24, -0.37
CASE_FLOOR = 0.006
SLOT_HALF = 0.009
GAUGE_Z0 = TOP + CASE_FLOOR + PIVOT_UP
GAUGE = part("gauge", pos=(CASE_X, CASE_Y, GAUGE_Z0), mass=G_MASS, color=WHITE, friction=0.6, boxes=_GB,
             cylinders=_GC, sliders=(BALL,))


def _case_boxes():
    L = G_LEN + 0.013
    b = [((0.0, 0.0, CASE_FLOOR / 2), (0.05, L + 0.012, CASE_FLOOR), CASE_C)]
    for s in (-1, 1):
        b.append(((s * (SLOT_HALF + 0.003), 0.0, CASE_FLOOR + 0.0175), (0.006, L + 0.012, 0.035), CASE_C))
        b.append(((0.0, s * (L / 2 + 0.003), CASE_FLOOR + 0.010), (2 * SLOT_HALF, 0.006, 0.020), CASE_C))
    return b


CASE = part("case", pos=(CASE_X, CASE_Y, TOP), mass=0.4, color=CASE_C, friction=0.8, boxes=_case_boxes())

# ---- referral slip block and the two trays ----------------------------------------------------------------------------
SLIP_X, SLIP_Y = 0.22, 0.22
SLIP_SIZE = (0.030, 0.060, 0.045)
SLIP = part("slip", pos=(SLIP_X, SLIP_Y, TOP + SLIP_SIZE[2] / 2), mass=0.04, color=SLIP_C, friction=0.9,
            boxes=[((0.0, 0.0, 0.0), SLIP_SIZE, SLIP_C),
                   ((0.0, 0.0, SLIP_SIZE[2] / 2 + 0.0004), (0.022, 0.050, 0.0008), CARD_C, None, None, "visual")])
TRAY_IN, TRAY_WALL, TRAY_H, TRAY_FLOOR = 0.120, 0.004, 0.020, 0.004
TRAYS = {"refer": (0.34, 0.33), "pass": (0.34, 0.15)}


def _tray(name, color):
    x, y = TRAYS[name]
    o = TRAY_IN / 2 + TRAY_WALL / 2
    b = [((0.0, 0.0, TRAY_FLOOR / 2), (TRAY_IN + 2 * TRAY_WALL, TRAY_IN + 2 * TRAY_WALL, TRAY_FLOOR), color)]
    for s in (-1, 1):
        b.append(((s * o, 0.0, TRAY_H / 2), (TRAY_WALL, TRAY_IN + 2 * TRAY_WALL, TRAY_H), color))
        b.append(((0.0, s * o, TRAY_H / 2), (TRAY_IN, TRAY_WALL, TRAY_H), color))
    return part(f"{name}_tray", pos=(x, y, TOP), mass=0.3, color=color, friction=0.8, boxes=b)


REFER_TRAY, PASS_TRAY = _tray("refer", REFER_C), _tray("pass", PASS_C)
OBJECTS = TORSOS + [PARKING, GAUGE, CASE, SLIP, REFER_TRAY, PASS_TRAY]

# ---- scoring ----------------------------------------------------------------------------------------------------------
CONTROL_DT = 0.05
TOUCH = 0.0015                # a foot within 1.5 mm of the skin is in contact
NOTCH_TOL = 0.010             # notch within 10 mm of the spine line
ACROSS_MAX = 20.0             # the plate within 20 deg of square to the spine
BIN = 0.005
N_BINS = int(round((SWEEP[1] - SWEEP[0]) / BIN))
COVER_025 = 0.60
PEAK_WIN = 0.020
BOTH_FRAC = 0.90
FORCE_MAX = 20.0
TORSO_MOVE_MAX = 0.020
HANDS_CLEAR = 0.05
_STATE: dict = {}


def _box_dist(p, s):
    d = np.abs(np.asarray(p)) - np.asarray(s) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)) + min(max(d[0], d[1], d[2]), 0.0))


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


def skin_gap(k, p_shell):
    """Distance (m) from a point in the shell frame (torso frame minus the shell's travel) to the skin colliders of
    back k (the capsules of the nearest slices)."""
    i = int(round((p_shell[0] + TORSO_X - X0) / PITCH - 0.5))
    n = 2 * len(SEGS)
    best = 1.0
    for j in range(max(0, i - 2), min(N_SLICE, i + 3)):
        for a, b, r in CAPS[k][n * j:n * j + n]:
            best = min(best, _seg_dist(p_shell, a, b) - r)
    for a, b, r in CAPS[k][n * N_SLICE:]:   # the spinous ridge
        best = min(best, _seg_dist(p_shell, a, b) - r)
    return best


def gauge_readings(k, torso, shell_q, gauge):
    """Feet gaps (m), notch level (world-equivalent x) and lateral offset (m), angle to square (deg)."""
    def to_shell(pw):
        return torso.to_local(pw) - np.array([0.0, 0.0, shell_q])

    gaps = []
    for s in (-1, 1):
        g = 1.0
        for dy in (-0.004, 0.0, 0.004):
            g = min(g, skin_gap(k, to_shell(gauge.to_world((0.0, s * FOOT_Y + dy, _fz(FOOT_R))))) - FOOT_R)
        gaps.append(g)
    n = to_shell(gauge.to_world((0.0, 0.0, _fz(BODY_LOW))))
    ay = torso.rotation().T @ gauge.rotation()[:, 1]
    across = math.degrees(math.atan2(abs(ay[0]), math.hypot(ay[1], ay[2])))
    return gaps, float(n[0] + TORSO_X), float(n[1]), across


def _active():
    return _STATE.get("k", 0)


def _settle_joints(environment, name):
    """Re-write the joint state after teleporting an articulation's root, so its links follow the root at once."""
    import torch

    asset = environment.unwrapped.scene[name]
    q = asset.data.joint_pos
    q = getattr(q, "torch", q).clone()
    asset.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q))


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(gate=None, k=0, torso0=None, bins=np.zeros(N_BINS, bool), on_back=0, both=0, last=None, tcp={},
                  max_force=0.0, max_read=0.0)
    if environment is None:
        return
    seed = _ENV.get("seed")
    k = (int(seed) if seed is not None else 0) % len(VARIANTS)
    _STATE["k"] = k
    for j in range(len(VARIANTS)):
        pos = (TORSO_X, SPINE_Y, TOP) if j == k else PARK[j]
        write_pose(environment, f"torso{j}", pos, (0.0, 0.0, 0.0, 1.0))
        _settle_joints(environment, f"torso{j}")
    # commit the teleport with one physics step: otherwise the next control step's write-back leaves the shell
    # link behind and the joint drags the back across the bench
    environment.unwrapped.sim.step(render=False)
    environment.unwrapped.scene.update(environment.unwrapped.physics_dt)
    A, xp = VARIANTS[k]
    _STATE["start"] = dict(variant=k, peak_atr_deg=A, peak_x_mm=round(xp * 1000, 1),
                           answer="refer" if abs(A) >= REFER_DEG else "pass")


_reset()


@on_step
def _watch(environment) -> None:
    k = _active()
    torso = _part_state(environment, TORSOS[k])
    gauge = _part_state(environment, GAUGE)
    if _STATE["torso0"] is None:
        _STATE["torso0"] = torso.pos.copy()
    _STATE["tcp"] = _tcps(environment)
    q = torso.sliders.get("shell", 0.0)
    force = K_SHELL * (-q)
    gaps, lvl, off, across = gauge_readings(k, torso, q, gauge)
    reading = -math.degrees(gauge.sliders.get("ball", 0.0))
    _STATE["last"] = dict(gaps=gaps, lvl=lvl, off=off, across=across, force=force, q=q, reading=reading)
    if _STATE["gate"]:
        return
    _STATE["max_force"] = max(_STATE["max_force"], force)
    if force > FORCE_MAX:
        _STATE["gate"] = f"load on the back {force:.1f} N > 20 N"
    elif gauge.pos[2] < TOP - 0.03:
        _STATE["gate"] = "scoliometer fell off the table"
    elif float(np.linalg.norm(torso.pos - _STATE["torso0"])) > TORSO_MOVE_MAX:
        _STATE["gate"] = "manikin moved more than 20 mm"
    if _STATE["gate"]:
        return
    touch = [g <= TOUCH for g in gaps]
    if SWEEP[0] <= lvl <= SWEEP[1] and any(touch):
        _STATE["on_back"] += 1
        if all(touch):
            _STATE["both"] += 1
            _STATE["max_read"] = max(_STATE["max_read"], abs(reading))
            if abs(off) <= NOTCH_TOL and across <= ACROSS_MAX:
                _STATE["bins"][min(N_BINS - 1, int((lvl - SWEEP[0]) / BIN))] = True


def _peak_bins():
    xp = VARIANTS[_active()][1]
    c = SWEEP[0] + (np.arange(N_BINS) + 0.5) * BIN
    return (c >= xp - PEAK_WIN) & (c <= xp + PEAK_WIN)


def _inside(obj, tray_name, objects):
    tray = objects[f"{tray_name}_tray"]
    hx, hy, hz = (v / 2 for v in SLIP_SIZE)
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = tray.to_local(obj.to_world((sx * hx, sy * hy, -hz)))
            if abs(c[0]) > TRAY_IN / 2 or abs(c[1]) > TRAY_IN / 2 or not (TRAY_FLOOR - 0.003 <= c[2] <= TRAY_FLOOR + 0.010):
                return False
    return float(np.linalg.norm(obj.lin_vel)) < 0.02


def _gauge_in_case(objects) -> bool:
    g, c = objects["gauge"], objects["case"]
    for s in (-1, 1):
        p = c.to_local(g.to_world((0.0, s * FOOT_Y, _fz(0.0))))
        if abs(p[0]) > SLOT_HALF or abs(p[1]) > G_LEN / 2 + 0.0065 + 0.003 or p[2] > CASE_FLOOR + 0.005:
            return False
    up = c.rotation().T @ g.rotation()[:, 2]
    return up[2] >= math.cos(math.radians(15)) and float(np.linalg.norm(g.lin_vel)) < 0.01


def _hands_clear(objects) -> bool:
    pts = [objects["gauge"].pos, objects["slip"].pos]
    return all(float(np.linalg.norm(t - p)) >= HANDS_CLEAR for t in (_STATE.get("tcp") or {}).values() for p in pts)


def _update(objects):
    if _STATE.get("gate"):
        return 0.0, False
    bins = _STATE.get("bins")
    if bins is None or bins.mean() < COVER_025:
        return 0.0, False
    ob = _STATE.get("on_back", 0)
    if not (bins[_peak_bins()].all() and ob > 0 and _STATE["both"] / ob >= BOTH_FRAC):
        return 0.25, False
    if not _inside(objects["slip"], _STATE["start"]["answer"], objects):
        return 0.5, False
    if _gauge_in_case(objects) and _hands_clear(objects):
        return 1.0, True
    return 0.75, False


def success(objects) -> bool:
    return _update(objects)[1]


def score(objects) -> float:
    """Score 0 if at any point the load on the manikin's back exceeds 20 N, the scoliometer falls off the table, or the
    manikin moves more than 20 mm. Contact: a foot within 1.5 mm of the skin. 0.25 when the control steps with both
    feet in contact, the notch within 10 mm of the spine line and the plate within 20 deg of square to the spine cover
    at least 60% of the 25 cm screening span (5 mm bins, upper thoracic to lumbar). 0.5 when, in addition, those steps
    cover every bin within 20 mm of the hump's peak level and both feet were in contact for at least 90% of the steps
    with a foot on the back inside the span. 0.75 when, in addition, the slip block rests wholly inside the correct
    tray: REFER if the peak angle of trunk rotation is 7 deg or more, else PASS. 1.0 (success, held 1 s) when, in
    addition, the scoliometer stands in its case and no gripper centre is within 5 cm of it or of the slip."""
    return _update(objects)[0]


def report(objects) -> dict:
    bins = _STATE.get("bins")
    ob = _STATE.get("on_back", 0)
    t0 = _STATE.get("torso0")
    torso = objects[f"torso{_active()}"]
    return {"start": _STATE.get("start"), "gate": _STATE.get("gate"),
            "coverage": None if bins is None else round(float(bins.mean()), 3),
            "peak_bins_covered": None if bins is None else int(bins[_peak_bins()].sum()),
            "peak_bins": int(_peak_bins().sum()), "on_back_steps": ob, "both_feet_steps": _STATE.get("both"),
            "both_frac": round(_STATE["both"] / ob, 3) if ob else None,
            "max_force_N": round(_STATE.get("max_force", 0.0), 2), "max_reading_deg": round(_STATE.get("max_read", 0.0), 2),
            "slip_in": {t: _inside(objects["slip"], t, objects) for t in TRAYS},
            "gauge_in_case": _gauge_in_case(objects), "hands_clear": _hands_clear(objects),
            "torso_shift_mm": round(float(np.linalg.norm(torso.pos - t0)) * 1000, 1) if t0 is not None else None,
            "score": _update(objects)[0]}


def debug_line(objects) -> str:
    L = _STATE.get("last")
    if not L:
        return "no reading"
    g = objects["gauge"]
    bins = _STATE.get("bins")
    tc = (_STATE.get("tcp") or {}).get("right")
    return (f"gaps {[round(v*1000,1) for v in L['gaps']]} lvl {L['lvl']*1000:.0f} off {L['off']*1000:.1f} "
            f"across {L['across']:.1f} F {L['force']:.1f} read {L['reading']:.2f} cov {bins.mean():.2f} "
            f"ob {_STATE['on_back']} both {_STATE['both']} gate {_STATE.get('gate')} score {_update(objects)[0]} "
            f"gauge {np.round(g.pos*1000,0).tolist()} tcpR {None if tc is None else np.round(tc*1000,0).tolist()}")
