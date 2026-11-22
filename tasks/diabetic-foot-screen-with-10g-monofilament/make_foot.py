"""Build the life-size foot model (foot_mesh.npz) for task.py: a smooth adult right foot sculpted as a signed-distance
field (heel, medial arch and lateral column, metatarsal-head pads on a transverse arc, five toes with pulps, dorsum,
ankle cut flat where it is bolted to the stand) and meshed with marching cubes (1.2 mm grid).

Frame F (metres): u heel -> toes, v toward the little toe, w out of the sole. Origin: the skin over the 3rd
metatarsal head. Needs numpy, scipy and scikit-image; task.py only loads the .npz.

Colliders (PhysX needs convex pieces on a moving body): the plantar skin is tiled with convex prisms (4 mm on the forefoot
and toes, 8 mm on the heel and arch) whose top corners lie on the surface, over a core of convex slabs
cut from the surface eroded by 3 mm. Every piece is checked to lie inside the skin (+0.3 mm).

    python make_foot.py [out.npz]
"""
import math
import sys

import numpy as np
from scipy.spatial import ConvexHull
from skimage.measure import marching_cubes


def ell(p, c, r):
    q = (p - c) / r
    k0 = np.linalg.norm(q, axis=-1)
    k1 = np.linalg.norm(q / r, axis=-1)
    return k0 * (k0 - 1.0) / np.maximum(k1, 1e-9)


def cap(p, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[..., None] * ab), axis=-1) - r


def smin(a, b, k):
    h = np.clip(0.5 + 0.5 * (b - a) / k, 0.0, 1.0)
    return b * (1 - h) + a * h - k * h * (1 - h)


A = np.array
ARC_W, ARC_R = -0.085, 0.065                       # metatarsal heads under a thick fat pad: ellipsoids on a
MT_RAD = A([0.028, 0.020, 0.020])                  # transverse arc about u (skin 85 mm from the arc's axis)
MT = [(0.000, -23.6), (0.006, -11.5), (0.000, 0.0), (-0.007, 12.5), (-0.015, 25.0)]   # (u, arc angle deg)


def mt_centre(k):
    u, th = MT[k]
    t = math.radians(th)
    return A([u, ARC_R * math.sin(t), ARC_W + ARC_R * math.cos(t)])


def mt_dist(p, k):
    """Signed distance to the k-th metatarsal-head pad (an ellipsoid turned to its place on the arc)."""
    t = math.radians(MT[k][1])
    c, s_ = math.cos(t), math.sin(t)
    q = p - mt_centre(k)
    q = np.stack([q[..., 0], c * q[..., 1] - s_ * q[..., 2], s_ * q[..., 1] + c * q[..., 2]], -1)
    return ell(q, A([0.0, 0.0, 0.0]), MT_RAD)


HALLUX_TIP = (A([0.049, -0.027, -0.0135]), A([0.0180, 0.0150, 0.0105]))   # broad, flat pulp, tipped up 25 deg
HALLUX_TILT = math.radians(25.0)
TOES = [  # proximal phalanx capsule (a, b, r), pulp ellipsoid (c, radii)
    ((A([0.018, -0.029, -0.016]), A([0.040, -0.028, -0.015]), 0.0120), HALLUX_TIP),
    ((A([0.020, -0.008, -0.017]), A([0.038, -0.006, -0.016]), 0.0080), (A([0.045, -0.005, -0.016]), A([0.0095, 0.0080, 0.0075]))),
    ((A([0.016, 0.008, -0.018]), A([0.032, 0.010, -0.017]), 0.0076), (A([0.039, 0.011, -0.017]), A([0.0090, 0.0076, 0.0072]))),
    ((A([0.010, 0.022, -0.019]), A([0.024, 0.024, -0.018]), 0.0072), (A([0.030, 0.025, -0.018]), A([0.0085, 0.0072, 0.0068]))),
    ((A([0.000, 0.035, -0.020]), A([0.012, 0.038, -0.019]), 0.0066), (A([0.018, 0.039, -0.019]), A([0.0078, 0.0066, 0.0062]))),
]
MOUNT_W = -0.086                                     # ankle cut: the foot's back face, bolted to the stand's board


def parts(p):
    """Signed distances of the pieces: body (heel to the ball, dorsum, ankle) and the five toes."""
    heel = ell(p, A([-0.155, 0.006, -0.030]), A([0.040, 0.031, 0.030]))
    mid = ell(p, A([-0.080, 0.010, -0.034]), A([0.075, 0.034, 0.026]))
    lat = cap(p, (-0.150, 0.024, -0.012), (-0.030, 0.033, -0.017), 0.011)
    fore = ell(p, A([-0.006, 0.002, -0.022]), A([0.032, 0.048, 0.022]))
    dors = ell(p, A([-0.100, 0.004, -0.040]), A([0.090, 0.036, 0.038]))
    ank = cap(p, (-0.130, 0.004, -0.055), (-0.130, 0.004, -0.090), 0.030)
    ball = fore
    for k in range(5):
        ball = smin(ball, mt_dist(p, k), 0.006)
    body = smin(smin(smin(smin(heel, mid, 0.020), lat, 0.012), ball, 0.016), smin(dors, ank, 0.015), 0.012)
    toes = []
    for i, ((a, b, r), (c, rr)) in enumerate(TOES):
        if i == 0:   # the hallux pulp faces 25 deg toward the toe's tip (the big toe rests slightly extended)
            q = p - c
            ct, st = math.cos(HALLUX_TILT), math.sin(HALLUX_TILT)
            q = np.stack([ct * q[..., 0] - st * q[..., 2], q[..., 1], st * q[..., 0] + ct * q[..., 2]], -1)
            pulp = ell(q, A([0.0, 0.0, 0.0]), rr)
        else:
            pulp = ell(p, c, rr)
        toes.append(smin(cap(p, a, b, r), pulp, 0.004))
    return body, toes


def sdf(p):
    p = np.asarray(p, float)
    body, toes = parts(p)
    t = toes[0]
    for x in toes[1:]:
        t = np.minimum(t, x)
    d = smin(body, t, 0.007)
    return np.maximum(d, MOUNT_W - p[..., 2])          # flat ankle cut


def grad(p, h=2e-5):
    p = np.atleast_2d(np.asarray(p, float))
    g = np.stack([(sdf(p + h * e) - sdf(p - h * e)) / (2 * h) for e in np.eye(3)], -1)
    return g / np.linalg.norm(g, axis=-1, keepdims=True)


def ray_hit(o, d, t0=0.0, t1=0.08, iso=0.0):
    """First crossing of sdf = iso along o + t d (o inside), by bisection."""
    o, d = np.asarray(o, float), np.asarray(d, float) / np.linalg.norm(d)
    lo, hi = t0, t1
    for _ in range(60):
        m = (lo + hi) / 2
        if sdf(o + m * d) - iso < 0:
            lo = m
        else:
            hi = m
    return o + lo * d


def sites():
    """Test sites: skin point and outward normal (unit), in F. Each is the skin point on a ray from inside the
    metatarsal head (or the hallux pulp) along the arc's outward direction."""
    out = {}
    hd = A([math.tan(math.radians(20)), -math.tan(math.radians(10)), 1.0])
    for nm, o, d in [("hallux", HALLUX_TIP[0], hd),
                     ("mth1", mt_centre(0), A([0, math.sin(math.radians(-23.6)), math.cos(math.radians(-23.6))])),
                     ("mth3", mt_centre(2), A([0.0, 0.0, 1.0])),
                     ("mth5", mt_centre(4), A([0, math.sin(math.radians(25)), math.cos(math.radians(25))]))]:
        p = ray_hit(o, d)
        out[nm] = (p, grad(p)[0])
    return out


def surface_mesh(step=0.0012, iso=0.0):
    lo, hi = A([-0.205, -0.062, -0.090]), A([0.075, 0.066, 0.014])
    ax = [np.arange(lo[i], hi[i] + step, step) for i in range(3)]
    G = np.stack(np.meshgrid(*ax, indexing="ij"), -1)
    vol = sdf(G.reshape(-1, 3)).reshape(G.shape[:3])
    v, f, _, _ = marching_cubes(vol, iso, spacing=(step, step, step))
    v = v + lo
    return v, f


def plantar_tiles(step=0.004, depth=0.010, min_nz=0.45):
    """Convex prisms on the plantar skin: 4 surface corners + the same 4 moved `depth` into the foot along -w; 4 mm
    tiles on the forefoot and toes (where the sites are), 8 mm on the heel and arch."""
    us = np.r_[np.arange(-0.200, -0.036, 2 * step), np.arange(-0.036, 0.072, step)]
    vs = np.arange(-0.060, 0.066, step)
    H = np.full((len(us), len(vs)), np.nan)
    N = np.zeros((len(us), len(vs)))
    for i, u in enumerate(us):
        for j, v in enumerate(vs):
            o = A([u, v, 0.030])
            if sdf(o) <= 0:
                continue
            # march down along -w to the first inside point
            ts = np.arange(0.0, 0.110, 0.0005)
            P = o + ts[:, None] * A([0, 0, -1.0])
            d = sdf(P)
            k = np.nonzero(d < 0)[0]
            if len(k) == 0:
                continue
            t_lo, t_hi = ts[k[0] - 1], ts[k[0]]
            for _ in range(40):
                m = (t_lo + t_hi) / 2
                if sdf(o + m * A([0, 0, -1.0])) < 0:
                    t_hi = m
                else:
                    t_lo = m
            p = o + t_hi * A([0, 0, -1.0])
            if p[2] < MOUNT_W + 0.02:
                continue
            H[i, j] = p[2]
            N[i, j] = grad(p)[0][2]
    tiles = []
    for i in range(len(us) - 1):
        for j in range(len(vs) - 1):
            h = H[i:i + 2, j:j + 2]
            if np.isnan(h).any() or (N[i:i + 2, j:j + 2] < min_nz).any():
                continue
            top = [A([us[i + a], vs[j + b], H[i + a, j + b]]) for a in (0, 1) for b in (0, 1)]
            tiles.append(np.array(top + [t - A([0, 0, depth]) for t in top]))
    return np.array(tiles)


def core_hulls(erode=0.003, slab=0.010):
    """Convex slabs of the skin eroded by `erode`: the body cut into slabs along u, each toe into slabs along u."""
    v, f = surface_mesh(step=0.0016, iso=-erode)
    body, toes = parts(v)
    lab = np.argmin(np.stack([body] + toes, -1), -1)
    hulls = []
    for g in range(6):
        P = v[lab == g]
        if len(P) < 10:
            continue
        s = slab if g == 0 else 0.008
        edges = np.arange(P[:, 0].min(), P[:, 0].max() + s, s)
        for a, b in zip(edges[:-1], edges[1:]):
            Q = P[(P[:, 0] >= a - 0.0008) & (P[:, 0] <= b + 0.0008)]
            if len(Q) < 8:
                continue
            h = ConvexHull(Q)
            hulls.append(Q[h.vertices])
    return hulls


def hull_excess(pts, n=400, rng=np.random.default_rng(0)):
    """Largest sdf on a convex piece's surface (sampled on its hull faces): > 0 means it pokes out of the skin."""
    h = ConvexHull(pts)
    tri = pts[h.simplices]
    w = rng.dirichlet((1, 1, 1), size=(len(tri), max(1, n // len(tri) + 1)))
    S = np.einsum("tkj,tjd->tkd", w, tri).reshape(-1, 3)
    return float(np.max(sdf(np.vstack([S, pts]))))


def callus_patch(p, n, R=0.009, Hc=0.0015, nr=10, nt=40):
    """A raised patch on the skin: polar grid in the site's tangent plane projected onto the skin, lifted along the
    local normal by Hc (flat top, rounded edge over the outer 1.5 mm), with a skirt 0.5 mm into the skin."""
    a = np.cross(n, [0.0, 1.0, 0.0]); a /= np.linalg.norm(a)
    b = np.cross(n, a)
    rings = list(np.linspace(0, R - 0.0015, nr)) + [R - 0.001, R - 0.0005, R, R]
    lift = [Hc] * nr + [Hc * 0.85, Hc * 0.55, 0.0002, -0.0005]
    verts = []
    for r, L in zip(rings, lift):
        for k in range(nt if r > 0 else 1):
            th = 2 * math.pi * k / nt
            q = p + r * (math.cos(th) * a + math.sin(th) * b)
            s = ray_hit(q - 0.01 * n, n, 0.0, 0.03)
            verts.append(s + L * grad(s)[0])
    verts = np.array(verts)
    faces = []
    for k in range(nt):
        faces.append((0, 1 + k, 1 + (k + 1) % nt))
    for ring in range(len(rings) - 2):
        b0, b1 = 1 + ring * nt, 1 + (ring + 1) * nt
        for k in range(nt):
            k1 = (k + 1) % nt
            faces += [(b0 + k, b1 + k, b1 + k1), (b0 + k, b1 + k1, b0 + k1)]
    return verts, np.array(faces)


def main(out="foot_mesh.npz"):
    v, f = surface_mesh()
    vn = grad(v)
    S = sites()
    tiles = plantar_tiles()
    core = core_hulls()
    ex = [hull_excess(t, 60) for t in tiles]
    for i, e in enumerate(ex):     # a tile over a concave patch: lower it into the skin by its excess
        if e > 0.0002:
            tiles[i] = tiles[i] - A([0.0, 0.0, e + 0.0001])
    print(f"tiles lowered: {sum(e > 0.0002 for e in ex)}, by up to {max(ex)*1000:.2f} mm")
    bad_t = [i for i, t in enumerate(tiles) if hull_excess(t, 60) > 0.0003]
    bad_c = [i for i, c in enumerate(core) if hull_excess(c) > 0.0003]
    print(f"mesh {len(v)} verts {len(f)} faces; tiles {len(tiles)} (poke out: {len(bad_t)}), core {len(core)} "
          f"(poke out: {len(bad_c)})")
    for i in bad_c:
        print("  core", i, round(hull_excess(core[i]) * 1000, 2), "mm")
    tiles = np.array([t for i, t in enumerate(tiles) if i not in bad_t])
    core = [c for i, c in enumerate(core) if i not in bad_c]
    # tile top deviation from the skin: centre of each tile's top
    dev = [abs(float(sdf(t[:4].mean(0)))) for t in tiles]
    print(f"tile-top centre to skin: max {max(dev)*1000:.2f} mm, mean {np.mean(dev)*1000:.3f} mm")
    for nm, (p, n) in S.items():
        print(nm, np.round(p * 1000, 1), np.round(n, 3))
    cal = {nm: callus_patch(p, n) for nm, (p, n) in S.items()}
    lens = np.array([len(c) for c in core])
    np.savez_compressed(out, verts=v.astype(np.float32), faces=f.astype(np.int32), normals=vn.astype(np.float32),
                        tiles=tiles.astype(np.float32), core=np.vstack(core).astype(np.float32), core_len=lens,
                        site_names=np.array(list(S)), site_p=np.array([S[k][0] for k in S]),
                        site_n=np.array([S[k][1] for k in S]),
                        **{f"callus_v_{k}": cal[k][0].astype(np.float32) for k in S},
                        **{f"callus_f_{k}": cal[k][1].astype(np.int32) for k in S})
    print("wrote", out)


if __name__ == "__main__":
    main(*sys.argv[1:])
