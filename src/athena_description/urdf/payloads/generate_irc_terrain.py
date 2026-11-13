#!/usr/bin/env python3
"""
IRC Mars Terrain Generator — Athena Rover / Kratos Workspace
=============================================================
Generates a 36 × 36 m heightfield-based STL terrain tuned for
International Rover Challenge (IRC) difficulty: 7 distinct challenge
zones, smooth inter-zone blending, and a clear flat spawn plateau.

Coordinate conventions
----------------------
  World space    : Genesis places this mesh at TERRAIN_POS = (10, 4, 0).
  Terrain-local  : (world_xy - terrain_pos_xy).
  Rover spawn    : world (10, 0)  →  terrain-local (0, −4).
  Terrain grid   : terrain-local (−18, −18) … (+18, +18), 0.15 m/cell.

Challenge zones (terrain-local XY, metres)
------------------------------------------
  Z0  Flat spawn plateau      centre  ( 0,  −4)  r = 3.5 m  — always clear
  Z1  Crater field            centre  (−9,   1)  — 5 craters, largest r=4 m
  Z2  Rocky debris field      centre  ( 9,   1)  — jagged boulders & rubble
  Z3  Ramp ridge              centre  ( 0,   7)  — 25° slope, 1.4 m peak
  Z4  Stepped escarpment      centre  (−7,  12)  — 4 discrete 18 cm steps
  Z5  Canyon passage          centre  ( 7,  12)  — 1.5 m wide, 1 m walls
  Z6  Dune field              centre  ( 0, −13)  — smooth undulating dunes
"""

import os
import sys
import numpy as np
from scipy.ndimage import gaussian_filter

try:
    import trimesh
except ImportError:
    sys.exit("trimesh not found — activate genesis_env first.")

# ---------------------------------------------------------------------------
# Grid parameters
# ---------------------------------------------------------------------------
TERRAIN_SIZE = 36.0          # metres, square
RESOLUTION   = 0.15          # metres per grid cell  →  241 × 241 = 58 k verts
N = int(TERRAIN_SIZE / RESOLUTION) + 1

OUTPUT_PATH = os.path.join(os.path.dirname(__file__), "irc_mars_terrain.stl")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_grids():
    lin = np.linspace(-TERRAIN_SIZE / 2, TERRAIN_SIZE / 2, N)
    return np.meshgrid(lin, lin)          # X[row,col], Y[row,col]


def smoothstep(t):
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def radial_weight(X, Y, cx, cy, inner_r, fade_w):
    """1 inside inner_r, smooth drop to 0 over fade_w."""
    d = np.sqrt((X - cx)**2 + (Y - cy)**2)
    return 1.0 - smoothstep((d - inner_r) / max(fade_w, 1e-6))


def octave_noise(X, Y, octaves=6, base_sigma=4.0, persistence=0.5, seed=0):
    """
    Multi-octave band-limited noise via iterated Gaussian-blurred white noise.
    Returns values roughly in [−1, 1].
    """
    rng  = np.random.default_rng(seed)
    h    = np.zeros_like(X)
    amp  = 1.0
    wtot = 0.0
    for k in range(octaves):
        sigma = max(base_sigma / (2.0**k) / RESOLUTION, 0.5)
        raw   = rng.standard_normal((N, N))
        band  = gaussian_filter(raw, sigma=sigma)
        mx    = np.max(np.abs(band))
        if mx > 0:
            band /= mx
        h    += amp * band
        wtot += amp
        amp  *= persistence
    return h / wtot


def add_crater(h, X, Y, cx, cy, radius, depth, rim_ratio=0.18):
    """Parabolic bowl + Gaussian raised rim."""
    r   = np.sqrt((X - cx)**2 + (Y - cy)**2)
    # bowl: deepest at centre, rises to 0 at rim edge
    bowl = np.where(r < radius, depth * ((r / radius)**2 - 1.0), 0.0)
    # rim ring
    rim  = rim_ratio * depth * np.exp(-0.5 * ((r - radius) / (radius * 0.22))**2)
    h   += bowl + rim


# ---------------------------------------------------------------------------
# Zone builders
# ---------------------------------------------------------------------------

def zone_base_roughness(X, Y):
    """Ubiquitous Martian micro-roughness — applied everywhere."""
    return octave_noise(X, Y, octaves=7, base_sigma=5.0, persistence=0.42, seed=1) * 0.07


def zone_craters(X, Y):
    h = np.zeros_like(X)
    # (cx, cy, radius, depth)
    specs = [
        (-7,   3,  3.8,  0.95, 0.20),  # large main crater
        (-11,  -2, 2.2,  0.55, 0.18),  # medium
        (-5,   6,  1.4,  0.35, 0.16),  # small
        (-13,  4,  1.0,  0.28, 0.15),  # small far
        (-9,  -1,  0.65, 0.18, 0.14),  # micro
    ]
    for cx, cy, r, d, rim in specs:
        add_crater(h, X, Y, cx, cy, r, d, rim)
    # extra roughness inside the zone
    zone_w = radial_weight(X, Y, -9, 1, 8, 3)
    rough  = octave_noise(X, Y, octaves=4, base_sigma=1.2, persistence=0.55, seed=2)
    h += zone_w * rough * 0.06
    return h


def zone_rock_garden(X, Y):
    """Jagged boulder rubble: high-freq noise with hard upward clipping."""
    zone_w = radial_weight(X, Y, 9, 1, 7, 3)
    # coarse boulders
    coarse = octave_noise(X, Y, octaves=3, base_sigma=1.8, persistence=0.6, seed=3)
    # fine rubble
    fine   = octave_noise(X, Y, octaves=5, base_sigma=0.6, persistence=0.65, seed=4)
    rocks  = np.abs(coarse) * 0.4 + np.abs(fine) * 0.12
    # random discrete boulder peaks
    rng = np.random.default_rng(7)
    bx = rng.uniform(5, 13, 25)
    by = rng.uniform(-3, 5,  25)
    br = rng.uniform(0.3, 1.0, 25)
    bh = rng.uniform(0.1, 0.45, 25)
    for i in range(len(bx)):
        d = np.sqrt((X - bx[i])**2 + (Y - by[i])**2)
        rocks += bh[i] * np.exp(-0.5 * (d / br[i])**2)
    return zone_w * rocks


def zone_ramp_ridge(X, Y):
    """
    Gaussian ridge running along X at Y≈7, peak ≈1.4 m.
    Left flank (Y < 7): gradual ~15° approach.
    Right flank (Y > 7): steep ~30° face.
    """
    # asymmetric ridge: compress the far side
    dy_near = Y - 7.0
    sigma_near = np.where(dy_near < 0, 3.5, 1.8)   # wide approach, narrow drop
    ridge = 1.4 * np.exp(-0.5 * (dy_near / sigma_near)**2)
    # slight X-tilt so right end is a bit higher → more interesting
    tilt  = 0.07 * X * np.exp(-0.5 * ((Y - 7.0) / 4.0)**2)
    ridge = np.clip(ridge + tilt, 0, None)
    return ridge


def zone_steps(X, Y):
    """
    Four discrete 18 cm steps ascending toward positive Y,
    concentrated around (−7, 12).  Each step is ~1.8 m deep.
    """
    zone_w = radial_weight(X, Y, -7, 12, 6.5, 2.5)
    step_y  = Y - 8.5          # reference from step base
    steps   = (
        np.where(step_y > 0.0, 0.18, 0.0) +
        np.where(step_y > 1.8, 0.18, 0.0) +
        np.where(step_y > 3.6, 0.18, 0.0) +
        np.where(step_y > 5.4, 0.18, 0.0)
    )
    # blur risers slightly so they don't cause physics mesh artefacts
    steps_smooth = gaussian_filter(steps, sigma=4)
    return zone_w * steps_smooth


def zone_canyon(X, Y):
    """
    Narrow 1.5 m-wide passage along Y = 8…16, centred at X = 7.
    Walls rise ~1.1 m on each side.
    """
    half_w  = 0.75           # half channel width
    wall_w  = 2.5            # lateral extent of wall
    in_y    = (Y > 7.5) & (Y < 16.5)
    dist_cx = np.abs(X - 7.0)
    # inside channel → 0, outside → wall rising to 1.1 m
    t = np.clip((dist_cx - half_w) / wall_w, 0.0, 1.0)
    wall = np.where(in_y, 1.1 * smoothstep(t), 0.0)
    # small boulders scattered in the channel for extra difficulty
    rng = np.random.default_rng(13)
    bx = rng.uniform(6.3, 7.7, 10)
    by = rng.uniform(8,   16,  10)
    bh_vals = rng.uniform(0.05, 0.2, 10)
    for i in range(10):
        d = np.sqrt((X - bx[i])**2 + (Y - by[i])**2)
        wall += bh_vals[i] * np.exp(-0.5 * (d / 0.25)**2)
    return gaussian_filter(wall, sigma=6)


def zone_dunes(X, Y):
    """
    Smooth sinusoidal dunes, 2.5 m wavelength, ~0.35 m amplitude.
    Concentrated south of spawn.
    """
    zone_w = radial_weight(X, Y, 0, -13, 7, 4)
    dunes  = 0.35 * np.sin(2 * np.pi * Y / 2.5 + 0.4 * X) * \
                   np.cos(2 * np.pi * X / 7.0 + 0.2 * Y)
    return zone_w * dunes


# ---------------------------------------------------------------------------
# Master builder
# ---------------------------------------------------------------------------

def build_heightfield():
    X, Y = make_grids()
    h = np.zeros((N, N))

    print("  [1/8] Base Martian roughness …")
    h += zone_base_roughness(X, Y)

    print("  [2/8] Crater field …")
    h += zone_craters(X, Y)

    print("  [3/8] Rock garden …")
    h += zone_rock_garden(X, Y)

    print("  [4/8] Ramp ridge …")
    h += zone_ramp_ridge(X, Y)

    print("  [5/8] Stepped escarpment …")
    h += zone_steps(X, Y)

    print("  [6/8] Canyon passage …")
    h += zone_canyon(X, Y)

    print("  [7/8] Dune field …")
    h += zone_dunes(X, Y)

    print("  [8/8] Flatten spawn plateau & final smoothing …")
    # Smooth out edges of terrain boundary (avoid cliff walls at border)
    edge_margin = 2.5  # metres
    edge_dist   = np.minimum(
        np.minimum(X + TERRAIN_SIZE/2, TERRAIN_SIZE/2 - X),
        np.minimum(Y + TERRAIN_SIZE/2, TERRAIN_SIZE/2 - Y),
    )
    edge_taper  = smoothstep(edge_dist / edge_margin)
    h          *= edge_taper

    # Flatten the rover spawn plateau
    spawn_w = radial_weight(X, Y, 0, -4, 3.0, 2.0)
    h      -= h * spawn_w    # interpolate h → 0 inside spawn zone

    # Raise the entire terrain so minimum is at z = 0
    h -= h.min()

    return h


# ---------------------------------------------------------------------------
# Mesh construction
# ---------------------------------------------------------------------------

def heightfield_to_trimesh(h):
    xs = np.linspace(-TERRAIN_SIZE / 2, TERRAIN_SIZE / 2, N)
    ys = np.linspace(-TERRAIN_SIZE / 2, TERRAIN_SIZE / 2, N)

    # Vertices: row-major (Y outer, X inner)
    xx, yy = np.meshgrid(xs, ys)
    verts = np.column_stack([xx.ravel(), yy.ravel(), h.ravel()])

    # Faces: two triangles per quad
    r = np.arange(N - 1)
    c = np.arange(N - 1)
    rr, cc = np.meshgrid(r, c, indexing="ij")
    rr, cc = rr.ravel(), cc.ravel()

    v00 = rr * N + cc
    v10 = rr * N + (cc + 1)
    v01 = (rr + 1) * N + cc
    v11 = (rr + 1) * N + (cc + 1)

    faces = np.concatenate([
        np.column_stack([v00, v10, v11]),
        np.column_stack([v00, v11, v01]),
    ], axis=0)

    surface = trimesh.Trimesh(vertices=verts, faces=faces, process=False)

    # Watertight bottom cap (flat quad at z = floor − 0.5)
    z_bot = h.min() - 0.5
    cap_v = np.array([
        [xs[0],  ys[0],  z_bot],
        [xs[-1], ys[0],  z_bot],
        [xs[-1], ys[-1], z_bot],
        [xs[0],  ys[-1], z_bot],
    ])
    cap_f = np.array([[0, 2, 1], [0, 3, 2]])
    cap   = trimesh.Trimesh(vertices=cap_v, faces=cap_f, process=False)

    return trimesh.util.concatenate([surface, cap])


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print(f"\n{'='*60}")
    print("IRC Mars Terrain Generator")
    print(f"Grid: {N}×{N}  ({TERRAIN_SIZE}m × {TERRAIN_SIZE}m at {RESOLUTION}m/cell)")
    print(f"Output: {OUTPUT_PATH}")
    print(f"{'='*60}\n")

    print("Building heightfield …")
    h = build_heightfield()
    print(f"\nHeight statistics:")
    print(f"  min  = {h.min():.3f} m")
    print(f"  max  = {h.max():.3f} m")
    print(f"  mean = {h.mean():.3f} m")
    print(f"  std  = {h.std():.3f} m")

    print("\nTriangulating …")
    mesh = heightfield_to_trimesh(h)
    print(f"  Vertices : {len(mesh.vertices):,}")
    print(f"  Faces    : {len(mesh.faces):,}")

    print(f"\nSaving STL → {OUTPUT_PATH}")
    mesh.export(OUTPUT_PATH)
    size_mb = os.path.getsize(OUTPUT_PATH) / 1e6
    print(f"  File size: {size_mb:.1f} MB")
    print("\nDone ✓  — update TERRAIN_PATH in config.py to point at irc_mars_terrain.stl")
