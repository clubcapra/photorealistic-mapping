"""Regenerate outdoor_rocky.wbt with rocks sitting on a known elevation grid.

Why this exists: the previous outdoor_rocky.wbt used UnevenTerrain with 3 m of
Perlin relief — Webots computed the terrain procedurally at proto-load time, so
we couldn't query its heights to place rocks correctly. Rocks were placed at
hardcoded z=0.8-1.2 m and ended up buried inside hills, leaving the LiDAR
nothing to match. This script replaces the terrain with an explicit
ElevationGrid we generated here, so rock z = terrain_height(x,y) + base_lift
is exact.
"""
from __future__ import annotations

import math
import random
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter

SIZE_XY = 80.0          # m, terrain extent
GRID = 128              # 128x128 vertices
MAX_RELIEF = 1.2        # m, peak-to-valley target (so rocks stand out)
SEED = 42

OUT = Path(__file__).parent / "outdoor_rocky.wbt"


def fractal_heights(n: int, octaves: int, seed: int) -> np.ndarray:
    """Sum of gaussian-smoothed white noise at several scales."""
    rng = np.random.default_rng(seed)
    acc = np.zeros((n, n), dtype=np.float64)
    amp = 1.0
    for o in range(octaves):
        # smaller sigma => higher frequency
        sigma = n / (4.0 * (2 ** o))
        noise = rng.standard_normal((n, n))
        h = gaussian_filter(noise, sigma=sigma, mode="reflect")
        h /= h.std() if h.std() > 0 else 1.0
        acc += amp * h
        amp *= 0.55
    # normalize to roughly [0, MAX_RELIEF]
    acc -= acc.min()
    acc = acc / acc.max() * MAX_RELIEF
    return acc


def sample_height(heights: np.ndarray, x: float, y: float) -> float:
    """Bilinear sample of `heights` at world coords (x, y).

    Grid origin (i=0, j=0) corresponds to (-SIZE_XY/2, -SIZE_XY/2)."""
    n = heights.shape[0]
    step = SIZE_XY / (n - 1)
    fx = (x + SIZE_XY / 2) / step
    fy = (y + SIZE_XY / 2) / step
    ix = int(np.clip(np.floor(fx), 0, n - 2))
    iy = int(np.clip(np.floor(fy), 0, n - 2))
    tx = fx - ix
    ty = fy - iy
    h00 = heights[iy, ix]
    h10 = heights[iy, ix + 1]
    h01 = heights[iy + 1, ix]
    h11 = heights[iy + 1, ix + 1]
    return (
        (1 - tx) * (1 - ty) * h00
        + tx * (1 - ty) * h10
        + (1 - tx) * ty * h01
        + tx * ty * h11
    )


def elevation_grid_node(heights: np.ndarray) -> str:
    n = heights.shape[0]
    step = SIZE_XY / (n - 1)
    # Webots ElevationGrid lies in xz-plane (y vertical) by default. For our
    # ENU world we rotate -pi/2 around X so the grid is in xy with z up, and
    # shift it so the (0,0) grid vertex sits at world (-40, -40, 0).
    height_str = "\n        ".join(
        " ".join(f"{heights[j, i]:.4f}" for i in range(n))
        for j in range(n)
    )
    return (
        f"DEF terrain Solid {{\n"
        f"  translation {-SIZE_XY/2:.3f} {-SIZE_XY/2:.3f} 0\n"
        f"  children [\n"
        f"    Shape {{\n"
        f"      appearance PBRAppearance {{\n"
        f"        baseColor 0.5 0.45 0.35\n"
        f"        roughness 1.0\n"
        f"        metalness 0\n"
        f"      }}\n"
        f"      geometry DEF terrain_grid ElevationGrid {{\n"
        f"        xDimension {n}\n"
        f"        yDimension {n}\n"
        f"        xSpacing {step:.5f}\n"
        f"        ySpacing {step:.5f}\n"
        f"        height [\n"
        f"        {height_str}\n"
        f"        ]\n"
        f"      }}\n"
        f"    }}\n"
        f"  ]\n"
        f"  boundingObject USE terrain_grid\n"
        f"  locked TRUE\n"
        f"  name \"terrain\"\n"
        f"}}"
    )


def rock_node(name: str, x: float, y: float, z: float, scale: float,
              flat: bool = False) -> str:
    type_str = '  type "flat"' if flat else ""
    return (
        f'Rock {{ translation {x:.2f} {y:.2f} {z:.3f}  '
        f'scale {scale:.2f}{type_str}  name "{name}" }}'
    )


def trajectory_polyline() -> list[tuple[float, float]]:
    """Reproduce the outdoor_loop1 trajectory the robot will drive.
    Matches rove_sim_trajectory_driver's leg sequence on the rocky world."""
    x, y, yaw = 0.0, -4.0, 0.5  # Rove start (matches rove block below)
    pts = [(x, y)]
    legs = [(0.5, 0.0, 8.0), (0.0, 1.0, 1.57)] * 3 + [(0.5, 0.0, 8.0)]
    for v, w, t in legs:
        if w != 0:
            yaw += w * t
        else:
            x += v * t * math.cos(yaw)
            y += v * t * math.sin(yaw)
            pts.append((x, y))
    return pts


def point_to_segment_dist(px, py, ax, ay, bx, by):
    """Shortest distance from (px,py) to segment AB."""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy)


ROBOT_HALF_W_M = 0.45      # Rove is ~0.85 m wide
SAFETY_MARGIN_M = 0.6      # extra padding around the driven path


def rock_footprint_radius(scale: float, flat: bool) -> float:
    """Half-width of the rock's bounding box at the given scale.
    Values come from the Rock.proto mesh extents at scale=1."""
    return (0.22 if flat else 0.066) * scale


def trajectory_clearance(x: float, y: float, traj: list[tuple[float, float]],
                          scale: float, flat: bool) -> float:
    """Slack between the rock's edge and the closest trajectory segment.
    Negative means the rock overlaps the path's safety lane."""
    center_dist = min(point_to_segment_dist(x, y, *traj[i], *traj[i + 1])
                      for i in range(len(traj) - 1))
    return (center_dist - rock_footprint_radius(scale, flat)
            - ROBOT_HALF_W_M - SAFETY_MARGIN_M)


def too_close_to_trajectory(x: float, y: float, traj: list[tuple[float, float]],
                             scale: float = 1.0, flat: bool = False) -> bool:
    return trajectory_clearance(x, y, traj, scale, flat) < 0


def fit_scale(x: float, y: float, traj: list[tuple[float, float]],
               desired_scale: float, flat: bool) -> float | None:
    """Return the largest scale ≤ desired_scale that still clears the path,
    or None if even a tiny rock would block (means: drop the position)."""
    if not too_close_to_trajectory(x, y, traj, desired_scale, flat):
        return desired_scale
    # binary-search down
    lo, hi = 0.5, desired_scale
    for _ in range(20):
        mid = (lo + hi) / 2
        if too_close_to_trajectory(x, y, traj, mid, flat):
            hi = mid
        else:
            lo = mid
    return lo if lo >= 1.0 else None


def main() -> None:
    rng = random.Random(SEED)
    heights = fractal_heights(GRID, octaves=4, seed=SEED)
    traj = trajectory_polyline()

    # Rock.proto mesh has scale-1 bounding box:
    #   regular: z in [-0.056, +0.049]  (mesh centered on z=0)
    #   flat:    z in [0, +0.176]       (flat underside at z=0)
    # To place the base ON the terrain:
    #   regular: z = terrain + 0.056 * scale  (lift by half-extent)
    #   flat:    z = terrain                  (base already at 0)
    def rock_z(x: float, y: float, scale: float, flat: bool) -> float:
        base_offset = 0.0 if flat else 0.056 * scale
        return sample_height(heights, x, y) + base_offset

    rocks: list[str] = []

    # E-W rock wall along y=8 — big flat boulders (1-3 m tall after scaling).
    for i, x in enumerate(np.linspace(-15, 15, 11)):
        scale = rng.uniform(8.0, 16.0)
        y = 8.0 + rng.uniform(-0.3, 0.3)
        rocks.append(rock_node(f"rw1_{i+1}", x, y,
                                rock_z(x, y, scale, flat=True), scale, flat=True))
    # Second row right behind, mid-size regular rocks.
    for i, x in enumerate(np.linspace(-13, 13, 9)):
        scale = rng.uniform(4.0, 10.0)
        y = 8.6 + rng.uniform(-0.2, 0.2)
        rocks.append(rock_node(f"rw1_b{i+1}", x, y,
                                rock_z(x, y, scale, flat=False), scale))

    # N-S rock wall along x=-12.
    for i, y in enumerate(np.linspace(-14, 4, 8)):
        scale = rng.uniform(8.0, 15.0)
        x = -12.0 + rng.uniform(-0.3, 0.3)
        rocks.append(rock_node(f"rw2_{i+1}", x, y,
                                rock_z(x, y, scale, flat=True), scale, flat=True))
    for i, y in enumerate(np.linspace(-12, 2, 7)):
        scale = rng.uniform(4.0, 9.0)
        x = -12.6 + rng.uniform(-0.2, 0.2)
        rocks.append(rock_node(f"rw2_b{i+1}", x, y,
                                rock_z(x, y, scale, flat=False), scale))

    # Scattered rock field — wide size distribution from pebbles to landmarks.
    # Positions are filtered against the driven trajectory so the robot has a
    # clear lane. Original on-path picks (3.5,-2.2), (1.8,3.3), (4.5,2.0) are
    # nudged outward.
    scatter_positions = [
        (-4.0, -3.8), (6.1, -5.5), (-6.5, -2.0),
        (8.5, -5.0), (-9.0, -4.0), (2.0, -8.0), (-3.0, -8.5),
        (-2.5, 4.5), (-5.0, 1.5), (7.0, 2.5), (-7.5, 2.0),
        (10.0, -12.0), (14.0, -10.0),
        (0.0, -14.0), (5.0, -14.0), (-5.0, -14.0),
        (12.0, 4.0), (-15.0, -8.0), (15.0, 0.0), (-8.0, 6.0),
        (3.0, -11.0), (-11.0, 5.0),
        # Replacements for the on-path picks, pushed outward
        (6.5, -2.5), (-4.0, 5.5), (8.0, 3.0),
    ]
    rs_idx = 0
    for x, y in scatter_positions:
        r = rng.random()
        if r < 0.30:
            desired = rng.uniform(12.0, 22.0)
            flat = rng.random() < 0.5
        elif r < 0.55:
            desired = rng.uniform(2.0, 5.0)
            flat = False
        else:
            desired = rng.uniform(5.0, 12.0)
            flat = rng.random() < 0.4
        scale = fit_scale(x, y, traj, desired, flat)
        if scale is None:
            continue  # position too close to path even for a small rock
        rs_idx += 1
        rocks.append(rock_node(f"rs_{rs_idx}", x, y,
                                rock_z(x, y, scale, flat=flat), scale, flat=flat))

    # Oil barrels — sit on the surface
    barrels: list[str] = []
    barrel_positions = [
        ("bc_1a", 14.0, 14.0), ("bc_1b", 14.6, 14.0), ("bc_1c", 14.0, 14.6),
        ("bc_2a", -14.0, 14.0), ("bc_2b", -14.6, 14.0), ("bc_2c", -14.0, 14.6),
        ("bc_3a", 18.0, -16.0), ("bc_3b", 18.6, -16.0), ("bc_3c", 18.0, -16.6),
    ]
    for name, x, y in barrel_positions:
        z = sample_height(heights, x, y) + 0.6  # half a barrel height
        barrels.append(
            f'OilBarrel {{ translation {x:.2f} {y:.2f} {z:.3f}  name "{name}" }}'
        )

    # Trees — base at terrain surface
    trees: list[str] = []
    tree_specs = [
        ("Pine", "tree_pine_1",  16,  16),
        ("Pine", "tree_pine_2", -16,  16),
        ("Pine", "tree_pine_3",  16, -16),
        ("Pine", "tree_pine_4", -16, -16),
        ("Pine", "tree_pine_5",  24,   0),
        ("Pine", "tree_pine_6", -24,   0),
        ("Pine", "tree_pine_7",   0,  24),
        ("Oak",  "tree_oak_1",   20,  10),
        ("Oak",  "tree_oak_2",  -20,  10),
        ("Oak",  "tree_oak_3",   20, -10),
        ("Cypress", "tree_cyp_1",  10,  20),
        ("Cypress", "tree_cyp_2", -10,  20),
        ("Cypress", "tree_cyp_3",   0, -24),
        ("Cypress", "tree_cyp_4",  22,  22),
        ("Cypress", "tree_cyp_5", -22, -22),
    ]
    for species, name, x, y in tree_specs:
        z = sample_height(heights, x, y)
        trees.append(
            f'{species} {{ translation {x} {y} {z:.3f}  name "{name}" }}'
        )

    # Rove starts slightly above ground at world origin region
    rove_z = sample_height(heights, 0.0, -4.0) + 0.5
    rove = (
        f'Rove {{\n  translation 0 -4 {rove_z:.3f}\n  '
        f'rotation 0 0 1 0.5\n  name "rove"\n}}'
    )

    body = f"""#VRML_SIM R2023b utf8
# Outdoor world — rocky / quarry terrain (regenerated 2026-05-25).
# Replaces the original procedural UnevenTerrain (3 m relief, rocks buried).
# Terrain now lives in an explicit ElevationGrid we generated in Python, and
# every rock / barrel / tree is placed at terrain_height(x,y) + clearance so
# the LiDAR actually sees them.

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/rocks/protos/Rock.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/obstacles/protos/OilBarrel.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/trees/protos/Pine.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/trees/protos/Oak.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/trees/protos/Cypress.proto"
EXTERNPROTO "../protos/Rove.proto"

WorldInfo {{
  basicTimeStep 16
  coordinateSystem "ENU"
  contactProperties [
    ContactProperties {{ coulombFriction [ 1.4 ] softCFM 0.0001 }}
  ]
}}
Viewpoint {{
  orientation -0.3 0.3 0.9 1.5
  position 18 -14 12
  follow "rove"
}}
TexturedBackground {{ texture "noon_cloudy_countryside" }}
TexturedBackgroundLight {{ texture "noon_cloudy_countryside" }}

{elevation_grid_node(heights)}

# ============================================================
# Rock wall 1 (E-W ridge through y=8)
# ============================================================
{chr(10).join(r for r in rocks if r.split('"')[1].startswith('rw1'))}

# ============================================================
# Rock wall 2 (N-S ridge through x=-12)
# ============================================================
{chr(10).join(r for r in rocks if r.split('"')[1].startswith('rw2'))}

# ============================================================
# Scattered rock field — varied sizes (0.4 to 3.5 scale)
# ============================================================
{chr(10).join(r for r in rocks if r.split('"')[1].startswith('rs'))}

# Oil barrel clusters
{chr(10).join(barrels)}

# Long-range tree landmarks
{chr(10).join(trees)}

{rove}
"""
    OUT.write_text(body)
    print(f"wrote {OUT}  ({GRID}x{GRID} terrain, "
          f"{len(rocks)} rocks, {len(barrels)} barrels, {len(trees)} trees)")
    print(f"terrain height range: {heights.min():.2f} to {heights.max():.2f} m")
    print(f"rove start z: {rove_z:.2f}")


if __name__ == "__main__":
    main()
