"""Generate collision-free trajectories per world.

Parses a Webots .wbt file for the robot start pose and all obstacles,
then forward-simulates candidate cmd_vel sequences until it finds one
that keeps the robot's footprint clear of every obstacle at every step.

Usage:
    python3 -m rove_sim_webots.auto_trajectory <world.wbt> [--write traj.yaml]
    # or programmatically:
    traj = plan(Path('indoor_warehouse.wbt'))

The planner enumerates rectangular loops of varying size + orientation
centered on (or shifted from) the robot start. It picks the largest loop
that closes back near the start and clears every obstacle.

Obstacles are approximated as circles or axis-aligned bounding boxes from
the .wbt's translation/size/scale fields. The robot is approximated as a
0.45 m disk.
"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


ROBOT_RADIUS_M = 0.45
SAFETY_MARGIN_M = 1.10   # extra clearance on top of robot radius
# Bumped 0.80 -> 1.10 after stability sweep v2: warehouse + urban consistently
# returned soft_fail at ~0.31 m clearance (robot edges grazing). Closed-loop
# driver overshoots into pinch points at the 0.80 m margin.
ARENA_MARGIN_M = 0.50    # stay this far from arena walls


@dataclass
class CircleObstacle:
    x: float
    y: float
    r: float   # footprint radius

    def clearance(self, px: float, py: float) -> float:
        return math.hypot(px - self.x, py - self.y) - self.r


@dataclass
class BoxObstacle:
    cx: float
    cy: float
    dx: float
    dy: float

    def clearance(self, px: float, py: float) -> float:
        # signed distance from point to AABB (positive outside).
        rx = max(abs(px - self.cx) - self.dx / 2, 0.0)
        ry = max(abs(py - self.cy) - self.dy / 2, 0.0)
        outside = math.hypot(rx, ry)
        if outside > 0:
            return outside
        # inside the box — clearance is negative distance to nearest edge
        return -min(self.dx / 2 - abs(px - self.cx),
                    self.dy / 2 - abs(py - self.cy))


Obstacle = CircleObstacle | BoxObstacle


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------

_NUM = r"[+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?"

# Default footprint radius (m) for nodes whose dimensions we don't parse.
DEFAULT_FOOTPRINTS: dict[str, float] = {
    "OilBarrel": 0.30,
    "Pine": 0.60,
    "Oak": 0.70,
    "Cypress": 0.55,
    "PalmTree": 0.50,
    "BigSassafras": 0.85,
    "FireExtinguisher": 0.20,
    "FireHydrant": 0.20,
    "PublicBin": 0.30,
    "TrafficCone": 0.20,
    "Bench": 0.60,
    "SquareManhole": 0.30,
    "Chair": 0.35,
    "OfficeChair": 0.40,
    "RoundTable": 0.65,
    "Table": 0.65,
    "Desk": 0.80,
    "Cabinet": 0.50,
    "WoodenBox": 0.30,
    "PlasticCrate": 0.30,
    "PipeSection": 0.40,
    "LargeValve": 0.30,
}


def _find_translation(block: str) -> Optional[tuple[float, float, float]]:
    m = re.search(rf"translation\s+({_NUM})\s+({_NUM})\s+({_NUM})", block)
    return tuple(float(g) for g in m.groups()) if m else None  # type: ignore


def _find_size(block: str) -> Optional[tuple[float, float, float]]:
    m = re.search(rf"size\s+({_NUM})\s+({_NUM})\s+({_NUM})", block)
    return tuple(float(g) for g in m.groups()) if m else None  # type: ignore


def _find_rotation_yaw(block: str) -> Optional[float]:
    # rotation ax ay az theta — assume axis is +z for our worlds
    m = re.search(rf"rotation\s+{_NUM}\s+{_NUM}\s+{_NUM}\s+({_NUM})", block)
    return float(m.group(1)) if m else None


_HEADER_RE = re.compile(
    # Either "DEF <ALL_CAPS_NAME> <NodeType>" or just "<NodeType>".
    # Anchored so we don't accidentally eat trailing words from a comment.
    r"(?<![A-Za-z0-9_])"
    r"(DEF\s+[A-Z][A-Z0-9_]*\s+[A-Z][A-Za-z0-9_]*|[A-Z][A-Za-z0-9_]*)"
    r"\s*\{",
    re.MULTILINE,
)


def _iter_top_level_blocks(text: str):
    """Yield (header, body) for each top-level brace-balanced block. Skips
    content inside line comments (#) so words from comments can't be parsed
    as headers."""
    # Strip comments first so regex anchoring is reliable.
    stripped = re.sub(r"#[^\n]*", "", text)
    i = 0
    while i < len(stripped):
        m = _HEADER_RE.search(stripped, i)
        if not m:
            return
        header = m.group(1).strip()
        start = m.end()
        depth = 1
        j = start
        while j < len(stripped) and depth > 0:
            ch = stripped[j]
            if ch == '{':
                depth += 1
            elif ch == '}':
                depth -= 1
            j += 1
        yield header, stripped[start:j - 1]
        i = j


def _extract_obstacles_from_blocks(text: str,
                                     parent_offset: tuple[float, float] = (0.0, 0.0)
                                     ) -> list[Obstacle]:
    """Walk top-level blocks at this level; recurse into Group/Transform/Solid
    bodies so nested obstacles inside DEF blocks are caught. Translations are
    summed across containers so a Group with its own translation shifts its
    children correctly."""
    obstacles: list[Obstacle] = []
    px, py = parent_offset
    for header, body in _iter_top_level_blocks(text):
        node_type = header.split()[-1] if header.startswith('DEF ') else header.split()[0]
        t = _find_translation(body) or (0.0, 0.0, 0.0)
        x, y = px + t[0], py + t[1]

        # Containers: recurse into their bodies with translation accumulated.
        if node_type in ('Group', 'Transform'):
            obstacles.extend(_extract_obstacles_from_blocks(body, (x, y)))
            continue

        # Plain Solid that wraps a Cylinder geometry -> column-style obstacle.
        if node_type == 'Solid':
            mc = re.search(rf"Cylinder\s*\{{[^}}]*radius\s+({_NUM})", body)
            if mc:
                obstacles.append(CircleObstacle(x, y, float(mc.group(1))))
                continue
            # Solid wrapping more obstacles -> recurse.
            obstacles.extend(_extract_obstacles_from_blocks(body, (x, y)))
            continue

        ob = _obstacle_from_leaf(node_type, body, x, y)
        if ob is not None:
            obstacles.append(ob)
    return obstacles


def _obstacle_from_leaf(node_type: str, body: str, x: float, y: float
                         ) -> Optional[Obstacle]:
    if node_type == 'Wall':
        sz = _find_size(body)
        if sz:
            dx, dy, _ = sz
            return BoxObstacle(x, y, dx, dy)
        return CircleObstacle(x, y, 0.5)
    if node_type == 'CardboardBox':
        sz = _find_size(body)
        if sz:
            dx, dy, _ = sz
            return BoxObstacle(x, y, dx, dy)
        return CircleObstacle(x, y, 0.5)
    if node_type == 'SimpleBuilding':
        sz = _find_size(body)
        if sz:
            dx, dy, _ = sz
            return BoxObstacle(x, y, dx, dy)
        # SimpleBuilding default footprint is roughly 10m
        return CircleObstacle(x, y, 6.0)
    if node_type == 'Rock':
        ms = re.search(rf"scale\s+({_NUM})", body)
        scale = float(ms.group(1)) if ms else 1.0
        is_flat = 'type "flat"' in body
        return CircleObstacle(x, y, (0.22 if is_flat else 0.066) * scale)
    r = DEFAULT_FOOTPRINTS.get(node_type)
    if r is not None:
        return CircleObstacle(x, y, r)
    return None


def parse_obstacles(wbt_path: Path) -> list[Obstacle]:
    text = wbt_path.read_text()
    return _extract_obstacles_from_blocks(text)


def parse_rove_pose(wbt_path: Path) -> tuple[float, float, float]:
    text = wbt_path.read_text()
    # Find the Rove block specifically
    for header, body in _iter_top_level_blocks(text):
        if header.split()[0] == 'Rove':
            t = _find_translation(body)
            yaw = _find_rotation_yaw(body) or 0.0
            if t is None:
                raise ValueError(f"Rove block has no translation in {wbt_path}")
            return t[0], t[1], yaw
    raise ValueError(f"No Rove block in {wbt_path}")


def parse_arena_bounds(wbt_path: Path) -> Optional[tuple[float, float, float, float]]:
    """Return (xmin, xmax, ymin, ymax) of a RectangleArena floor, if present."""
    text = wbt_path.read_text()
    for header, body in _iter_top_level_blocks(text):
        if header.split()[0] != 'RectangleArena':
            continue
        m = re.search(rf"floorSize\s+({_NUM})\s+({_NUM})", body)
        if not m:
            continue
        sx, sy = float(m.group(1)), float(m.group(2))
        t = _find_translation(body) or (0.0, 0.0, 0.0)
        return (t[0] - sx / 2, t[0] + sx / 2, t[1] - sy / 2, t[1] + sy / 2)
    return None


# ---------------------------------------------------------------------------
# Simulation + collision check
# ---------------------------------------------------------------------------

Segment = dict  # {'dt': float, 'v': float, 'w': float}


def simulate_path(start: tuple[float, float, float],
                   segments: list[Segment],
                   step_s: float = 0.1) -> list[tuple[float, float]]:
    """Forward-integrate cmd_vel sequence with skid-steer slip applied so
    the simulated trajectory matches what the real robot actually executes."""
    x, y, yaw = start
    pts = [(x, y)]
    for seg in segments:
        v_cmd, w_cmd, dt = float(seg['v']), float(seg['w']), float(seg['dt'])
        v = v_cmd * SKID_DRIVE_SLIP
        w = w_cmd * SKID_TURN_SLIP
        n = max(1, int(dt / step_s))
        h = dt / n
        for _ in range(n):
            x += v * math.cos(yaw) * h
            y += v * math.sin(yaw) * h
            yaw += w * h
            pts.append((x, y))
    return pts


def path_clearance(path: list[tuple[float, float]],
                    obstacles: list[Obstacle]) -> tuple[float, int]:
    """Return (min_clearance_m, blocker_index) along the path."""
    worst = float('inf')
    worst_i = -1
    for i, (px, py) in enumerate(path):
        for obs in obstacles:
            c = obs.clearance(px, py)
            if c < worst:
                worst = c
                worst_i = i
    return worst, worst_i


def is_clear(path: list[tuple[float, float]],
              obstacles: list[Obstacle],
              arena: Optional[tuple[float, float, float, float]]) -> bool:
    min_c, _ = path_clearance(path, obstacles)
    if min_c < ROBOT_RADIUS_M + SAFETY_MARGIN_M:
        return False
    if arena is not None:
        xmin, xmax, ymin, ymax = arena
        for px, py in path:
            if not (xmin + ARENA_MARGIN_M <= px <= xmax - ARENA_MARGIN_M and
                    ymin + ARENA_MARGIN_M <= py <= ymax - ARENA_MARGIN_M):
                return False
    return True


# ---------------------------------------------------------------------------
# Trajectory generation
# ---------------------------------------------------------------------------

# Skid-steer slip: open-loop commanded rotation under-shoots by ~25%.
# Generation compensates via SKID_TURN_GAIN, simulation models the slip via
# SKID_TURN_SLIP so commanded -> actual matches reality.
SKID_TURN_GAIN = 1.33
SKID_TURN_SLIP = 1.0 / SKID_TURN_GAIN
SKID_DRIVE_SLIP = 1.0  # forward motion: assume negligible slip


def _turn_dt(angle_rad: float, w: float) -> float:
    return abs(angle_rad) / abs(w) * SKID_TURN_GAIN


def _rect_loop(turn_first: bool,
                turn_dir: int,
                leg_lengths_m: list[float],
                v: float = 0.4,
                w: float = 0.5) -> list[Segment]:
    """Right-angle loop. turn_first=True rotates 90 deg before driving;
    turn_dir = +1 left, -1 right. leg_lengths_m has 4 entries (or 2 repeated)."""
    segs: list[Segment] = []
    if turn_first:
        segs.append({'dt': _turn_dt(math.pi / 2, w), 'v': 0.0, 'w': turn_dir * w})
    for i, L in enumerate(leg_lengths_m):
        segs.append({'dt': L / v, 'v': v, 'w': 0.0})
        if i < len(leg_lengths_m) - 1:
            segs.append({'dt': _turn_dt(math.pi / 2, w), 'v': 0.0, 'w': turn_dir * w})
    segs.append({'dt': 2.0, 'v': 0.0, 'w': 0.0})  # settle
    return segs


def _with_lead_in(lead_m: float, segs: list[Segment],
                    v: float = 0.4) -> list[Segment]:
    """Prepend a straight 'go this far before looping' segment, then loop,
    then drive back to (roughly) start."""
    if lead_m <= 0:
        return segs
    lead = {'dt': lead_m / v, 'v': v, 'w': 0.0}
    # After the loop closes near where it started, undo the lead-in.
    turn_180 = {'dt': math.pi / 1.0, 'v': 0.0, 'w': 1.0}
    return [lead, *segs, turn_180, lead, {'dt': 2.0, 'v': 0.0, 'w': 0.0}]


def generate_candidates(start: tuple[float, float, float]
                         ) -> list[tuple[str, list[Segment]]]:
    """Build a list of candidate trajectories ranked by total path length."""
    candidates: list[tuple[str, list[Segment]]] = []
    # Square loops (return to start). Include small sizes for tight spaces.
    sizes = (1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0)
    for L in sizes:
        for turn_dir in (+1, -1):
            for turn_first in (False, True):
                tag = f"square{L:.1f}_{'L' if turn_dir > 0 else 'R'}{'T' if turn_first else ''}"
                segs = _rect_loop(turn_first, turn_dir, [L, L, L, L])
                candidates.append((tag, segs))
    # Rectangular 2:1 loops.
    for sx, sy in ((3.0, 1.5), (4.0, 2.0), (6.0, 3.0), (8.0, 4.0), (10.0, 5.0)):
        for turn_dir in (+1, -1):
            for turn_first in (False, True):
                tag = f"rect{sx:.0f}x{sy:.0f}_{'L' if turn_dir > 0 else 'R'}{'T' if turn_first else ''}"
                segs = _rect_loop(turn_first, turn_dir, [sx, sy, sx, sy])
                candidates.append((tag, segs))
    # Simple back-and-forth wiggle, slower angular vel to avoid overshoot.
    # (Defined before lead-in so the wiggle constant is in scope.)
    # Lead-in variants — drive into open space first, then loop, then come back.
    for lead in (1.5, 3.0, 5.0):
        for L in (2.0, 3.0, 4.0):
            for turn_dir in (+1, -1):
                tag = f"lead{lead:.0f}+sq{L:.0f}_{'L' if turn_dir > 0 else 'R'}"
                core = _rect_loop(False, turn_dir, [L, L, L, L])
                segs = _with_lead_in(lead, core)
                candidates.append((tag, segs))
    # Simple back-and-forth wiggle as a fallback for the tightest spaces.
    for L in (0.6, 1.0, 1.5, 2.0, 3.0, 5.0):
        segs = [
            {'dt': L / 0.3, 'v': 0.3, 'w': 0.0},
            {'dt': math.pi / 1.0, 'v': 0.0, 'w': 1.0},
            {'dt': L / 0.3, 'v': 0.3, 'w': 0.0},
            {'dt': 2.0, 'v': 0.0, 'w': 0.0},
        ]
        candidates.append((f"wiggle{L:.1f}", segs))
    # In-place rotation — last resort. Stays put but at least lidar scans rotate.
    candidates.append(('spin_in_place', [
        {'dt': math.tau, 'v': 0.0, 'w': 1.0},
        {'dt': 2.0, 'v': 0.0, 'w': 0.0},
    ]))
    # Sort largest first so we prefer richer SLAM signal.
    def total_distance(segs):
        return sum(seg['v'] * seg['dt'] for seg in segs)
    candidates.sort(key=lambda c: -total_distance(c[1]))
    return candidates


def trajectory_to_yaml(name: str, description: str,
                        segments: list[Segment]) -> str:
    lines = [f"name: {name}", f"description: |", f"  {description}", "segments:"]
    for seg in segments:
        lines.append(
            f"  - {{ dt: {seg['dt']:.3f}, v: {seg['v']:.2f}, w: {seg['w']:+.2f} }}"
        )
    return "\n".join(lines) + "\n"


def waypoint_yaml(name: str, description: str,
                    waypoints: list[tuple[float, float]],
                    max_v: float = 0.4, max_w: float = 0.5) -> str:
    wp_lines = '\n'.join(
        f"  - [{x:.3f}, {y:.3f}]" for (x, y) in waypoints
    )
    return (
        f"name: {name}\n"
        f"type: waypoints\n"
        f"description: |\n  {description}\n"
        f"max_v: {max_v}\n"
        f"max_w: {max_w}\n"
        f"pos_tol_m: 0.30\n"
        f"yaw_tol_rad: 0.10\n"
        f"stuck_timeout_s: 6.0\n"
        f"waypoints:\n{wp_lines}\n"
    )


# ---------------------------------------------------------------------------
# Top-level planner
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Waypoint-based planner (primary): explores the world, weaves between
# obstacles. Falls back to rectangular loops if no waypoint tour fits.
# ---------------------------------------------------------------------------

GRID_CELL_M = 0.20


def build_occupancy(arena: tuple[float, float, float, float],
                     obstacles: list[Obstacle],
                     inflate_m: float) -> tuple:
    """Build a boolean occupancy grid over the arena.
    Returns (grid, x0, y0, cell) — grid[iy, ix] True means occupied."""
    import numpy as np
    xmin, xmax, ymin, ymax = arena
    nx = max(1, int(round((xmax - xmin) / GRID_CELL_M)))
    ny = max(1, int(round((ymax - ymin) / GRID_CELL_M)))
    grid = np.zeros((ny, nx), dtype=bool)
    for iy in range(ny):
        for ix in range(nx):
            x = xmin + (ix + 0.5) * GRID_CELL_M
            y = ymin + (iy + 0.5) * GRID_CELL_M
            for o in obstacles:
                if o.clearance(x, y) < inflate_m:
                    grid[iy, ix] = True
                    break
    return grid, xmin, ymin, GRID_CELL_M


def _world_to_grid(x: float, y: float, x0: float, y0: float, cell: float
                    ) -> tuple[int, int]:
    return int((x - x0) / cell), int((y - y0) / cell)


def line_of_sight(grid, x0: float, y0: float, cell: float,
                   ax: float, ay: float, bx: float, by: float) -> bool:
    """Bresenham-style march from A to B; True if every cell along the line
    is free in `grid`."""
    ny, nx = grid.shape
    iax, iay = _world_to_grid(ax, ay, x0, y0, cell)
    ibx, iby = _world_to_grid(bx, by, x0, y0, cell)
    dx = abs(ibx - iax)
    dy = abs(iby - iay)
    sx = 1 if ibx > iax else -1
    sy = 1 if iby > iay else -1
    err = dx - dy
    x_i, y_i = iax, iay
    while True:
        if not (0 <= x_i < nx and 0 <= y_i < ny):
            return False
        if grid[y_i, x_i]:
            return False
        if x_i == ibx and y_i == iby:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x_i += sx
        if e2 < dx:
            err += dx
            y_i += sy


def flood_fill_reachable(grid, ix0: int, iy0: int):
    """Return the set of (ix, iy) cells reachable from (ix0, iy0) (8-conn)."""
    ny, nx = grid.shape
    if not (0 <= ix0 < nx and 0 <= iy0 < ny) or grid[iy0, ix0]:
        return set()
    seen = {(ix0, iy0)}
    stack = [(ix0, iy0)]
    while stack:
        ix, iy = stack.pop()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nxx, nyy = ix + dx, iy + dy
                if (nxx, nyy) in seen:
                    continue
                if not (0 <= nxx < nx and 0 <= nyy < ny):
                    continue
                if grid[nyy, nxx]:
                    continue
                seen.add((nxx, nyy))
                stack.append((nxx, nyy))
    return seen


def sample_waypoints(arena: tuple[float, float, float, float],
                      obstacles: list[Obstacle],
                      start: tuple[float, float],
                      n_target: int,
                      min_clearance: float,
                      seed: int = 0,
                      reachable_cells=None,
                      grid_x0: float = 0.0, grid_y0: float = 0.0,
                      grid_cell: float = GRID_CELL_M,
                      ) -> list[tuple[float, float]]:
    """Random-sample points that have decent clearance and are well-spread.
    Returns at most n_target waypoints; start is NOT included in the list."""
    import random
    rng = random.Random(seed)
    xmin, xmax, ymin, ymax = arena
    margin = ARENA_MARGIN_M
    pool: list[tuple[float, float, float]] = []  # (clearance, x, y)
    # Generous oversample so we can filter for spacing.
    for _ in range(4000):
        x = rng.uniform(xmin + margin, xmax - margin)
        y = rng.uniform(ymin + margin, ymax - margin)
        if reachable_cells is not None:
            ix = int((x - grid_x0) / grid_cell)
            iy = int((y - grid_y0) / grid_cell)
            if (ix, iy) not in reachable_cells:
                continue
        c = min((o.clearance(x, y) for o in obstacles), default=float('inf'))
        if c >= min_clearance:
            pool.append((c, x, y))
    if not pool:
        return []
    # Sort by clearance descending so we prefer wide-open spots.
    pool.sort(reverse=True)
    picked: list[tuple[float, float]] = []
    min_spacing = 2.5  # m — keep waypoints separated for SLAM diversity
    for c, x, y in pool:
        if all(math.hypot(x - px, y - py) >= min_spacing for px, py in picked):
            if math.hypot(x - start[0], y - start[1]) >= min_spacing:
                picked.append((x, y))
        if len(picked) >= n_target:
            break
    return picked


def bfs_path(grid, x0: float, y0: float, cell: float,
              start: tuple[float, float], goal: tuple[float, float]
              ) -> Optional[list[tuple[float, float]]]:
    """BFS through the free grid; return a path of world coords from start
    to goal, or None if unreachable."""
    from collections import deque
    ny, nx = grid.shape
    sx, sy = int((start[0] - x0) / cell), int((start[1] - y0) / cell)
    gx, gy = int((goal[0] - x0) / cell), int((goal[1] - y0) / cell)
    if not (0 <= sx < nx and 0 <= sy < ny and 0 <= gx < nx and 0 <= gy < ny):
        return None
    if grid[sy, sx] or grid[gy, gx]:
        return None
    parent = {(sx, sy): None}
    q = deque([(sx, sy)])
    while q:
        cx, cy = q.popleft()
        if (cx, cy) == (gx, gy):
            break
        for ddx, ddy in ((-1, 0), (1, 0), (0, -1), (0, 1),
                          (-1, -1), (-1, 1), (1, -1), (1, 1)):
            nxx, nyy = cx + ddx, cy + ddy
            if not (0 <= nxx < nx and 0 <= nyy < ny):
                continue
            if grid[nyy, nxx]:
                continue
            if (nxx, nyy) in parent:
                continue
            parent[(nxx, nyy)] = (cx, cy)
            q.append((nxx, nyy))
    if (gx, gy) not in parent:
        return None
    # Reconstruct.
    cells = []
    cur = (gx, gy)
    while cur is not None:
        cells.append(cur)
        cur = parent[cur]
    cells.reverse()
    return [(x0 + (ix + 0.5) * cell, y0 + (iy + 0.5) * cell) for ix, iy in cells]


def simplify_path(path: list[tuple[float, float]],
                    los_fn) -> list[tuple[float, float]]:
    """Greedy line-of-sight simplification — collapse runs of waypoints whose
    endpoints are mutually visible into single straight segments."""
    if len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        # find the furthest j such that LOS from path[i] to path[j].
        j = len(path) - 1
        while j > i + 1:
            if los_fn(*path[i], *path[j]):
                break
            j -= 1
        out.append(path[j])
        i = j
    return out


def nearest_neighbor_tour(start: tuple[float, float],
                            waypoints: list[tuple[float, float]],
                            grid, x0: float, y0: float, cell: float,
                            ) -> list[tuple[float, float]]:
    """Greedy tour starting at start, visiting each waypoint reachable via
    BFS on the occupancy grid, returning to start. Inserts intermediate
    points from BFS paths (after LOS-simplification) so the resulting tour
    is a sequence of straight-line segments that all clear obstacles."""
    def los(ax, ay, bx, by):
        return line_of_sight(grid, x0, y0, cell, ax, ay, bx, by)

    visited = [start]
    remaining = list(waypoints)
    cur = start
    while remaining:
        # Find nearest reachable waypoint via BFS distance proxy = euclidean.
        # Actual reachability checked by running BFS.
        candidates = sorted(remaining, key=lambda wp: math.hypot(wp[0] - cur[0], wp[1] - cur[1]))
        chosen = None
        chosen_path = None
        for wp in candidates:
            path = bfs_path(grid, x0, y0, cell, cur, wp)
            if path is not None:
                chosen = wp
                chosen_path = simplify_path(path, los)
                break
        if chosen is None:
            break
        # Append all but the first point (which equals `cur`).
        visited.extend(chosen_path[1:])
        remaining.remove(chosen)
        cur = chosen
    # Close the loop back to start (via BFS if needed).
    if cur != start:
        back = bfs_path(grid, x0, y0, cell, cur, start)
        if back is not None:
            visited.extend(simplify_path(back, los)[1:])
    return visited


def tour_to_segments(start_yaw: float,
                      tour: list[tuple[float, float]],
                      v: float = 0.4,
                      w: float = 0.5) -> list[Segment]:
    """Convert a waypoint tour into (v, w, dt) cmd_vel segments.
    Robot rotates to face each next waypoint then drives straight."""
    segs: list[Segment] = []
    yaw = start_yaw
    for i in range(1, len(tour)):
        cx, cy = tour[i - 1]
        nx, ny = tour[i]
        dx, dy = nx - cx, ny - cy
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            continue
        target_yaw = math.atan2(dy, dx)
        # smallest signed turn
        dyaw = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(dyaw) > 0.03:
            segs.append({'dt': _turn_dt(abs(dyaw), w),
                         'v': 0.0,
                         'w': (1.0 if dyaw > 0 else -1.0) * w})
        segs.append({'dt': dist / v, 'v': v, 'w': 0.0})
        yaw = target_yaw
    segs.append({'dt': 2.0, 'v': 0.0, 'w': 0.0})  # settle
    return segs


@dataclass
class PlanResult:
    world: str
    name: str
    segments: list[Segment]
    path_length_m: float
    min_clearance_m: float
    n_waypoints: int = 0
    waypoints: list[tuple[float, float]] = field(default_factory=list)


def plan_waypoint_tour(wbt_path: Path,
                        n_waypoints: int = 6,
                        seed: int = 0
                        ) -> Optional[PlanResult]:
    start_pose = parse_rove_pose(wbt_path)
    obstacles = parse_obstacles(wbt_path)
    arena = parse_arena_bounds(wbt_path)
    if arena is None:
        # Default arena box for outdoor worlds.
        arena = (-20.0, 20.0, -20.0, 20.0)

    inflate = ROBOT_RADIUS_M + SAFETY_MARGIN_M
    grid, x0, y0, cell = build_occupancy(arena, obstacles, inflate)

    def los(ax, ay, bx, by) -> bool:
        return line_of_sight(grid, x0, y0, cell, ax, ay, bx, by)

    start = (start_pose[0], start_pose[1])
    yaw = start_pose[2]

    # Verify start itself is clear; if not, bail and let caller move it.
    if min((o.clearance(*start) for o in obstacles), default=float('inf')) < inflate:
        return None

    # Restrict waypoint sampling to the flood-fill region containing start —
    # avoids picking waypoints in disconnected rooms / behind walls.
    ix0 = int((start[0] - x0) / cell)
    iy0 = int((start[1] - y0) / cell)
    reachable = flood_fill_reachable(grid, ix0, iy0)
    if len(reachable) < 4:
        return None  # robot is too boxed in

    waypoints = sample_waypoints(arena, obstacles, start,
                                  n_target=n_waypoints,
                                  min_clearance=inflate,
                                  seed=seed,
                                  reachable_cells=reachable,
                                  grid_x0=x0, grid_y0=y0, grid_cell=cell)
    if not waypoints:
        return None

    tour = nearest_neighbor_tour(start, waypoints, grid, x0, y0, cell)
    if len(tour) < 3:
        return None

    segs = tour_to_segments(yaw, tour)
    # Cross-verify the integrated path against obstacle clearance.
    path = simulate_path(start_pose, segs)
    if not is_clear(path, obstacles, arena):
        return None
    min_c, _ = path_clearance(path, obstacles)
    path_len = sum(seg['v'] * seg['dt'] for seg in segs)
    return PlanResult(
        world=wbt_path.name,
        name=f"auto_{wbt_path.stem}_tour{len(tour) - 1}",
        segments=segs, path_length_m=path_len, min_clearance_m=min_c,
        n_waypoints=len(tour) - 2,  # exclude start and the close-the-loop entry
        waypoints=tour[1:],  # drop start; driver navigates to each from current pose
    )


def _tiny_inplace_loop(start: tuple[float, float, float],
                        obstacles: list[Obstacle]
                        ) -> Optional[list[tuple[float, float]]]:
    """Pick the largest 4-point box centered on start that clears obstacles.
    Always emit something so the verifier gets a non-trivial recording window."""
    sx, sy = start[0], start[1]
    for r in (1.0, 0.6, 0.4, 0.25):
        candidates = [
            [(sx + r, sy), (sx + r, sy + r), (sx, sy + r), (sx, sy)],
            [(sx - r, sy), (sx - r, sy + r), (sx, sy + r), (sx, sy)],
            [(sx + r, sy), (sx + r, sy - r), (sx, sy - r), (sx, sy)],
            [(sx - r, sy), (sx - r, sy - r), (sx, sy - r), (sx, sy)],
        ]
        for wps in candidates:
            # Each waypoint must clear all obstacles by at least robot
            # footprint (no safety margin here — tight space worlds).
            if all(min(o.clearance(*wp) for o in obstacles) >= ROBOT_RADIUS_M
                   for wp in wps):
                return wps
    return None


def plan(wbt_path: Path) -> Optional[PlanResult]:
    # Try the waypoint tour first with a few seeds; fall back to a tiny
    # in-place loop if no full tour fits.
    for seed in range(8):
        for n in (8, 6, 5, 4, 3):
            result = plan_waypoint_tour(wbt_path, n_waypoints=n, seed=seed)
            if result is not None:
                return result

    # Fallback A: a tiny in-place loop (still waypoint-format so the
    # closed-loop driver runs and the recorder gets a meaningful window).
    start_pose = parse_rove_pose(wbt_path)
    obstacles = parse_obstacles(wbt_path)
    tiny = _tiny_inplace_loop(start_pose, obstacles)
    if tiny is not None:
        # crude planned-length estimate: perimeter of the box
        path_len = sum(
            math.hypot(tiny[i + 1][0] - tiny[i][0], tiny[i + 1][1] - tiny[i][1])
            for i in range(len(tiny) - 1)
        )
        min_c = min(min(o.clearance(*wp) for o in obstacles) for wp in tiny)
        return PlanResult(
            world=wbt_path.name,
            name=f"auto_{wbt_path.stem}_tinyloop",
            segments=tour_to_segments(start_pose[2], [(start_pose[0], start_pose[1])] + tiny),
            path_length_m=path_len, min_clearance_m=min_c,
            waypoints=tiny,
        )

    # Fallback B: rectangular loops (legacy open-loop format).
    arena = parse_arena_bounds(wbt_path)
    candidates = generate_candidates(start_pose)
    for tag, segs in candidates:
        path = simulate_path(start_pose, segs)
        if not is_clear(path, obstacles, arena):
            continue
        min_c, _ = path_clearance(path, obstacles)
        path_len = sum(seg['v'] * seg['dt'] for seg in segs)
        return PlanResult(
            world=wbt_path.name, name=f"auto_{wbt_path.stem}_{tag}",
            segments=segs, path_length_m=path_len, min_clearance_m=min_c,
        )
    return None


def _summary(wbt_path: Path) -> str:
    start = parse_rove_pose(wbt_path)
    obs = parse_obstacles(wbt_path)
    arena = parse_arena_bounds(wbt_path)
    msg = [f"{wbt_path.name}: start=({start[0]:.2f},{start[1]:.2f},yaw={start[2]:.2f})  "
           f"{len(obs)} obstacles"]
    if arena is not None:
        msg.append(f"  arena x[{arena[0]:.1f},{arena[1]:.1f}] y[{arena[2]:.1f},{arena[3]:.1f}]")
    return "\n".join(msg)


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('world', type=Path)
    p.add_argument('--write', type=Path, default=None,
                    help="Write the resulting trajectory yaml to this path.")
    p.add_argument('--summary-only', action='store_true')
    args = p.parse_args()

    if not args.world.exists():
        # try resolving relative to the worlds dir
        share = Path(__file__).resolve().parent.parent / 'worlds' / args.world.name
        if share.exists():
            args.world = share
        else:
            raise SystemExit(f"world not found: {args.world}")

    print(_summary(args.world))
    if args.summary_only:
        return

    result = plan(args.world)
    if result is None:
        print("  NO CLEAR TRAJECTORY FOUND")
        raise SystemExit(2)

    print(f"  picked: {result.name}  "
          f"length={result.path_length_m:.1f} m  "
          f"min_clearance={result.min_clearance_m:.2f} m")

    desc = (f"Auto-generated for {result.world}. path={result.path_length_m:.1f} m, "
            f"min_clearance={result.min_clearance_m:.2f} m.")
    if result.waypoints:
        yaml_text = waypoint_yaml(result.name, desc, result.waypoints)
    else:
        yaml_text = trajectory_to_yaml(result.name, desc, result.segments)
    if args.write:
        args.write.write_text(yaml_text)
        print(f"  wrote {args.write}")
    else:
        print()
        print(yaml_text)


if __name__ == '__main__':
    main()
