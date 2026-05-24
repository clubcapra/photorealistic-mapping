"""Trajectory loading and segment iteration for sim runs.

Trajectory YAML format (see config/trajectories/*.yaml):

    name: outdoor_loop1
    description: 10m square loop with a return for loop closure
    segments:
      - { dt: 8.0, v: 0.5, w: 0.0 }
      - { dt: 1.57, v: 0.0, w: 1.0 }
      ...

Each segment runs for `dt` simulated seconds at constant linear velocity `v`
(m/s along the robot's +X) and angular velocity `w` (rad/s about +Z).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

import yaml


@dataclass
class Segment:
    dt: float
    v: float
    w: float


@dataclass
class Trajectory:
    name: str
    description: str
    segments: List[Segment]

    @property
    def duration(self) -> float:
        return sum(s.dt for s in self.segments)

    @classmethod
    def load(cls, path: Path) -> 'Trajectory':
        with open(path) as f:
            data = yaml.safe_load(f)
        segs = [
            Segment(
                dt=float(s['dt']),
                v=float(s.get('v', 0.0)),
                w=float(s.get('w', 0.0)),
            )
            for s in data.get('segments', [])
        ]
        return cls(
            name=data.get('name', path.stem),
            description=data.get('description', ''),
            segments=segs,
        )
