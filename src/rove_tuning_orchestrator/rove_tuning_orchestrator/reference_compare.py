"""Compare a candidate RTAB-Map .db against a reference .db.

The reference is a cleaned-up, mostly-accurate database built from the same
room as the candidate run. Two metrics are produced:

    correspondence_ratio   How many candidate nodes have a spatially-close
                           reference node within `tau_m`. Higher = better
                           topological consistency with the reference.

    tracking_loss_events   Number of nodes in the candidate db with a null
                           pose or a transform marked as lost. Lower = better.

The implementation reads the RTAB-Map SQLite schema directly. The schema is
stable across RTAB-Map 0.20+; if it changes, the queries in this file are
the only thing that breaks.

CLI:
    compare_to_reference --candidate run/rtabmap.db --reference clean.db
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import struct
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np


# RTAB-Map stores poses as a packed 12-float row-major 3x4 matrix (the last
# row [0 0 0 1] is implicit). See rtabmap/corelib/src/DBDriverSqlite3.cpp.
# Older versions used 16 floats (full 4x4). We support both.
def _unpack_pose(blob: bytes) -> Optional[np.ndarray]:
    if blob is None:
        return None
    n = len(blob) // 4
    if n == 12:
        m = struct.unpack('12f', blob)
        T = np.eye(4)
        T[:3, :4] = np.array(m).reshape(3, 4)
        return T
    if n == 16:
        m = struct.unpack('16f', blob)
        return np.array(m).reshape(4, 4)
    return None


@dataclass
class _Node:
    id: int
    stamp: float
    pose: Optional[np.ndarray]
    label: str = ''
    # True if pose is missing OR effectively identity-from-init OR flagged lost.
    is_lost: bool = False


def _read_nodes(db_path: Path) -> List[_Node]:
    """Read the Node table from an RTAB-Map .db file."""
    con = sqlite3.connect(f'file:{db_path}?mode=ro', uri=True)
    cur = con.cursor()
    # Some versions have 'time_enter' instead of 'stamp'. Try both.
    cols = [r[1] for r in cur.execute('PRAGMA table_info(Node)').fetchall()]
    stamp_col = 'stamp' if 'stamp' in cols else (
        'time_enter' if 'time_enter' in cols else None
    )
    pose_col = 'pose' if 'pose' in cols else 'ground_truth_pose'
    label_col = 'label' if 'label' in cols else None
    if stamp_col is None:
        raise RuntimeError(
            f'Cannot find a timestamp column in Node table of {db_path} '
            f'(saw {cols}). Schema may have changed; update reference_compare.py.'
        )
    select_cols = ['id', stamp_col, pose_col]
    if label_col:
        select_cols.append(label_col)
    rows = cur.execute(f'SELECT {", ".join(select_cols)} FROM Node').fetchall()
    con.close()

    out: List[_Node] = []
    for row in rows:
        if label_col:
            nid, stamp, pose_blob, label = row
        else:
            nid, stamp, pose_blob = row
            label = ''
        T = _unpack_pose(pose_blob) if pose_blob else None
        is_lost = (T is None) or (
            T is not None
            and np.allclose(T[:3, :4], np.eye(3, 4), atol=1e-6)
            and nid != 1
        )
        out.append(_Node(
            id=int(nid),
            stamp=float(stamp),
            pose=T,
            label=label or '',
            is_lost=is_lost,
        ))
    return out


@dataclass
class ReferenceCompareResult:
    candidate_db: str = ''
    reference_db: str = ''
    n_candidate_nodes: int = 0
    n_reference_nodes: int = 0
    tau_m: float = 0.5
    # Phase-2 headline metrics.
    correspondence_ratio: float = 0.0
    tracking_loss_events: int = 0
    tracking_loss_ratio: float = 0.0
    # Diagnostic detail.
    mean_nn_distance_m: float = 0.0
    median_nn_distance_m: float = 0.0
    max_nn_distance_m: float = 0.0
    candidate_with_pose: int = 0
    reference_with_pose: int = 0
    alignment_yaw_rad: float = 0.0  # yaw used to align candidate frame to ref
    warnings: List[str] = field(default_factory=list)


def compare(
    candidate_db: Path,
    reference_db: Path,
    tau_m: float = 0.5,
) -> ReferenceCompareResult:
    """Compute correspondence ratio + tracking-loss events.

    correspondence_ratio = fraction of candidate nodes whose nearest reference
                           node (by candidate-pose XYZ) is within tau_m.
    tracking_loss_events = candidate nodes flagged is_lost.
    """
    cand = _read_nodes(candidate_db)
    ref = _read_nodes(reference_db)

    res = ReferenceCompareResult(
        candidate_db=str(candidate_db),
        reference_db=str(reference_db),
        n_candidate_nodes=len(cand),
        n_reference_nodes=len(ref),
        tau_m=tau_m,
        tracking_loss_events=sum(1 for n in cand if n.is_lost),
    )
    res.tracking_loss_ratio = (
        res.tracking_loss_events / max(1, res.n_candidate_nodes)
    )

    cand_xyz = np.array([n.pose[:3, 3] for n in cand if n.pose is not None])
    ref_xyz = np.array([n.pose[:3, 3] for n in ref if n.pose is not None])
    res.candidate_with_pose = len(cand_xyz)
    res.reference_with_pose = len(ref_xyz)

    if len(cand_xyz) == 0 or len(ref_xyz) == 0:
        res.warnings.append(
            f'candidate_with_pose={len(cand_xyz)} '
            f'reference_with_pose={len(ref_xyz)} — cannot compute correspondence'
        )
        return res

    # Frame alignment: when a bag is recorded with a different starting yaw
    # than the demo, the two dbs' coordinate frames are rotated relative to
    # each other (each starts at identity). Search yaw around the start to
    # find the rotation that maximises correspondence_ratio. Without this,
    # the NN distances are inflated by frame misalignment — verified via
    # ICP-aligning the assembled clouds (e.g. terrain cand 1 vs the demo
    # required ~143° rotation, RMSE 7.9 cm after alignment).
    best_yaw, best_cand_xyz = _align_by_yaw(cand_xyz, ref_xyz, tau_m=tau_m)
    res.alignment_yaw_rad = float(best_yaw)

    diffs = best_cand_xyz[:, None, :] - ref_xyz[None, :, :]
    dists = np.linalg.norm(diffs, axis=2)
    nn_dist = dists.min(axis=1)

    res.mean_nn_distance_m = float(np.mean(nn_dist))
    res.median_nn_distance_m = float(np.median(nn_dist))
    res.max_nn_distance_m = float(np.max(nn_dist))
    res.correspondence_ratio = float(np.mean(nn_dist <= tau_m))
    return res


def _align_by_yaw(cand_xyz: np.ndarray, ref_xyz: np.ndarray, tau_m: float
                   ) -> tuple:
    """Coarse-to-fine yaw search around z-axis to align candidate poses to
    reference. Returns (best_yaw_rad, transformed candidate xyz)."""
    def _corr_at_yaw(yaw: float) -> float:
        c, s = np.cos(yaw), np.sin(yaw)
        R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        rotated = cand_xyz @ R.T
        diffs = rotated[:, None, :] - ref_xyz[None, :, :]
        dists = np.linalg.norm(diffs, axis=2)
        nn_dist = dists.min(axis=1)
        return float(np.mean(nn_dist <= tau_m))

    # Coarse — 36 steps at 10° resolution.
    yaws_coarse = np.linspace(0, 2 * np.pi, 36, endpoint=False)
    scores_coarse = [_corr_at_yaw(y) for y in yaws_coarse]
    best_idx = int(np.argmax(scores_coarse))
    best_yaw = yaws_coarse[best_idx]

    # Refine — 21 steps in ±10° around the coarse best.
    yaws_fine = best_yaw + np.linspace(-np.pi/18, np.pi/18, 21)
    scores_fine = [_corr_at_yaw(y) for y in yaws_fine]
    best_yaw = yaws_fine[int(np.argmax(scores_fine))]

    # Apply best rotation.
    c, s = np.cos(best_yaw), np.sin(best_yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return best_yaw, cand_xyz @ R.T


def main() -> int:
    p = argparse.ArgumentParser(prog='compare_to_reference')
    p.add_argument('--candidate', required=True,
                   help='Path to candidate (trial-produced) rtabmap.db')
    p.add_argument('--reference', required=True,
                   help='Path to reference (cleaned-up) rtabmap.db')
    p.add_argument('--tau', type=float, default=0.5,
                   help='Distance threshold (m) for correspondence.')
    p.add_argument('--out', default=None,
                   help='Output JSON path (default: stdout).')
    args = p.parse_args()

    result = compare(
        candidate_db=Path(args.candidate).expanduser().resolve(),
        reference_db=Path(args.reference).expanduser().resolve(),
        tau_m=args.tau,
    )
    payload = json.dumps(asdict(result), indent=2)
    if args.out:
        Path(args.out).write_text(payload)
        print(f'wrote {args.out}')
    else:
        print(payload)
    return 0


if __name__ == '__main__':
    sys.exit(main())
