"""Map quality validation: compare RTAB-Map's accumulated point cloud against
the simulator's ground-truth geometry.

NOT YET IMPLEMENTED — interface stub. The trajectory-validator path
(validator.py) is sufficient for grading SLAM runs against ground truth and
is the higher-signal metric for the tuner. Build this out when the trajectory
metric stops being the bottleneck.

Planned approach:
1. Sample the world's ground-truth surfaces to a dense reference point cloud
   by parsing the .wbt's PROTO instances + their referenced mesh files
   (.stl / .dae / .obj). Use trimesh for surface sampling. Each instance's
   pose comes from its Pose/translation/rotation fields.
2. Extract RTAB-Map's accumulated cloud either via rtabmap-export
   --output-format=ply, or by subscribing to /rtabmap/cloud_map during the
   live run.
3. Align the two clouds with the SE(3) transform from validator.py's
   trajectory alignment (reuse the same transform to avoid double-fitting).
4. Compute:
   - chamfer_mean / chamfer_rmse — mean / RMS bidirectional NN distance.
   - one_sided_est_to_ref — coverage of the world by the estimated map.
   - one_sided_ref_to_est — coverage of the estimated map by the world
     (high values = phantom geometry / drift smearing).
   - surface_completeness — fraction of ref points within tau of an est point.
   - phantom_fraction — fraction of est points further than tau from any ref.

Output: extends validation.json with the above fields.

Implementation tradeoffs:
- For a 224 MiB bag, RTAB-Map's accumulated cloud is ~1-10 M points. Naive
  pairwise NN is O(N^2) — use a kd-tree (scipy.spatial.cKDTree) or downsample
  via voxel grid to ~100 k points before NN.
- Webots EXTERNPROTO meshes are fetched from github at world-load time;
  trimesh would need the same URLs (or a local cache).
- For Webots-builtin primitives (Box, Cylinder, UnevenTerrain) we'd sample
  analytically rather than mesh-load.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MapValidationResult:
    chamfer_rmse_m: float
    chamfer_mean_m: float
    surface_completeness: float  # fraction of ref points within tau of est
    phantom_fraction: float      # fraction of est points further than tau from ref
    tau_m: float                 # the threshold used for the binary metrics
    n_ref_points: int
    n_est_points: int


def validate_map(
    rtabmap_db_or_ply: str,
    world_file: str,
    tau_m: float = 0.10,
) -> MapValidationResult:
    raise NotImplementedError(
        'map_validator is a stub. See docstring for the planned implementation.'
    )
