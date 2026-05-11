"""Render the RTAB-Map tunable launch template into a concrete launch file.

The template lives at ``templates/lidar3d_tunable.launch.py.tmpl`` and uses
``string.Template`` placeholders for every tunable hyper-parameter. Callers
pass an overrides dict; missing keys fall back to ``DEFAULTS`` (which mirror
the values currently hard-coded in ``rove_color_mapping/launch/lidar3d.launch.py``).

The renderer validates two things before writing:
  1. Every placeholder in the template is filled (``string.Template.substitute``
     raises ``KeyError`` otherwise).
  2. The rendered text parses as Python (``ast.parse``).
"""

from __future__ import annotations

import argparse
import ast
import re
from pathlib import Path
from string import Template

# Defaults mirror the values currently hard-coded in lidar3d.launch.py.
# All values are strings because RTAB-Map parameters are string-typed at the
# ROS layer (the C++ side parses them).
DEFAULTS: dict[str, str] = {
    # ---- shared ICP --------------------------------------------------------
    'icp_point_to_plane': 'true',
    'icp_iterations': '10',
    'icp_voxel_size': '0.01',
    'icp_epsilon': '0.0001',
    'icp_point_to_plane_k': '20',
    'icp_point_to_plane_radius': '0',
    'icp_max_translation': '0.5',
    # max_correspondence_distance was previously voxel_size * 10 (= 0.1).
    'icp_max_correspondence_distance': '0.1',
    'icp_strategy': '1',
    'icp_outlier_ratio': '0.65',

    # ---- ICP odometry ------------------------------------------------------
    'odom_scan_keyframe_thr': '0.4',
    # ScanSubtractRadius was previously equal to voxel_size (= 0.01).
    'odomf2m_scan_subtract_radius': '0.01',
    'odomf2m_scan_max_size': '25000',
    'odomf2m_bundle_adjustment': 'false',
    'icp_odom_correspondence_ratio': '0.03',
    'icp_point_to_plane_min_complexity': '0.0',

    # ---- RTAB-Map SLAM -----------------------------------------------------
    'rgbd_proximity_max_graph_depth': '0',
    'rgbd_proximity_path_max_neighbors': '1',
    'rgbd_angular_update': '0.1',
    'rgbd_linear_update': '0.1',
    'rgbd_create_occupancy_grid': 'true',
    'mem_not_linked_nodes_kept': 'false',
    'mem_stm_size': '30',
    'reg_strategy': '1',
    'icp_map_correspondence_ratio': '0.1',
    'grid_range_max': '20.0',
    'grid_max_ground_height': '0.25',
    'grid_max_obstacle_height': '1.8',
    'grid_min_cluster_size': '30',
    'grid_normals_segmentation': 'true',
    'grid_footprint_height': '0.0',
    'grid_cell_size': '0.05',
}

_PLACEHOLDER_RE = re.compile(r'\$\{([a-zA-Z_][a-zA-Z0-9_]*)\}')


def template_path() -> Path:
    return Path(__file__).parent / 'templates' / 'lidar3d_tunable.launch.py.tmpl'


def known_keys() -> set[str]:
    """Return the set of placeholder names present in the template file.

    Useful for validating an overrides dict against the template before
    rendering, and for generating CLI help.
    """
    return set(_PLACEHOLDER_RE.findall(template_path().read_text()))


def _coerce(value: object) -> str:
    """Coerce a Python value to the string form RTAB-Map expects."""
    if value is True:
        return 'true'
    if value is False:
        return 'false'
    return str(value)


def effective_params(
    overrides: dict[str, object] | None = None,
    *,
    template: Path | None = None,
) -> dict[str, str]:
    """Merge ``overrides`` over ``DEFAULTS`` for every placeholder in the
    template, coercing values to strings. Raises ``KeyError`` if overrides
    target placeholders that don't exist or if a placeholder has neither a
    default nor an override.
    """
    overrides = overrides or {}
    template = template or template_path()

    placeholders = set(_PLACEHOLDER_RE.findall(template.read_text()))
    unknown = set(overrides) - placeholders
    if unknown:
        raise KeyError(
            f'Overrides target placeholders not in the template: '
            f'{sorted(unknown)}. Known placeholders: {sorted(placeholders)}'
        )

    merged: dict[str, str] = {}
    for key in placeholders:
        if key in overrides:
            merged[key] = _coerce(overrides[key])
        elif key in DEFAULTS:
            merged[key] = DEFAULTS[key]
        else:
            raise KeyError(
                f'Template placeholder ${{{key}}} has no default and was not '
                f'provided in overrides. Either add it to DEFAULTS or pass it explicitly.'
            )
    return merged


def render(
    overrides: dict[str, object] | None = None,
    *,
    output_path: Path,
    template: Path | None = None,
) -> Path:
    """Render the template with ``DEFAULTS`` merged with ``overrides`` and
    write it to ``output_path``. Returns ``output_path``.

    ``overrides`` values may be any type with a ``str()``; booleans are
    lowercased to match RTAB-Map's string-bool convention.
    """
    template = template or template_path()
    merged = effective_params(overrides, template=template)
    rendered = Template(template.read_text()).substitute(merged)

    # Sanity check: catches typos like an unquoted placeholder in the template.
    try:
        ast.parse(rendered)
    except SyntaxError as exc:
        raise RuntimeError(f'Rendered launch file is not valid Python: {exc}') from exc

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(rendered)
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        '--output', '-o', type=Path,
        help='Path to write the rendered launch file. Required unless --list-keys.',
    )
    parser.add_argument(
        '--set', '-s', action='append', default=[], metavar='KEY=VALUE',
        help='Override one placeholder; repeat for multiple. Unset keys take DEFAULTS.',
    )
    parser.add_argument(
        '--list-keys', action='store_true',
        help='Print the placeholder names found in the template and exit.',
    )
    args = parser.parse_args()

    if args.list_keys:
        for key in sorted(known_keys()):
            default = DEFAULTS.get(key, '<no default>')
            print(f'{key} = {default}')
        return 0

    if args.output is None:
        parser.error('--output is required unless --list-keys is given')

    overrides: dict[str, str] = {}
    for entry in args.set:
        if '=' not in entry:
            parser.error(f'--set expects KEY=VALUE, got: {entry!r}')
        key, _, value = entry.partition('=')
        overrides[key] = value

    path = render(overrides, output_path=args.output)
    print(f'Rendered to {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
