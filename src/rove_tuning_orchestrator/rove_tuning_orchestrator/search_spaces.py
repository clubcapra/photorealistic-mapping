"""Load + apply stage search-space YAML files.

Format (see config/search_spaces/*.yaml):

    name: stage_a_icp_core
    description: ...
    params:
      Icp/VoxelSize:
        type: float        # float | int | categorical
        low: 0.02          # for float/int
        high: 0.30
        log: true          # optional, for float
        choices: ['0','1'] # for categorical
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


@dataclass
class ParamSpec:
    name: str
    kind: str  # 'float' | 'int' | 'categorical'
    low: Optional[float] = None
    high: Optional[float] = None
    log: bool = False
    choices: List[str] = field(default_factory=list)

    def suggest(self, trial) -> Any:
        """Apply this spec to an Optuna trial."""
        if self.kind == 'float':
            return trial.suggest_float(self.name, self.low, self.high, log=self.log)
        if self.kind == 'int':
            return trial.suggest_int(self.name, int(self.low), int(self.high))
        if self.kind == 'categorical':
            return trial.suggest_categorical(self.name, self.choices)
        raise ValueError(f'Unknown param kind: {self.kind}')


@dataclass
class SearchSpace:
    name: str
    description: str
    params: List[ParamSpec]

    def suggest_all(self, trial) -> Dict[str, Any]:
        """Convert one Optuna trial into a dict of RTAB-Map params."""
        return {p.name: p.suggest(trial) for p in self.params}


def load(path: Path) -> SearchSpace:
    data = yaml.safe_load(path.read_text())
    params: List[ParamSpec] = []
    for pname, spec in (data.get('params') or {}).items():
        kind = spec.get('type', 'float')
        params.append(ParamSpec(
            name=pname,
            kind=kind,
            low=spec.get('low'),
            high=spec.get('high'),
            log=bool(spec.get('log', False)),
            choices=[str(c) for c in spec.get('choices', [])],
        ))
    return SearchSpace(
        name=data['name'],
        description=data.get('description', ''),
        params=params,
    )


def load_from_package_share(filename: str) -> SearchSpace:
    """Load a stage YAML by name from the installed package's share dir."""
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('rove_tuning_orchestrator'))
    return load(share / 'config' / 'search_spaces' / filename)


def build_refine_space(
    best_per_stage: Dict[str, Dict[str, Any]],
    half_width_frac: float = 0.20,
) -> SearchSpace:
    """Build the stage-E joint-refine space from previous stages' bests.

    For each float/int param, the new bounds are best * (1 +/- half_width_frac).
    Categorical params are pinned to their best value (no further search).
    """
    params: List[ParamSpec] = []
    for stage_name, best_params in best_per_stage.items():
        for pname, value in best_params.items():
            if isinstance(value, bool) or isinstance(value, str):
                # Categorical — pin via single-choice.
                params.append(ParamSpec(
                    name=pname, kind='categorical',
                    choices=[str(value).lower() if isinstance(value, bool) else str(value)],
                ))
            elif isinstance(value, (int, float)):
                v = float(value)
                half = abs(v) * half_width_frac if v != 0 else half_width_frac
                low = max(0.0, v - half) if v >= 0 else v - half
                high = v + half
                kind = 'int' if isinstance(value, int) else 'float'
                params.append(ParamSpec(
                    name=pname, kind=kind, low=low, high=high, log=False,
                ))
    return SearchSpace(
        name='stage_e_joint_refine',
        description='Narrow window around best from prior stages.',
        params=params,
    )


def merge_params(
    *param_dicts: Dict[str, Any],
) -> Dict[str, Any]:
    """Right-most wins. Used to compose params from multiple stages."""
    out: Dict[str, Any] = {}
    for d in param_dicts:
        out.update(d)
    return out
