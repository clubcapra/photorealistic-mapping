"""Optuna JournalFileStorage helpers.

Why JournalFileStorage:
- Pure file-based, no database server.
- Concurrent writers from multiple processes / machines (filesystem locks).
- Distributed: put the journal on a shared dir (NFS / syncthing / rclone-
  mounted) and multiple workers can append. No open ports anywhere.

Studies layout:
    <studies_root>/
      <project_name>/
        sim_phase1.journal       # phase-1 (sim) study
        real_phase2.journal      # phase-2 (real bag) study
        artifacts/<trial>/...    # per-trial outputs

A separate journal per phase keeps Optuna's sampler state clean — sim trials
and real trials are scored on different metrics, so they live in separate
studies that the orchestrator coordinates externally.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import optuna
from optuna.storages.journal import JournalFileBackend
from optuna.storages import JournalStorage


def journal_storage(study_root: Path, study_name: str) -> JournalStorage:
    """Create or open a JournalFileStorage at <study_root>/<study_name>.journal."""
    study_root.mkdir(parents=True, exist_ok=True)
    journal_path = study_root / f'{study_name}.journal'
    return JournalStorage(JournalFileBackend(str(journal_path)))


def create_or_load_study(
    study_root: Path,
    study_name: str,
    sampler: Optional[optuna.samplers.BaseSampler] = None,
    direction: str = 'minimize',
) -> optuna.Study:
    """Resume if it exists, otherwise create."""
    storage = journal_storage(study_root, study_name)
    return optuna.create_study(
        study_name=study_name,
        storage=storage,
        sampler=sampler,
        direction=direction,
        load_if_exists=True,
    )


def make_sampler(kind: str, seed: Optional[int] = None) -> optuna.samplers.BaseSampler:
    """One place to construct samplers so the CLI defaults stay coherent."""
    if kind == 'cma_es' or kind == 'cmaes':
        return optuna.samplers.CmaEsSampler(seed=seed, restart_strategy='ipop')
    if kind == 'tpe':
        return optuna.samplers.TPESampler(seed=seed, multivariate=True)
    if kind == 'random':
        return optuna.samplers.RandomSampler(seed=seed)
    raise ValueError(f'Unknown sampler kind: {kind}')
