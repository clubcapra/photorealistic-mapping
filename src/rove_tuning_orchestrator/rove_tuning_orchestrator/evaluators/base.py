"""Evaluator interface — the contract every metric source must implement.

An evaluator takes a parameter dict (the RTAB-Map params under test) and a
trial label, runs whatever computation it needs (sim, bag replay, etc.),
and returns an EvaluationResult holding the score plus diagnostic detail.

The orchestrator does not care HOW the evaluator computes the score — only
that it conforms to the interface. This keeps sim, real-bag, future
on-robot, and any other source of metrics interchangeable.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List


@dataclass
class EvaluationResult:
    """Outcome of one evaluation.

    `score` is the scalar Optuna optimizes against (lower-is-better
    convention — flip in the wrapper if you have a higher-is-better metric).
    `failed` is True if the evaluator could not produce a usable score
    (e.g. RTAB-Map crashed, sim hung); the orchestrator treats failed
    trials as worst-case for the sampler.
    """
    score: float
    failed: bool = False
    failure_reason: str = ''
    # Free-form per-metric breakdown — written into Optuna user_attrs.
    metrics: Dict[str, Any] = field(default_factory=dict)
    # Paths to artifacts produced (bag dir, db file, validation json).
    artifacts: Dict[str, str] = field(default_factory=dict)
    # Warnings / non-fatal notes (e.g. "fell back to default param X").
    warnings: List[str] = field(default_factory=list)


class Evaluator(abc.ABC):
    """Base class. Subclass + implement evaluate()."""

    #: Short identifier used in Optuna user_attrs and log lines.
    name: str = 'evaluator'

    @abc.abstractmethod
    def evaluate(
        self,
        params: Dict[str, Any],
        trial_id: str,
        out_dir: Path,
    ) -> EvaluationResult:
        """Run one evaluation.

        params: RTAB-Map parameters dict (string keys, mixed value types)
        trial_id: stable identifier (Optuna's trial.number or similar)
        out_dir: pre-created directory for this evaluator's artifacts
        """
        raise NotImplementedError

    def warmup(self) -> None:
        """Optional: spawn one-time resources (e.g. start a worker pool).

        Called once before the first evaluate() on this instance.
        """

    def shutdown(self) -> None:
        """Optional: tear down resources."""
