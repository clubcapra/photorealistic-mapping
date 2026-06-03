"""Unit tests for the aggregator + cleanup utilities.

Lightweight: doesn't require ROS sourced. Run with `python3 -m pytest tests/`
from the package root, or directly:
    python3 tests/test_aggregators.py
"""

from __future__ import annotations

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

# Make the package importable when run directly (not via colcon install).
PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG_ROOT))

from rove_rtabmap_tuner.optimizer import _quantile, _aggregate_metric, METRICS  # noqa: E402
from rove_rtabmap_tuner.trial_runner import cleanup_orphan_trials  # noqa: E402


class FakeRun:
    """Stand-in for BagRunResult — just needs .success + .metrics dict."""
    def __init__(self, drift, success=True):
        self.success = success
        if success:
            self.metrics = {
                'success': True,
                'stats': {
                    'drift_m': drift * 30,  # arbitrary scale
                    'drift_per_path': drift,
                    'mean_icp_ratio': 0.5,
                    'loop_closure_count': 10,
                },
            }
        else:
            self.metrics = {'success': False, 'error': 'fake fail'}


class TestQuantile(unittest.TestCase):
    def test_q75_linear_interpolation(self):
        # 10 evenly-spaced values; q75 sits at index 6.75
        vals = [0.01 * i for i in range(1, 11)]
        # Interpolation: 0.75 * 9 = 6.75 → between vals[6]=0.07 and vals[7]=0.08
        # = 0.07 * 0.25 + 0.08 * 0.75 = 0.0775
        self.assertAlmostEqual(_quantile(vals, 0.75), 0.0775, places=4)

    def test_q90(self):
        vals = [0.01 * i for i in range(1, 11)]
        # 0.9 * 9 = 8.1 → between vals[8]=0.09 and vals[9]=0.10
        # = 0.09 * 0.9 + 0.10 * 0.1 = 0.091
        self.assertAlmostEqual(_quantile(vals, 0.90), 0.091, places=4)

    def test_single_value(self):
        self.assertEqual(_quantile([0.5], 0.75), 0.5)
        self.assertEqual(_quantile([0.5], 0.0), 0.5)
        self.assertEqual(_quantile([0.5], 1.0), 0.5)

    def test_empty(self):
        self.assertEqual(_quantile([], 0.5), 0.0)

    def test_q50_is_median_for_odd_count(self):
        # For odd count, q50 should equal median
        vals = [0.1, 0.3, 0.5, 0.7, 0.9]
        self.assertAlmostEqual(_quantile(vals, 0.5), 0.5)


class TestAggregator(unittest.TestCase):
    def test_max_aggregator_catches_outlier(self):
        # 9 OK bags + 1 catastrophic
        runs = [FakeRun(d) for d in [0.02, 0.03, 0.04, 0.05, 0.05, 0.06, 0.07, 0.08, 1.0]]
        max_metric = METRICS['max_drift_per_path']
        agg, vals = _aggregate_metric(max_metric, runs)
        self.assertAlmostEqual(agg, 1.0)

    def test_q75_softens_outlier(self):
        runs = [FakeRun(d) for d in [0.02, 0.03, 0.04, 0.05, 0.05, 0.06, 0.07, 0.08, 1.0]]
        q75_metric = METRICS['q75_drift_per_path']
        agg, _ = _aggregate_metric(q75_metric, runs)
        # 9 values, q75 idx = 0.75 * 8 = 6.0 → vals[6] sorted = 0.07
        self.assertAlmostEqual(agg, 0.07)

    def test_median_hides_outlier(self):
        runs = [FakeRun(d) for d in [0.02, 0.03, 0.04, 0.05, 0.05, 0.06, 0.07, 0.08, 1.0]]
        med_metric = METRICS['drift_per_path']
        agg, _ = _aggregate_metric(med_metric, runs)
        # Median of 9 sorted: middle = 0.05
        self.assertAlmostEqual(agg, 0.05)

    def test_failed_bag_uses_fail_value(self):
        runs = [FakeRun(0.05), FakeRun(0.05), FakeRun(0.05, success=False)]
        max_metric = METRICS['max_drift_per_path']
        agg, vals = _aggregate_metric(max_metric, runs)
        # max should pick up the fail_value=1.0
        self.assertEqual(agg, 1.0)
        self.assertIn(1.0, vals)


class TestCleanupOrphanTrials(unittest.TestCase):
    def setUp(self):
        self.root = Path(tempfile.mkdtemp(prefix='cleanup_test_'))

    def tearDown(self):
        shutil.rmtree(self.root, ignore_errors=True)

    def test_keeps_complete_removes_incomplete(self):
        (self.root / 'trial_0001').mkdir()
        (self.root / 'trial_0001' / 'trial.json').write_text('{}')
        (self.root / 'trial_0002').mkdir()  # incomplete
        (self.root / 'trial_0003').mkdir()
        (self.root / 'trial_0003' / 'bag_a').mkdir()  # also incomplete
        (self.root / 'unrelated_dir').mkdir()  # not a trial dir

        stats = cleanup_orphan_trials(self.root)

        self.assertEqual(stats['inspected'], 3)
        self.assertEqual(stats['removed'], 2)
        self.assertEqual(stats['kept'], 1)
        self.assertTrue((self.root / 'trial_0001').is_dir())
        self.assertFalse((self.root / 'trial_0002').exists())
        self.assertFalse((self.root / 'trial_0003').exists())
        self.assertTrue((self.root / 'unrelated_dir').is_dir())

    def test_missing_root(self):
        stats = cleanup_orphan_trials(Path('/tmp/this_dir_does_not_exist_xyz'))
        self.assertEqual(stats, {'inspected': 0, 'removed': 0, 'kept': 0})

    def test_empty_root(self):
        stats = cleanup_orphan_trials(self.root)
        self.assertEqual(stats, {'inspected': 0, 'removed': 0, 'kept': 0})


if __name__ == '__main__':
    unittest.main()
