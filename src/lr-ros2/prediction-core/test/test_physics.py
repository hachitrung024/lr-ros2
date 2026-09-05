"""Canonical physics regressions, including cases migrated from the old UI engine."""

from dataclasses import asdict
import json
import math
from pathlib import Path

import pytest

from prediction_core.config import load_config
from prediction_core.models import (
    GeometryStep,
    RoverState,
    TrackedObject,
    Trajectory,
    TrajectoryStep,
)
from prediction_core.predictor import PredictionCore


CASES = ('flat', 'slope', 'dynamic', 'margin', 'intersection', 'unknown_terrain')


def evidence(case):
    config = load_config(Path(__file__).parents[1] / 'config/rover.reference.yaml')
    trajectory = Trajectory(10.0, 'map', [TrajectoryStep(0, 0.0, 0.0, 0.0)])
    slope = math.radians(25) if case in ('slope', 'dynamic') else 0.0
    geometry = [GeometryStep(10.0, 0, 'plane', (0.0, math.sin(slope), math.cos(slope)))]
    state = RoverState(10.0, acceleration_xyz=(1.0, 0.5, 0.0)) if case == 'dynamic' else None
    objects = []
    if case in ('margin', 'intersection'):
        start = 0.6 if case == 'margin' else 0.0
        objects = [
            TrackedObject(
                10.0, 7, 'rock', [(start, -0.3), (start + 1, -0.3), (start + 1, 0.3), (start, 0.3)]
            )
        ]
    if case == 'unknown_terrain':
        geometry = []
    result = asdict(PredictionCore(config).predict(trajectory, objects, geometry, state))
    result.pop('timestamp')  # Wall time is not part of the physical evidence.
    return result


def assert_same(actual, expected):
    if isinstance(expected, dict):
        assert actual.keys() == expected.keys()
        for key in expected:
            assert_same(actual[key], expected[key])
    elif isinstance(expected, list):
        assert len(actual) == len(expected)
        for first, second in zip(actual, expected):
            assert_same(first, second)
    elif isinstance(expected, float):
        assert actual == pytest.approx(expected, abs=1e-6, nan_ok=True)
    else:
        assert actual == expected


@pytest.mark.parametrize('case', CASES)
def test_evidence_matches_pre_refactor_engine(case):
    expected = json.loads(
        (Path(__file__).parent / 'fixtures/pre_refactor_evidence.json').read_text()
    )
    assert_same(evidence(case), expected[case])
