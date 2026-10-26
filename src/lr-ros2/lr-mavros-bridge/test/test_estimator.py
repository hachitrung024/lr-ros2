"""Causality, dropout, rotation and out-of-order sensor tests."""

import numpy as np
import pytest
from lr_mavros_bridge.estimator import Estimator, Gps, Attitude


def ready():
    e = Estimator()
    e.add_gps(Gps(1_000_000_000, np.array([0.0, 0.0, 0.0])))
    e.add_gps(Gps(1_500_000_000, np.array([0.5, 0.0, 0.0])))
    e.add_attitude(
        Attitude(1_500_000_000, np.array([0.0, 0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0]))
    )
    return e


def test_future_samples_do_not_change_current_estimate():
    e = ready()
    expected, _, evidence = e.estimate(1_600_000_000)
    e.add_gps(Gps(1_700_000_000, np.array([100.0, 0.0, 0.0])))
    e.add_attitude(Attitude(1_700_000_000, np.array([1.0, 0.0, 0.0, 0.0]), np.zeros(3)))
    actual, status, actual_evidence = e.estimate(1_600_000_000)
    assert status == "valid"
    np.testing.assert_allclose(actual.position, [0.6, 0, 0])
    np.testing.assert_allclose(actual.orientation_xyzw, [0, 0, np.sin(0.05), np.cos(0.05)])
    assert actual_evidence == evidence
    np.testing.assert_array_equal(expected.position, actual.position)


@pytest.mark.parametrize(
    "age,status", [(260_000_000, "attitude_stale"), (760_000_000, "gps_stale")]
)
def test_stale_data_never_gets_a_fresh_pose(age, status):
    sample, reason, _ = ready().estimate(1_500_000_000 + age)
    assert sample is None and reason == status


def test_invalid_fix_and_speed_are_rejected():
    e = ready()
    e.add_gps(Gps(1_550_000_000, np.zeros(3), False))
    assert e.estimate(1_600_000_000)[1] == "gps_no_fix"
    e = ready()
    e.add_gps(Gps(1_550_000_000, np.array([100.0, 0.0, 0.0])))
    assert e.estimate(1_600_000_000)[1] == "gps_speed_rejected"


def test_reorder_duplicate_and_reset():
    e = ready()
    assert not e.add_gps(Gps(1_500_000_000, np.ones(3) * 999))
    assert e.add_gps(Gps(900_000_000, np.zeros(3)))
    assert e.estimate(1_600_000_000)[1] == "valid"
    e.reset()
    assert e.estimate(1_600_000_000)[1] == "waiting_samples"


def test_missing_rates_fail_closed():
    e = ready()
    e.add_attitude(Attitude(1_600_000_000, np.array([0.0, 0.0, 0.0, 1.0]), np.full(3, np.nan)))
    assert e.estimate(1_650_000_000)[1] == "invalid_angular_velocity"
