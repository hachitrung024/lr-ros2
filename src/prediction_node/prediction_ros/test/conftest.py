"""Shared fixtures for prediction_ros tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from prediction_core.config import load_config
from prediction_core.predictor import PredictionCore

from prediction_ros.cache import PredictionInputCache
from prediction_ros.coordinator import PredictionCoordinator
from prediction_ros.validation import InputValidator, ValidationConfig


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def mock_config():
    return load_config(ROOT / "config" / "rover.mock.yaml")


@pytest.fixture
def validator() -> InputValidator:
    return InputValidator(ValidationConfig(expected_frame_id="map"))


@pytest.fixture
def cache() -> PredictionInputCache:
    return PredictionInputCache()


@pytest.fixture
def core(mock_config) -> PredictionCore:
    return PredictionCore(mock_config)


@pytest.fixture
def coordinator(core, cache, validator) -> PredictionCoordinator:
    return PredictionCoordinator(core, cache, validator)
