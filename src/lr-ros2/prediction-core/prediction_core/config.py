"""Validated YAML configuration for rover geometry and prediction settings."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import yaml


def _positive(name: str, value: Any) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return number


def _finite(name: str, value: Any) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


@dataclass(frozen=True)
class PredictionConfig:
    collision_margin_m: float = 0.20

    def __post_init__(self) -> None:
        if not math.isfinite(self.collision_margin_m) or self.collision_margin_m < 0:
            raise ValueError("collision_margin_m must be finite and non-negative")


@dataclass(frozen=True)
class RoverConfig:
    """Static rover geometry.

    CoM (Center of Mass) and CoG are equivalent for this baseline under
    uniform gravity.  The body footprint is collision geometry; the smaller
    support rectangle is used exclusively for quasi-static rollover.
    """

    mass_kg: float
    body_length_m: float
    body_width_m: float
    body_height_m: float
    support_length_m: float
    support_width_m: float
    ground_clearance_m: float
    com_x_m: float
    com_y_m: float
    com_height_m: float
    prediction: PredictionConfig

    def __post_init__(self) -> None:
        numeric_values = (
            self.mass_kg,
            self.body_length_m,
            self.body_width_m,
            self.body_height_m,
            self.support_length_m,
            self.support_width_m,
            self.ground_clearance_m,
            self.com_x_m,
            self.com_y_m,
            self.com_height_m,
        )
        if not all(math.isfinite(value) for value in numeric_values):
            raise ValueError("all rover configuration values must be finite")
        if any(
            value <= 0
            for value in (
                self.mass_kg,
                self.body_length_m,
                self.body_width_m,
                self.body_height_m,
                self.support_length_m,
                self.support_width_m,
                self.com_height_m,
            )
        ):
            raise ValueError("mass, dimensions, and CoM height must be positive")
        if self.ground_clearance_m < 0:
            raise ValueError("ground clearance must be non-negative")
        if (
            abs(self.com_x_m) >= self.support_length_m / 2
            or abs(self.com_y_m) >= self.support_width_m / 2
        ):
            raise ValueError("configured CoM must begin inside the support rectangle")

    @property
    def length_m(self) -> float:
        """Deprecated compatibility alias for collision body length."""
        return self.body_length_m

    @property
    def width_m(self) -> float:
        """Deprecated compatibility alias for collision body width."""
        return self.body_width_m

    @property
    def cg_x_m(self) -> float:
        """Deprecated alias for com_x_m."""
        return self.com_x_m

    @property
    def cg_y_m(self) -> float:
        """Deprecated alias for com_y_m."""
        return self.com_y_m

    @property
    def cg_height_m(self) -> float:
        """Deprecated alias for com_height_m."""
        return self.com_height_m


def load_config(path: str | Path) -> RoverConfig:
    """Load rover configuration.

    The legacy ``length_m``/``width_m``/``cg_*`` YAML schema remains accepted
    as a migration path. It maps one rectangle to both body and support
    geometry, so it must be replaced with the explicit schema before field use.
    """
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError(f"{config_path} must contain a YAML mapping")
    try:
        rover = raw["rover"]
        prediction = raw["prediction"]
        is_legacy = "body_length_m" not in rover
        body_length = rover["length_m"] if is_legacy else rover["body_length_m"]
        body_width = rover["width_m"] if is_legacy else rover["body_width_m"]
        com_x = rover["cg_x_m"] if is_legacy else rover["com_x_m"]
        com_y = rover["cg_y_m"] if is_legacy else rover["com_y_m"]
        com_height = rover["cg_height_m"] if is_legacy else rover["com_height_m"]
        return RoverConfig(
            mass_kg=_positive("rover.mass_kg", rover.get("mass_kg", 1.0)),
            body_length_m=_positive("rover.body_length_m", body_length),
            body_width_m=_positive("rover.body_width_m", body_width),
            body_height_m=_positive("rover.body_height_m", rover.get("body_height_m", 1.0)),
            support_length_m=_positive(
                "rover.support_length_m", rover.get("support_length_m", body_length)
            ),
            support_width_m=_positive(
                "rover.support_width_m", rover.get("support_width_m", body_width)
            ),
            ground_clearance_m=_finite(
                "rover.ground_clearance_m", rover.get("ground_clearance_m", 0.0)
            ),
            com_x_m=_finite("rover.com_x_m", com_x),
            com_y_m=_finite("rover.com_y_m", com_y),
            com_height_m=_positive("rover.com_height_m", com_height),
            prediction=PredictionConfig(
                collision_margin_m=_finite(
                    "prediction.collision_margin_m", prediction["collision_margin_m"]
                )
            ),
        )
    except (KeyError, TypeError) as exc:
        raise ValueError(f"missing or malformed config field: {exc}") from exc
