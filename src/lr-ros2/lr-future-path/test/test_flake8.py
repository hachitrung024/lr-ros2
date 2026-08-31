"""Run flake8 against this package only."""

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    """Check package Python style."""
    package_root = Path(__file__).resolve().parents[1]
    rc, errors = main_with_errors(argv=[str(package_root)])
    assert rc == 0, "\n".join(errors)
