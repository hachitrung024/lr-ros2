"""Run pydocstyle against this package only."""

from pathlib import Path

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check package docstrings."""
    package_root = Path(__file__).resolve().parents[1]
    assert main(argv=[str(package_root)]) == 0
