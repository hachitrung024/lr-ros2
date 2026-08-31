"""Cache identity, location, and manifest validation helpers."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

from . import CACHE_SCHEMA_VERSION


CACHE_TOPIC = "/lr/future_path/cache/pose"
CACHE_DIRECTORY_NAME = ".lr_future_path_cache"
TRACKING_PROFILE = "zed_wrapper_default_v1"


@dataclass(frozen=True)
class CacheSpec:
    """Resolved cache path and the identity expected in its manifest."""

    path: Path
    identity: dict[str, Any]
    fingerprint: str


def _zed_sdk_version() -> str:
    try:
        import pyzed.sl as sl

        return str(sl.Camera.get_sdk_version())
    except (ImportError, RuntimeError):
        return "unknown"


def _ros_package_version(package_name: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory

        package_xml = Path(
            get_package_share_directory(package_name)
        ) / "package.xml"
        root = ET.parse(package_xml).getroot()
        value = root.findtext("version")
        return value.strip() if value else "unknown"
    except (ImportError, LookupError, OSError, ET.ParseError):
        return "unknown"


def build_cache_spec(
    svo_path: str | Path,
    camera_model: str,
    cache_root: str | Path = "",
    *,
    zed_sdk_version: str | None = None,
    zed_wrapper_version: str | None = None,
) -> CacheSpec:
    """Build a deterministic cache identity and destination for an SVO."""
    source = Path(svo_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"SVO file does not exist: {source}")
    if not camera_model.strip():
        raise ValueError("camera_model must not be empty")

    stat = source.stat()
    identity = {
        "schema_version": CACHE_SCHEMA_VERSION,
        "source": {
            "path": str(source),
            "size_bytes": stat.st_size,
            "mtime_ns": stat.st_mtime_ns,
        },
        "camera_model": camera_model,
        "tracking_profile": TRACKING_PROFILE,
        "zed_sdk_version": (
            _zed_sdk_version()
            if zed_sdk_version is None
            else str(zed_sdk_version)
        ),
        "zed_wrapper_version": (
            _ros_package_version("zed_wrapper")
            if zed_wrapper_version is None
            else str(zed_wrapper_version)
        ),
        "pose_topic": CACHE_TOPIC,
    }
    canonical = json.dumps(
        identity, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    fingerprint = hashlib.sha256(canonical).hexdigest()[:16]
    root = (
        Path(cache_root).expanduser().resolve()
        if str(cache_root).strip()
        else source.parent / CACHE_DIRECTORY_NAME
    )
    return CacheSpec(
        path=root / f"{source.stem}-{fingerprint}",
        identity=identity,
        fingerprint=fingerprint,
    )


def manifest_path(cache_path: str | Path) -> Path:
    """Return the LR manifest path for a rosbag cache directory."""
    return Path(cache_path) / "manifest.json"


def load_manifest(cache_path: str | Path) -> dict[str, Any]:
    """Load a cache manifest, raising a useful error if it is invalid."""
    path = manifest_path(cache_path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exception:
        raise ValueError(f"cache manifest is missing: {path}") from exception
    except (OSError, json.JSONDecodeError) as exception:
        raise ValueError(f"cache manifest is invalid: {path}") from exception
    if not isinstance(data, dict):
        raise ValueError(f"cache manifest must contain an object: {path}")
    return data


def cache_is_valid(
    cache_path: str | Path,
    expected_identity: dict[str, Any] | None = None,
) -> bool:
    """Check manifest identity and required rosbag files."""
    path = Path(cache_path)
    try:
        manifest = load_manifest(path)
    except ValueError:
        return False
    if manifest.get("complete") is not True:
        return False
    if int(manifest.get("pose_count", 0)) < 2:
        return False
    if manifest.get("pose_topic") != CACHE_TOPIC:
        return False
    if (
        expected_identity is not None
        and manifest.get("identity") != expected_identity
    ):
        return False
    if not (path / "metadata.yaml").is_file():
        return False
    return any(path.glob("*.db3"))
