"""Tests for SVO cache identity and validation."""

import json

from lr_future_path.cache import (
    CACHE_DIRECTORY_NAME,
    CACHE_TOPIC,
    build_cache_spec,
    cache_is_valid,
)


def _complete_cache(spec, pose_count=5):
    spec.path.mkdir(parents=True)
    (spec.path / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    (spec.path / "future_path_0.db3").touch()
    (spec.path / "manifest.json").write_text(
        json.dumps(
            {
                "complete": True,
                "identity": spec.identity,
                "pose_topic": CACHE_TOPIC,
                "pose_count": pose_count,
            }
        )
    )


def test_cache_spec_is_stable_and_defaults_beside_svo(tmp_path):
    """A source and configuration resolve to one adjacent cache path."""
    svo = tmp_path / "recording.svo2"
    svo.write_bytes(b"svo")
    first = build_cache_spec(
        svo,
        "zed2i",
        zed_sdk_version="5.4.1",
        zed_wrapper_version="5.4.1",
    )
    second = build_cache_spec(
        svo,
        "zed2i",
        zed_sdk_version="5.4.1",
        zed_wrapper_version="5.4.1",
    )

    assert first == second
    assert first.path.parent == tmp_path / CACHE_DIRECTORY_NAME
    assert first.path.name.startswith("recording-")


def test_cache_fingerprint_changes_with_source_or_camera(tmp_path):
    """Source metadata and camera model participate in the fingerprint."""
    svo = tmp_path / "recording.svo2"
    svo.write_bytes(b"first")
    first = build_cache_spec(
        svo,
        "zed2i",
        zed_sdk_version="sdk",
        zed_wrapper_version="wrapper",
    )
    svo.write_bytes(b"second-version")
    changed_source = build_cache_spec(
        svo,
        "zed2i",
        zed_sdk_version="sdk",
        zed_wrapper_version="wrapper",
    )
    changed_camera = build_cache_spec(
        svo,
        "zedx",
        zed_sdk_version="sdk",
        zed_wrapper_version="wrapper",
    )

    assert first.fingerprint != changed_source.fingerprint
    assert changed_source.fingerprint != changed_camera.fingerprint


def test_cache_validation_requires_complete_matching_bag(tmp_path):
    """Validation rejects absent, incomplete, and mismatched caches."""
    svo = tmp_path / "recording.svo2"
    svo.touch()
    spec = build_cache_spec(
        svo,
        "zed2i",
        tmp_path / "cache",
        zed_sdk_version="sdk",
        zed_wrapper_version="wrapper",
    )
    assert not cache_is_valid(spec.path, spec.identity)

    _complete_cache(spec)
    assert cache_is_valid(spec.path, spec.identity)

    wrong_identity = dict(spec.identity)
    wrong_identity["camera_model"] = "zedx"
    assert not cache_is_valid(spec.path, wrong_identity)


def test_cache_with_too_few_poses_is_invalid(tmp_path):
    """At least two poses are required to form a future path."""
    svo = tmp_path / "recording.svo2"
    svo.touch()
    spec = build_cache_spec(
        svo,
        "zed2i",
        zed_sdk_version="sdk",
        zed_wrapper_version="wrapper",
    )
    _complete_cache(spec, pose_count=1)

    assert not cache_is_valid(spec.path, spec.identity)
