#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
wrapper_dir="${workspace_dir}/src/zed-ros2-wrapper"
patch_file="${workspace_dir}/patches/zed-ros2-wrapper/enable-svo-realtime-pause.patch"
patch_applied=false

cleanup() {
  if [[ "${patch_applied}" == true ]]; then
    if ! git -C "${wrapper_dir}" apply --reverse --check "${patch_file}"; then
      echo "ERROR: Cannot restore the zed-ros2-wrapper source automatically." >&2
      echo "The applied patch is still present in: ${wrapper_dir}" >&2
      return 1
    fi
    git -C "${wrapper_dir}" apply --reverse "${patch_file}"
  fi
}

trap cleanup EXIT

if [[ ! -d "${wrapper_dir}/.git" && ! -f "${wrapper_dir}/.git" ]]; then
  echo "ERROR: zed-ros2-wrapper submodule is not initialized." >&2
  echo "Run: git submodule update --init --recursive" >&2
  exit 1
fi

if ! git -C "${wrapper_dir}" diff --quiet -- \
  zed_components/src/tools/include/sl_tools.hpp \
  zed_components/src/zed_camera_one/src/zed_camera_one_component_main.cpp; then
  echo "ERROR: The ZED wrapper files touched by the patch already have local changes." >&2
  echo "Restore or save those changes before running this script." >&2
  exit 1
fi

if ! git -C "${wrapper_dir}" apply --check "${patch_file}"; then
  echo "ERROR: The realtime-pause patch is incompatible with the checked-out wrapper commit:" >&2
  git -C "${wrapper_dir}" rev-parse HEAD >&2
  exit 1
fi

git -C "${wrapper_dir}" apply "${patch_file}"
patch_applied=true

cd "${workspace_dir}"
colcon build --symlink-install --packages-select zed_components "$@"
