#!/usr/bin/env bash
# Build the LR desktop environment on top of the existing ZED wrapper image.
#
# All normal ZED builder arguments are forwarded unchanged. The ZED image is
# built first (and normally comes entirely from Docker's cache), then the LR
# build/runtime dependencies are added. LR sources are mounted at runtime.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ZED_BUILDER="${REPO_ROOT}/src/zed-ros2-wrapper/docker/build_desktop.sh"

BASE_IMAGE=""
OUTPUT_IMAGE=""
ZED_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./docker/build_desktop.sh [ZED options] [LR options]

ZED options (forwarded to src/zed-ros2-wrapper/docker/build_desktop.sh):
  --ros-distro <distro>
  --os <ubuntu-XX.XX>
  --sdk <X.Y.Z|latest>
  --cuda <XX.X>
  --sdk-url <URL|path>

LR options:
  --base-image <image>  Reuse an existing ZED wrapper image without rebuilding it.
  --tag <image>         Override the output LR image tag.
  -h, --help            Show this help.

Example for this Humble workspace:
  ./docker/build_desktop.sh --ros-distro humble --os ubuntu-22.04 \
    --sdk 5.4.1 --cuda 12.8
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --base-image)
      [ "$#" -ge 2 ] || { echo "ERROR: --base-image requires a value." >&2; exit 2; }
      BASE_IMAGE="$2"
      shift 2
      ;;
    --tag)
      [ "$#" -ge 2 ] || { echo "ERROR: --tag requires a value." >&2; exit 2; }
      OUTPUT_IMAGE="$2"
      shift 2
      ;;
    --ros-distro|--os|--sdk|--cuda|--sdk-url)
      [ "$#" -ge 2 ] || { echo "ERROR: $1 requires a value." >&2; exit 2; }
      ZED_ARGS+=("$1" "$2")
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: Unknown argument: $1 (use --help)." >&2
      exit 2
      ;;
  esac
done

command -v docker >/dev/null 2>&1 || {
  echo "ERROR: docker is required but is not installed or not exposed in this environment." >&2
  exit 1
}
[ -x "${ZED_BUILDER}" ] || {
  echo "ERROR: ZED builder is not executable: ${ZED_BUILDER}" >&2
  exit 1
}

if [ -z "${BASE_IMAGE}" ]; then
  BUILD_LOG="$(mktemp)"
  cleanup() {
    rm -f -- "${BUILD_LOG}"
  }
  trap cleanup EXIT

  echo ">>> Building/reusing the standard ZED wrapper image"
  "${ZED_BUILDER}" "${ZED_ARGS[@]}" | tee "${BUILD_LOG}"
  BASE_IMAGE="$(sed -n 's/^>>> Built image: //p' "${BUILD_LOG}" | tail -n 1)"
  [ -n "${BASE_IMAGE}" ] || {
    echo "ERROR: Could not determine the image tag produced by the ZED builder." >&2
    exit 1
  }
fi

if [ -z "${OUTPUT_IMAGE}" ]; then
  case "${BASE_IMAGE}" in
    zed_ros2_desktop_*) OUTPUT_IMAGE="lr_${BASE_IMAGE}" ;;
    *) OUTPUT_IMAGE="lr_ros2_desktop:latest" ;;
  esac
fi

echo "============================================================"
echo " Base image : ${BASE_IMAGE}"
echo " Output tag : ${OUTPUT_IMAGE}"
echo " Context    : ${SCRIPT_DIR}"
echo "============================================================"

docker build \
  --tag "${OUTPUT_IMAGE}" \
  --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "LR_PLATFORM=desktop" \
  --file "${SCRIPT_DIR}/Dockerfile" \
  "${SCRIPT_DIR}"

echo ">>> Built LR environment image: ${OUTPUT_IMAGE}"
