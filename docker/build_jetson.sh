#!/usr/bin/env bash
# Build the LR Jetson environment on top of the existing ZED Jetson image.
#
# JetPack selects the Ubuntu/CUDA stack. LR sources and data are not copied
# into the image; users mount and build the repository at runtime.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ZED_BUILDER="${REPO_ROOT}/src/zed-ros2-wrapper/docker/build_jetson.sh"

BASE_IMAGE=""
OUTPUT_IMAGE=""
JETSON_OS=""
TORCH_INDEX_URL=""
ZED_ARGS=()

usage() {
  cat <<'EOF'
Usage: ./docker/build_jetson.sh [ZED options] [LR options]

ZED options (forwarded to src/zed-ros2-wrapper/docker/build_jetson.sh):
  --ros-distro <distro>
  --os <jpX.Y.Z|l4t-rXX.X>
  --sdk <X.Y.Z|latest>
  --sdk-url <URL|path>

LR options:
  --base-image <image>       Reuse an existing ZED Jetson image.
  --tag <image>              Override the output LR image tag.
  --torch-index-url <URL>    Override the JetPack PyTorch package index.
  -h, --help                 Show this help.

Defaults target JetPack 6.2.2, ROS 2 Humble, and the latest compatible ZED SDK.

Examples:
  ./docker/build_jetson.sh --ros-distro humble --os jp6.2.2 --sdk latest
  ./docker/build_jetson.sh --ros-distro jazzy --os jp7.1.0 --sdk latest
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
    --torch-index-url)
      [ "$#" -ge 2 ] || { echo "ERROR: --torch-index-url requires a value." >&2; exit 2; }
      TORCH_INDEX_URL="$2"
      shift 2
      ;;
    --os)
      [ "$#" -ge 2 ] || { echo "ERROR: --os requires a value." >&2; exit 2; }
      JETSON_OS="$2"
      ZED_ARGS+=("$1" "$2")
      shift 2
      ;;
    --ros-distro|--sdk|--sdk-url)
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
  echo "ERROR: ZED Jetson builder is not executable: ${ZED_BUILDER}" >&2
  exit 1
}

if [ -z "${BASE_IMAGE}" ]; then
  BUILD_LOG="$(mktemp)"
  cleanup() {
    rm -f -- "${BUILD_LOG}"
  }
  trap cleanup EXIT

  echo ">>> Building/reusing the standard ZED Jetson image"
  "${ZED_BUILDER}" "${ZED_ARGS[@]}" | tee "${BUILD_LOG}"
  BASE_IMAGE="$(sed -n 's/^>>> Built image: //p' "${BUILD_LOG}" | tail -n 1)"
  [ -n "${BASE_IMAGE}" ] || {
    echo "ERROR: Could not determine the image tag produced by the ZED Jetson builder." >&2
    exit 1
  }
fi

# Infer the JetPack generation from --os, or from a reused standard image tag.
PLATFORM_HINT="${JETSON_OS:-${BASE_IMAGE}}"
case "${PLATFORM_HINT}" in
  *jp6.1*|*jp6.2*|*r36.4*|*r36.5*)
    JETPACK_SERIES=6
    TORCH_PROFILE=jp6-cu126
    ;;
  *jp7.1*|*r38.4*)
    JETPACK_SERIES=7
    TORCH_PROFILE=jp7-cu132
    ;;
  *jp6*|*r36*|*jp7*|*r38*)
    if [ -n "${TORCH_INDEX_URL}" ]; then
      JETPACK_SERIES=custom
      TORCH_PROFILE=custom
    else
      echo "ERROR: No default PyTorch index is defined for '${PLATFORM_HINT}'." >&2
      echo "Pass --torch-index-url with a compatible JetPack repository." >&2
      exit 2
    fi
    ;;
  *)
    if [ -n "${TORCH_INDEX_URL}" ]; then
      JETPACK_SERIES=custom
      TORCH_PROFILE=custom
    else
      echo "ERROR: Cannot infer JetPack generation from '${PLATFORM_HINT}'." >&2
      echo "Pass --os jp6.2.2/jp7.1.0 or provide --torch-index-url." >&2
      exit 2
    fi
    ;;
esac

if [ -z "${TORCH_INDEX_URL}" ]; then
  case "${TORCH_PROFILE}" in
    jp6-cu126) TORCH_INDEX_URL="https://pypi.jetson-ai-lab.io/jp6/cu126" ;;
    jp7-cu132) TORCH_INDEX_URL="https://download.pytorch.org/whl/cu132" ;;
  esac
fi

# JetPack 6's CUDA-enabled ARM64 repository has a known compatible pair.
# JetPack 7 uses the matching latest pair from PyTorch's CUDA 13.2 index.
case "${TORCH_PROFILE}" in
  jp6-cu126)
    TORCH_SPEC="torch==2.8.0"
    TORCHVISION_SPEC="torchvision==0.23.0"
    ;;
  *)
    TORCH_SPEC="torch>=2.8,<3"
    TORCHVISION_SPEC="torchvision"
    ;;
esac

if [ "${JETPACK_SERIES}" = "custom" ]; then
  JETPACK_LABEL=custom
else
  JETPACK_LABEL="${JETPACK_SERIES}.x"
fi

if [ -z "${OUTPUT_IMAGE}" ]; then
  case "${BASE_IMAGE}" in
    zed_ros2_*) OUTPUT_IMAGE="lr_${BASE_IMAGE}" ;;
    *) OUTPUT_IMAGE="lr_ros2_jetson:latest" ;;
  esac
fi

echo "============================================================"
echo " Base image  : ${BASE_IMAGE}"
echo " JetPack     : ${JETPACK_LABEL}"
echo " Torch index : ${TORCH_INDEX_URL}"
echo " PyTorch     : ${TORCH_SPEC} / ${TORCHVISION_SPEC}"
echo " Output tag  : ${OUTPUT_IMAGE}"
echo " Context     : ${SCRIPT_DIR}"
echo "============================================================"

docker build \
  --tag "${OUTPUT_IMAGE}" \
  --build-arg "BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "LR_PLATFORM=jetson" \
  --build-arg "TORCH_INDEX_URL=${TORCH_INDEX_URL}" \
  --build-arg "TORCH_SPEC=${TORCH_SPEC}" \
  --build-arg "TORCHVISION_SPEC=${TORCHVISION_SPEC}" \
  --file "${SCRIPT_DIR}/Dockerfile" \
  "${SCRIPT_DIR}"

echo ">>> Built LR Jetson environment image: ${OUTPUT_IMAGE}"
