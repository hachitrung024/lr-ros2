#!/usr/bin/env bash
# Remove Docker resources that are not used by any running container.
# Volumes are intentionally preserved because they may contain recordings,
# calibration data, models, or databases.
set -Eeuo pipefail

ALL_IMAGES=false
case "${1:-}" in
  "") ;;
  --all-images) ALL_IMAGES=true ;;
  -h|--help)
    cat <<'EOF'
Usage: ./docker/cleanup.sh [--all-images]

Default: remove stopped containers, unused networks, dangling images, and
build cache. Docker volumes are never removed.

--all-images also removes every image not referenced by a container.
EOF
    exit 0
    ;;
  *) echo "ERROR: Unknown argument: $1" >&2; exit 2 ;;
esac

command -v docker >/dev/null 2>&1 || {
  echo "ERROR: docker is required but is not installed or not exposed in this environment." >&2
  exit 1
}

echo ">>> Docker usage before cleanup"
docker system df
echo ">>> Stopped containers eligible for cleanup"
docker ps -a --filter status=exited --filter status=created \
  --format 'table {{.ID}}\t{{.Image}}\t{{.Status}}\t{{.Names}}'
echo ">>> Dangling images eligible for cleanup"
docker images --filter dangling=true \
  --format 'table {{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.Size}}\t{{.CreatedSince}}'

PRUNE_ARGS=(--force)
if [ "${ALL_IMAGES}" = true ]; then
  PRUNE_ARGS+=(--all)
fi
docker system prune "${PRUNE_ARGS[@]}"

echo ">>> Docker usage after cleanup"
docker system df
echo ">>> Volumes were preserved."
