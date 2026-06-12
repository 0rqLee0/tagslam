#!/bin/bash
# Build the image (first run) and enter a container with all paths mounted.
# Host stays clean — only Docker is used.
#
# Inside the container, once:   bash docker/build_ws.sh
# Then per sequence:            bash scripts/run_tagslam_table.sh 01
set -e
IMG=tagslam-table
ROOT=/home/lee/tagslam
WS_HOST=/home/lee/tagslam/.docker_ws         # persistent colcon build output (in-project)

if ! docker image inspect "$IMG" >/dev/null 2>&1; then
  echo ">> building image $IMG (first time: installs GTSAM/apriltag/flex_sync/...)"
  docker build -f "$ROOT/docker/Dockerfile" -t "$IMG" "$ROOT"
fi

mkdir -p "$WS_HOST"
docker run -it --rm \
  --net host --ipc host \
  -e TAGSLAM_WS=/opt/tagslam_ws/install/setup.bash \
  -v "$ROOT":/home/lee/tagslam \
  -v "$WS_HOST":/opt/tagslam_ws \
  -v /home/lee/Documents/Datasets_VSLAM/table:/home/lee/Documents/Datasets_VSLAM/table:ro \
  -v /home/lee/Results/board/openvins/table:/home/lee/Results/board/openvins/table:ro \
  "$IMG" bash
