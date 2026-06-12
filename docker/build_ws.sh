#!/bin/bash
# Run INSIDE the container, once, to compile tagslam into the mounted ws volume
# (/opt/tagslam_ws -> persisted on the host at ~/tagslam_ws_docker).
set -e
source /opt/ros/humble/setup.bash
mkdir -p /opt/tagslam_ws/src
ln -sfn /home/lee/tagslam /opt/tagslam_ws/src/tagslam
cd /opt/tagslam_ws
colcon build --symlink-install
echo "OK -> source /opt/tagslam_ws/install/setup.bash"
