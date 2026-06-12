#!/bin/bash
# Run the TagSLAM marker baseline on one Table sequence:
#  (1) offline-generate a drop-free /detector/tags bag from the dataset images
#      (cached under table_eval/tags/),
#  (2) replay that + the OpenVINS-derived odom into tagslam, record rig odom,
#  (3) extract a TUM trajectory (board frame, marker-visible spans).
# Stitching with the full OpenVINS trajectory is a separate step
# (stitch_trajectory.py).
set -e
i=${1:?usage: run_tagslam_table.sh <01..05>}
if [ -z "$TAGSLAM_WS" ]; then
  for c in /opt/tagslam_ws "$HOME/tagslam_ws"; do
    [ -f "$c/install/setup.bash" ] && TAGSLAM_WS="$c/install/setup.bash" && break
  done
fi
: "${TAGSLAM_WS:?set TAGSLAM_WS to the tagslam install/setup.bash}"

source /opt/ros/humble/setup.bash
source "$TAGSLAM_WS"

ROOT=/home/lee/tagslam
CFG=$ROOT/config/table
SCR=$ROOT/scripts
EVAL=$ROOT/table_eval
BAG=/home/lee/Documents/Datasets_VSLAM/table/table_$i
ODOM=$EVAL/odom/odom_$i
TAGS=$EVAL/tags/tag_$i
REC=$EVAL/rec/rec_$i
TRAJ=$EVAL/traj/tagslam_$i.txt
ODOM_TOPIC=/odom/body_rig

mkdir -p "$EVAL/traj" "$EVAL/log" "$EVAL/tags"

# (1) offline, drop-free tag bag (cached)
if [ ! -f "$TAGS/metadata.yaml" ]; then
  echo "[1/4] generating drop-free tag bag (offline detection, slow)..."
  rm -rf "$TAGS"
  python3 $SCR/make_tag_bag.py "$BAG" "$TAGS" \
    --odom /home/lee/Results/board/openvins/table/table$i.txt \
    > $EVAL/log/maketag_$i.log 2>&1
  tail -1 $EVAL/log/maketag_$i.log
else
  echo "[1/4] reusing cached tag bag $TAGS"
fi

# (2) launch tagslam + record rig odom
echo "[2/4] launch tagslam"
cd "$EVAL"; rm -rf out.bag      # tagslam dumps out.bag on optimizer failure; stale dir crashes it
ros2 launch tagslam tagslam.launch.py \
  cameras:=$CFG/cameras.yaml camera_poses:=$CFG/camera_poses.yaml \
  tagslam_config:=$CFG/tagslam.yaml use_sim_time:=true \
  > $EVAL/log/tagslam_$i.log 2>&1 &
PID_TS=$!; sleep 4
rm -rf "$REC"
ros2 bag record -o "$REC" "$ODOM_TOPIC" > $EVAL/log/rec_$i.log 2>&1 &
PID_REC=$!; sleep 2

# (3) merge tags + odom into one time-aligned bag, then replay
echo "[3/4] merge + replay tags + odom"
MERGED=$EVAL/tags/merged_$i
rm -rf "$MERGED"
python3 $SCR/merge_bags.py "$MERGED" "$TAGS" "$ODOM" > $EVAL/log/merge_$i.log 2>&1
tail -1 $EVAL/log/merge_$i.log
ros2 bag play "$MERGED" --clock > $EVAL/log/play_$i.log 2>&1
sleep 3

# (4) stop, extract trajectory
echo "[4/4] stop nodes, extract trajectory"
kill -INT $PID_REC 2>/dev/null || true; sleep 2
kill $PID_TS 2>/dev/null || true; sleep 1
python3 $SCR/bag_topic_to_tum.py "$REC" "$ODOM_TOPIC" "$TRAJ"
echo "done -> $TRAJ"
