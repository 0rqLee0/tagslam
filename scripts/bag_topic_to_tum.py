#!/usr/bin/env python3
"""Extract a pose topic from a ROS2 bag into a TUM trajectory file.

Works for geometry_msgs/PoseStamped (e.g. the /d455/rigidbody mocap ground
truth) and nav_msgs/Odometry (e.g. tagslam's /tagslam/odom/rig output), so the
same script produces both the GT and the estimate for evo_ape.

Timestamps use the message header stamp so GT and estimate share the bag clock.

Usage:
    python3 bag_topic_to_tum.py table_01 /d455/rigidbody gt_01.txt
    python3 bag_topic_to_tum.py tagslam_rec_01 /tagslam/odom/rig tagslam_01.txt
"""
import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='rosbag2 directory')
    ap.add_argument('topic', help='pose topic (PoseStamped or Odometry)')
    ap.add_argument('out', help='output TUM file')
    a = ap.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=a.bag, storage_id='sqlite3'),
                ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if a.topic not in types:
        raise SystemExit(f'topic {a.topic} not in bag; available: {list(types)}')
    msg_type = get_message(types[a.topic])
    reader.set_filter(StorageFilter(topics=[a.topic]))  # skip image data, read only this topic

    n = 0
    with open(a.out, 'w') as f:
        f.write('# timestamp tx ty tz qx qy qz qw\n')
        while reader.has_next():
            tn, data, _ = reader.read_next()
            if tn != a.topic:
                continue
            msg = deserialize_message(data, msg_type)
            pose = msg.pose.pose if hasattr(msg.pose, 'pose') else msg.pose
            st = msg.header.stamp
            t = st.sec + st.nanosec * 1e-9
            p, q = pose.position, pose.orientation
            f.write(f'{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} '
                    f'{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')
            n += 1
    print(f'wrote {n} poses from {a.topic} to {a.out}')


if __name__ == '__main__':
    main()
