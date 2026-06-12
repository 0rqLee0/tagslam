#!/usr/bin/env python3
"""Convert a TUM trajectory (OpenVINS output) into a rosbag2 of nav_msgs/Odometry.

TagSLAM's rig odom factor needs a runtime nav_msgs/Odometry stream, not a txt
trajectory. On the Table dataset the board is only intermittently visible, so
this odometry is what keeps the rig pose continuous through the gaps; the tags
then pull it back to an absolute frame.

Timestamps are kept at full nanosecond precision (parsed from the string, not
via float) so they line up with the image/IMU clock in the original bag.

Usage:
    python3 tum_to_odom_bag.py table01.txt odom_table01 \\
        --topic /ov_msckf/odomimu --frame odom --child-frame imu
Then play both bags together:
    ros2 bag play table_01 & ros2 bag play odom_table01
"""
import argparse
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry
import rosbag2_py


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tum', help='input TUM trajectory file')
    ap.add_argument('out_bag', help='output rosbag2 directory (created)')
    ap.add_argument('--topic', default='/ov_msckf/odomimu')
    ap.add_argument('--frame', default='odom', help='header.frame_id (match tagslam odom_frame_id)')
    ap.add_argument('--child-frame', default='imu', help='child_frame_id (the body OpenVINS reports)')
    a = ap.parse_args()

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=a.out_bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=a.topic, type='nav_msgs/msg/Odometry', serialization_format='cdr'))

    n = 0
    with open(a.tum) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            sec_str, _, frac = p[0].partition('.')
            sec = int(sec_str)
            nsec = int((frac + '000000000')[:9])
            ts_ns = sec * 1_000_000_000 + nsec

            o = Odometry()
            o.header.stamp.sec = sec
            o.header.stamp.nanosec = nsec
            o.header.frame_id = a.frame
            o.child_frame_id = a.child_frame
            o.pose.pose.position.x = float(p[1])
            o.pose.pose.position.y = float(p[2])
            o.pose.pose.position.z = float(p[3])
            o.pose.pose.orientation.x = float(p[4])
            o.pose.pose.orientation.y = float(p[5])
            o.pose.pose.orientation.z = float(p[6])
            o.pose.pose.orientation.w = float(p[7])
            writer.write(a.topic, serialize_message(o), ts_ns)
            n += 1

    print(f'wrote {n} odometry messages to {a.out_bag} on {a.topic}')


if __name__ == '__main__':
    main()
