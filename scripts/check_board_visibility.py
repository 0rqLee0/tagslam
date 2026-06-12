#!/usr/bin/env python3
"""Offline-count how many frames the checkerboard is actually visible in a bag
(no dropped frames), to distinguish a data limitation from bridge frame-drop.

Usage: check_board_visibility.py <bag_dir> [image_topic]
"""
import sys
import numpy as np
import cv2
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bag = sys.argv[1]
topic = sys.argv[2] if len(sys.argv) > 2 else '/d455/color/image_raw'

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
       rosbag2_py.ConverterOptions('', ''))
r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
br = CvBridge()
flags = cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY

found, total, t0 = [], 0, None
while r.has_next():
    _, d, _ = r.read_next()
    total += 1
    m = deserialize_message(d, Image)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    if t0 is None:
        t0 = t
    gray = cv2.cvtColor(br.imgmsg_to_cv2(m, 'bgr8'), cv2.COLOR_BGR2GRAY)
    ok, _ = cv2.findChessboardCornersSB(gray, (6, 6), flags=flags)
    if ok:
        found.append(t - t0)

print(f'total={total} visible={len(found)} ({100*len(found)/max(total,1):.0f}%)')
if found:
    f = np.array(found)
    print(f'visible span: {f.min():.1f}..{f.max():.1f}s (bag ~{(t-t0):.0f}s)')
    print('visible frames per 10s bin:',
          list(np.histogram(f, bins=range(0, int(t - t0) + 11, 10))[0]))
