#!/usr/bin/env python3
"""Merge several rosbag2 bags into one, ordered by record timestamp, so topics
from different bags (e.g. /detector/tags and /ov_msckf/odomimu) replay on a
single aligned clock. Two separate `ros2 bag play` with different start times
desync, which breaks TagSLAM's tag/odom synchronization.

Usage: merge_bags.py <out_bag> <in_bag1> <in_bag2> [...]
"""
import sys
import rosbag2_py

out = sys.argv[1]
ins = sys.argv[2:]

writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=out, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
created = set()
msgs = []
for b in ins:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=b, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    for t in r.get_all_topics_and_types():
        if t.name not in created:
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=t.name, type=t.type, serialization_format='cdr'))
            created.add(t.name)
    while r.has_next():
        topic, data, ts = r.read_next()
        msgs.append((ts, topic, data))

msgs.sort(key=lambda x: x[0])
for ts, topic, data in msgs:
    writer.write(topic, data, ts)
print(f'merged {len(ins)} bags -> {out} ({len(msgs)} msgs, {len(created)} topics)')
