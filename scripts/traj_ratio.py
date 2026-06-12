#!/usr/bin/env python3
"""Report the visible-segment (tagslam_0X) length as a fraction of the full
trajectory (odom_0X), by both duration and travelled distance."""
import numpy as np

D = '/home/lee/tagslam/table_eval/traj'
print(f"{'seq':8}{'visible':>18}{'total':>18}{'time%':>8}{'dist%':>8}")
for i in ['01', '02', '03', '04', '05']:
    v = np.loadtxt(f'{D}/tagslam_{i}.txt', comments='#')
    o = np.loadtxt(f'{D}/odom_{i}.txt', comments='#')
    vd, od = v[-1, 0] - v[0, 0], o[-1, 0] - o[0, 0]
    vl = np.sum(np.linalg.norm(np.diff(v[:, 1:4], axis=0), axis=1))
    ol = np.sum(np.linalg.norm(np.diff(o[:, 1:4], axis=0), axis=1))
    print(f"table{i} {vd:7.1f}s/{vl:5.2f}m {od:7.1f}s/{ol:6.2f}m "
          f"{vd/od*100:7.1f}%{vl/ol*100:7.1f}%")
