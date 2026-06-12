#!/usr/bin/env python3
"""Export only the best continuous board-visible (disambiguated) segment of a
TagSLAM trajectory: finds runs of consecutive board-visible frames (from the
log's `sees tags` lines), picks the one with lowest ATE vs GT, and writes just
those frames (board frame, full pose) to <out>.

Usage:
  export_visible_segment.py <tagslam_traj.txt> <gt.txt> <log> <out.txt> [min_frames]
"""
import sys
import re
import numpy as np


def load_full(p):
    return np.loadtxt(p, comments='#')  # N x 8


def visible_ts(log):
    ts = []
    for line in open(log, errors='ignore'):
        if 'sees tags:' in line:
            tail = line.split('sees tags:')[1].strip()
            if tail:
                m = re.search(r'\[(\d+)\]', line)
                if m:
                    ts.append(int(m.group(1)) * 1e-9)
    return np.array(sorted(ts))


def umeyama(src, dst):
    ms, md = src.mean(0), dst.mean(0)
    S = (dst - md).T @ (src - ms) / len(src)
    U, _, Vt = np.linalg.svd(S)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    return R, md - R @ ms


def ate(sp, gp):
    R, t = umeyama(sp, gp)
    al = (R @ sp.T).T + t
    return np.sqrt(np.mean(np.sum((al - gp) ** 2, axis=1)))


def main():
    traj, gt, log, out = sys.argv[1:5]
    minf = int(sys.argv[5]) if len(sys.argv) > 5 else 5
    gap = 0.3
    data = load_full(traj)
    tt, tp = data[:, 0], data[:, 1:4]
    g = load_full(gt)
    gtt, gtp = g[:, 0], g[:, 1:4]
    vis = visible_ts(log)
    if len(vis) == 0:
        print('  no visible frames')
        return
    segs, cur = [], [vis[0]]
    for x in vis[1:]:
        if x - cur[-1] > gap:
            segs.append((cur[0], cur[-1]))
            cur = []
        cur.append(x)
    segs.append((cur[0], cur[-1]))
    best = None
    for t0, t1 in segs:
        m = (tt >= t0 - 1e-3) & (tt <= t1 + 1e-3)
        if m.sum() < minf:
            continue
        st = tt[m]
        if st[0] < gtt[0] or st[-1] > gtt[-1]:
            continue
        gp = np.column_stack([np.interp(st, gtt, gtp[:, k]) for k in range(3)])
        a = ate(tp[m], gp)
        if best is None or a < best[1]:
            best = (m, a, t0, t1)
    if best is None:
        print(f'  no segment >= {minf} frames')
        return
    m, a, t0, t1 = best
    np.savetxt(out, data[m], fmt='%.9f',
               header='timestamp tx ty tz qx qy qz qw')
    print(f'  exported {int(m.sum())} frames, {t1-t0:.1f}s, ATE={a*1000:.1f}mm')


if __name__ == '__main__':
    main()
