#!/usr/bin/env python3
"""Evaluate TagSLAM only on continuous board-visible (successfully disambiguated)
segments. Isolates runs of consecutive board-visible frames (from the tagslam
log's `sees tags` lines), SE3-aligns each long-enough run to GT, reports ATE.

Usage:
  eval_visible_segments.py <tagslam_traj.txt> <gt.txt> <tagslam_log> [gap_s] [min_frames]
"""
import sys
import re
import numpy as np


def load_tum(p):
    d = np.loadtxt(p, comments='#')
    return d[:, 0], d[:, 1:4]


def visible_ts(logpath):
    ts = []
    for line in open(logpath, errors='ignore'):
        if 'sees tags:' in line:
            tail = line.split('sees tags:')[1].strip()
            if tail:
                m = re.search(r'\[(\d+)\]', line)
                if m:
                    ts.append(int(m.group(1)) * 1e-9)
    return np.array(sorted(ts))


def umeyama_se3(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    S = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(S)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    return R, mu_d - R @ mu_s


def seg_ate(seg_p, gt_p):
    R, t = umeyama_se3(seg_p, gt_p)
    al = (R @ seg_p.T).T + t
    return np.sqrt(np.mean(np.sum((al - gt_p) ** 2, axis=1)))


def main():
    traj, gt, log = sys.argv[1:4]
    gap = float(sys.argv[4]) if len(sys.argv) > 4 else 0.3
    minf = int(sys.argv[5]) if len(sys.argv) > 5 else 30
    tt, tp = load_tum(traj)
    gtt, gtp = load_tum(gt)
    vis = visible_ts(log)
    if len(vis) == 0:
        print('  no visible frames in log')
        return

    segs, cur = [], [vis[0]]
    for x in vis[1:]:
        if x - cur[-1] > gap:
            segs.append((cur[0], cur[-1]))
            cur = []
        cur.append(x)
    segs.append((cur[0], cur[-1]))

    results = []
    for t0, t1 in segs:
        m = (tt >= t0 - 1e-3) & (tt <= t1 + 1e-3)
        if m.sum() < minf:
            continue
        st, sp = tt[m], tp[m]
        if st[0] < gtt[0] or st[-1] > gtt[-1]:
            continue
        gp = np.column_stack([np.interp(st, gtt, gtp[:, k]) for k in range(3)])
        results.append((t1 - t0, int(m.sum()), seg_ate(sp, gp), t0, t1))

    if not results:
        print(f'  no continuous visible segment >= {minf} frames')
        return
    for dur, n, a, t0, t1 in sorted(results, key=lambda r: -r[0]):
        print(f'  {dur:5.1f}s {n:4d}f  ATE={a*1000:7.1f}mm  [{t0:.1f}..{t1:.1f}]')
    best = min(results, key=lambda r: r[2])
    print(f'  BEST: {best[1]} frames / {best[0]:.1f}s  ATE={best[2]*1000:.1f}mm')


if __name__ == '__main__':
    main()
