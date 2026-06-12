#!/usr/bin/env python3
"""Stitch an intermittent TagSLAM trajectory with a continuous OpenVINS one.

TagSLAM only produces poses while the checkerboard is visible (board frame,
drift-free, marker-corrected). OpenVINS is continuous but in its own frame and
drifts. We produce one continuous trajectory in the board frame:
  - board visible   -> TagSLAM poses
  - board invisible -> OpenVINS, rigidly aligned to the board frame

Alignment ov->board is a SE(3) fit (rotation+translation, NO scale) on the
overlapping timestamps (Umeyama without scaling), so OpenVINS keeps its metric
scale and is only rotated/translated into the board frame.

Usage:
  stitch_trajectory.py tagslam_01.txt openvins_table01.txt stitched_01.txt
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation


def load_tum(path):
    d = np.loadtxt(path, comments='#')
    return d[:, 0], d[:, 1:4], d[:, 4:8]  # t, xyz, quat(xyzw)


def umeyama_se3(src, dst):
    """R, t such that dst ~= (R @ src.T).T + t, no scaling."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(cov)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    t = mu_d - R @ mu_s
    return R, t


def main():
    tag_f, ov_f, out_f = sys.argv[1:4]
    tt, tp, tq = load_tum(tag_f)
    ot, op, oq = load_tum(ov_f)

    # overlap window = tag span clipped to ov range
    lo, hi = max(tt.min(), ot.min()), min(tt.max(), ot.max())
    m = (tt >= lo) & (tt <= hi)
    if m.sum() < 3:
        raise SystemExit(f'too few overlapping poses ({m.sum()}) to align')

    # interpolate ov positions at the tag timestamps within the overlap
    ov_at = np.column_stack([np.interp(tt[m], ot, op[:, k]) for k in range(3)])
    R, t = umeyama_se3(ov_at, tp[m])  # ov -> board

    # bring the full ov trajectory into the board frame
    op_b = (R @ op.T).T + t
    oq_b = (Rotation.from_matrix(R) * Rotation.from_quat(oq)).as_quat()

    # stitch: ov samples outside the tag span + all tag samples, time-sorted
    span_lo, span_hi = tt.min(), tt.max()
    keep = (ot < span_lo) | (ot > span_hi)
    T = np.concatenate([ot[keep], tt])
    P = np.vstack([op_b[keep], tp])
    Q = np.vstack([oq_b[keep], tq])
    order = np.argsort(T)
    out = np.column_stack([T, P, Q])[order]
    np.savetxt(out_f, out, fmt='%.9f',
               header='timestamp tx ty tz qx qy qz qw')
    print(f'stitched {int(keep.sum())} ov + {len(tt)} tag = {len(T)} poses '
          f'(overlap {int(m.sum())} for alignment) -> {out_f}')


if __name__ == '__main__':
    main()
