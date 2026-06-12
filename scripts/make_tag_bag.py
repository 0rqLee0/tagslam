#!/usr/bin/env python3
"""Offline checkerboard -> virtual AprilTag detections, with OpenVINS-aided
orientation disambiguation.

A 6x6 (symmetric) checkerboard carries no orientation code, so PnP has a 4-fold
flip ambiguity; with the board only intermittently visible, temporal tracking
fails on re-entry. We instead pick the flip whose board orientation is
consistent with the camera pose predicted by the OpenVINS odometry, keeping the
tag->physical-corner mapping consistent across the whole sequence (the
equivalent of the IMU-based disambiguation the proposed method uses). Frames
outside the odom time range fall back to temporal consistency.

Usage:
  make_tag_bag.py <in_bag> <out_tag_bag> --odom <openvins_tum.txt>
                  [--image-topic T] [--info-topic T]
"""
import argparse
import numpy as np
import cv2
import rosbag2_py
from scipy.spatial.transform import Rotation, Slerp
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point

COLS, ROWS, S, SUB = 6, 6, 0.088, 2

# kalibr T_imu_cam: camera pose in the IMU frame (rotation cam->imu). Only used
# to predict orientation for flip disambiguation, so approximate is fine.
R_CAM_IMU = np.array([
    [0.9997296168385974, -0.0098837220325796, -0.0210476900047239],
    [0.0102479554752026, 0.9997983742911989, 0.0172681838435744],
    [0.0208727723201907, -0.0174792106074593, 0.9996293335893115]])


def build_tags(cols, rows, sub):
    tags, tid, stride = [], 0, sub + 1
    for r0 in range(0, rows - sub, stride):
        for c0 in range(0, cols - sub, stride):
            cmin, cmax, rmin, rmax = c0, c0 + sub, r0, r0 + sub
            tags.append((tid, [(cmin, rmin), (cmax, rmin), (cmax, rmax), (cmin, rmax)]))
            tid += 1
    return tags


def rot_angle(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return float(np.arccos(np.clip(c, -1, 1)))


def reader(bag, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('in_bag')
    ap.add_argument('out_bag')
    ap.add_argument('--odom', required=True, help='OpenVINS TUM trajectory (IMU in world)')
    ap.add_argument('--image-topic', default='/d455/color/image_raw')
    ap.add_argument('--info-topic', default='/d455/color/camera_info')
    ap.add_argument('--out-topic', default='/detector/tags')
    a = ap.parse_args()

    K = D = None
    ri = reader(a.in_bag, a.info_topic)
    while ri.has_next():
        _, d, _ = ri.read_next()
        m = deserialize_message(d, CameraInfo)
        K = np.array(m.k, dtype=np.float64).reshape(3, 3)
        D = np.array(m.d, dtype=np.float64)
        break
    if K is None:
        raise SystemExit(f'no camera_info on {a.info_topic}')

    od = np.loadtxt(a.odom, comments='#')
    odom_t = od[:, 0]
    slerp = Slerp(odom_t, Rotation.from_quat(od[:, 4:8]))  # IMU->World

    cs, rs = np.meshgrid(np.arange(COLS), np.arange(ROWS))
    objp = np.zeros((ROWS * COLS, 3), np.float64)
    objp[:, 0] = cs.reshape(-1) * S
    objp[:, 1] = rs.reshape(-1) * S
    tags = build_tags(COLS, ROWS, SUB)

    wr = rosbag2_py.SequentialWriter()
    wr.open(rosbag2_py.StorageOptions(uri=a.out_bag, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('', ''))
    wr.create_topic(rosbag2_py.TopicMetadata(
        name=a.out_topic, type='apriltag_msgs/msg/AprilTagDetectionArray',
        serialization_format='cdr'))

    br = CvBridge()
    flags = cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE + cv2.CALIB_CB_ACCURACY
    R_board_world = None   # board orientation in world, fixed once initialized
    prev_R = None          # fallback outside odom range
    total = found = odom_used = 0
    rim = reader(a.in_bag, a.image_topic)
    while rim.has_next():
        _, data, ts = rim.read_next()
        total += 1
        msg = deserialize_message(data, Image)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gray = cv2.cvtColor(br.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCornersSB(gray, (COLS, ROWS), flags=flags)

        out = AprilTagDetectionArray()
        out.header = msg.header
        if ok:
            grid = corners.reshape(ROWS, COLS, 2).astype(np.float64)
            cands = []  # (grid_d, R_pnp board->cam)
            for dd in range(4):
                g = np.rot90(grid, dd)
                if g.shape[:2] != (ROWS, COLS):
                    continue
                # IPPE returns both planar-PnP solutions, resolving the in-image
                # (IPPE) flip on top of the 4 rot90 board-symmetry flips
                retval, rvecs, _, reproj = cv2.solvePnPGeneric(
                    objp, g.reshape(-1, 2), K, D, flags=cv2.SOLVEPNP_IPPE)
                for k, rvec in enumerate(rvecs):
                    e = float(reproj[k]) if reproj is not None else 0.0
                    cands.append((g, cv2.Rodrigues(rvec)[0], e))
            if cands:
                R_cw = None
                if odom_t[0] <= t <= odom_t[-1]:
                    R_cw = slerp([t])[0].as_matrix() @ R_CAM_IMU  # cam->world
                if R_board_world is None and R_cw is not None:
                    _, R0, _ = min(cands, key=lambda c: c[2])     # lowest-reproj candidate
                    R_board_world = R_cw @ R0
                g = R_sel = None
                if R_cw is not None and R_board_world is not None:
                    R_pred = R_cw.T @ R_board_world               # board in cam (predicted)
                    scored = sorted(cands, key=lambda c: rot_angle(c[1], R_pred))
                    best_a = rot_angle(scored[0][1], R_pred)
                    second_a = rot_angle(scored[1][1], R_pred) if len(scored) > 1 else 9.9
                    # keep only confidently disambiguated frames: good match AND
                    # clearly separated from the next-best candidate
                    if best_a < 0.5 and (second_a - best_a) > 0.4:
                        g, R_sel, _ = scored[0]
                        odom_used += 1
                elif prev_R is not None:
                    g, R_sel, _ = min(cands, key=lambda c: rot_angle(c[1], prev_R))
                else:
                    g, R_sel, _ = cands[0]
                if g is not None:
                    prev_R = R_sel
                    found += 1
                    for tid, block in tags:
                        det = AprilTagDetection()
                        det.id = tid
                        det.hamming = 0
                        det.family = 'checkerboard'
                        pts = [Point(x=float(g[r, c, 0]), y=float(g[r, c, 1])) for (c, r) in block]
                        det.corners = pts
                        det.centre = Point(
                            x=float(np.mean([p.x for p in pts])),
                            y=float(np.mean([p.y for p in pts])))
                        out.detections.append(det)
        wr.write(a.out_topic, serialize_message(out), ts)

    print(f'images={total} board_found={found} odom_disambiguated={odom_used} '
          f'-> {a.out_bag} ({a.out_topic})')


if __name__ == '__main__':
    main()
