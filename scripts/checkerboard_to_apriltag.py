#!/usr/bin/env python3
"""Bridge a checkerboard into virtual AprilTag detections for TagSLAM.

TagSLAM only consumes apriltag_msgs/AprilTagDetectionArray. To use it as a
fiducial-marker baseline on a checkerboard dataset (e.g. ov_plane's Table), we
detect the board, carve it into a grid of *square* virtual tags, and publish
their four image corners in TagSLAM's object-corner order:

    corner0 = (-s/2,-s/2)  corner1 = (s/2,-s/2)  corner2 = (s/2,s/2)  corner3 = (-s/2,s/2)

i.e. board-frame  (cmin,rmin) -> (cmax,rmin) -> (cmax,rmax) -> (cmin,rmax).
The matching tag poses are written in the tagslam.yaml board body.

Orientation disambiguation
--------------------------
A checkerboard carries no orientation code, and a square board (rows == cols)
has a 4-fold rotation ambiguity, so OpenCV's corner ordering can flip between
frames or when the board re-enters view. We resolve it per frame by PnP: the
four 90-degree rotations are tried, and the one whose board pose is closest to
the previous frame is kept (temporal consistency). On the first sighting a
canonical orientation is chosen (board x-axis pointing right in the image).
Long gaps in visibility can still pick the wrong flip on re-entry -- inspect
the published ids visually if TagSLAM diverges.
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point


def rotvec_angle(r1, r2):
    R1, _ = cv2.Rodrigues(r1)
    R2, _ = cv2.Rodrigues(r2)
    dR = R1.T @ R2
    return float(np.arccos(np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)))


class CheckerboardToApriltag(Node):
    def __init__(self):
        super().__init__('checkerboard_to_apriltag')
        self.cols = int(self.declare_parameter('pattern_cols', 6).value)
        self.rows = int(self.declare_parameter('pattern_rows', 6).value)
        self.s = float(self.declare_parameter('square_size', 0.088).value)
        # each virtual tag spans `sub` x `sub` squares (must divide the board
        # into non-overlapping blocks: cols, rows divisible by (sub+1)-spaced starts)
        self.sub = int(self.declare_parameter('sub_squares', 2).value)
        img_topic = self.declare_parameter('image_topic', '/d455/color/image_raw').value
        info_topic = self.declare_parameter('camera_info_topic', '/d455/color/camera_info').value
        out_topic = self.declare_parameter('output_topic', '/detector/tags').value

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.prev_rvec = None

        # canonical object points, linear index k = r*cols + c -> (c*s, r*s, 0)
        cs, rs = np.meshgrid(np.arange(self.cols), np.arange(self.rows))  # (rows, cols)
        self.objp = np.zeros((self.rows * self.cols, 3), np.float64)
        self.objp[:, 0] = cs.reshape(-1) * self.s
        self.objp[:, 1] = rs.reshape(-1) * self.s

        self.tags = self._build_tags()

        self.create_subscription(CameraInfo, info_topic, self.info_cb, 10)
        self.create_subscription(Image, img_topic, self.image_cb, 10)
        self.pub = self.create_publisher(AprilTagDetectionArray, out_topic, 10)
        self.get_logger().info(
            f'checkerboard_to_apriltag: {self.cols}x{self.rows} inner corners, '
            f'square={self.s} m, {len(self.tags)} virtual tags of size '
            f'{self.sub * self.s:.3f} m')

    def _build_tags(self):
        """Non-overlapping square blocks; each consumes (sub+1) inner corners."""
        tags, tid, stride = [], 0, self.sub + 1
        for r0 in range(0, self.rows - self.sub, stride):
            for c0 in range(0, self.cols - self.sub, stride):
                cmin, cmax, rmin, rmax = c0, c0 + self.sub, r0, r0 + self.sub
                corners = [(cmin, rmin), (cmax, rmin), (cmax, rmax), (cmin, rmax)]
                tags.append((tid, corners))
                tid += 1
        return tags

    def info_cb(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(
                f'camera_info: fx={self.K[0,0]:.1f} fy={self.K[1,1]:.1f} '
                f'cx={self.K[0,2]:.1f} cy={self.K[1,2]:.1f}, {len(self.D)} dist coeffs')

    def _detect(self, gray):
        flags = (cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_EXHAUSTIVE +
                 cv2.CALIB_CB_ACCURACY)
        try:
            found, corners = cv2.findChessboardCornersSB(gray, (self.cols, self.rows), flags=flags)
        except AttributeError:
            found, corners = cv2.findChessboardCorners(gray, (self.cols, self.rows))
            if found:
                cv2.cornerSubPix(
                    gray, corners, (5, 5), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        return found, corners

    def image_cb(self, msg):
        if self.K is None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = self._detect(gray)

        out = AprilTagDetectionArray()
        out.header = msg.header
        if not found:
            self.pub.publish(out)  # empty: board not (fully) visible this frame
            return

        grid = corners.reshape(self.rows, self.cols, 2).astype(np.float64)
        best = None  # (score, rotated_grid, rvec)
        for d in range(4):
            g = np.rot90(grid, d)  # square board keeps (rows, cols, 2)
            if g.shape[:2] != (self.rows, self.cols):
                continue
            ok, rvec, tvec = cv2.solvePnP(
                self.objp, g.reshape(-1, 2), self.K, self.D, flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                continue
            if self.prev_rvec is not None:
                score = rotvec_angle(self.prev_rvec, rvec)
            else:
                R, _ = cv2.Rodrigues(rvec)
                score = -float(R[0, 0])  # prefer board x-axis pointing right
            if best is None or score < best[0]:
                best = (score, g, rvec)

        if best is None:
            self.pub.publish(out)
            return
        _, g, rvec = best
        self.prev_rvec = rvec

        for tid, block in self.tags:
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
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CheckerboardToApriltag()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
