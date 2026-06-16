#!/usr/bin/env python3
"""
@file   red_box_detector.py
@brief  Gazebo 仿真相机红色方块识别 — HSV + solvePnP + TF 坐标变换
@version 3.0
@date   2026-06-04

数据流：
    /camera/camera_sensor/image_raw (RGB)
        → HSV 阈值提取红色掩码 → 形态学去噪 → findContours
        → minAreaRect 获取旋转矩形四角点
        → solvePnP 解算相机坐标系位姿 tvec = (tx, ty, tz)
        → TF lookup_transform(base_link ← camera_optical_frame)
        → p_base = R @ p_cam + t  （相机坐标 → 基座坐标）
        → 发布 /red_detector/feature (PointStamped: x_norm, y_norm, depth)
        → 绿色旋转框 + 坐标轴 + 多行标注 + cv2.imshow
        → 发布 /red_detector/image 供 rqt 查看

  运行方式：
      ros2 launch robot_arm_bringup gazebo.launch.py controller:=ibvs_control  # 随仿真一起启动
      ros2 launch robot_arm_bringup red_box_detect.launch.py                   # 单独启动（纯检测）
      ros2 run robot_arm_node red_box_detector --ros-args -p show_window:=false  # headless

用法：
  ros2 run robot_arm_node red_box_detector
  ros2 launch robot_arm_bringup gazebo.launch.py controller:=ibvs_control
  ros2 launch robot_arm_bringup red_box_detect.launch.py

@copyright Copyright (c) 2026 eMeet
"""

import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image as ImageMsg
from tf2_ros import TransformListener, Buffer

# ── 相机内参（与 ibvs_controller.py 保持一致）───────────────────────────────
CAM_W    = 640
CAM_H    = 480
CAM_FOVY = 70.9
_fy = (CAM_H / 2) / math.tan(math.radians(CAM_FOVY / 2))
CAM_K    = np.array([[_fy, 0,  CAM_W / 2],
                     [0,  _fy, CAM_H / 2],
                     [0,   0,  1        ]], dtype=np.float32)
CAM_DIST = np.zeros((4, 1), dtype=np.float32)

# ── 红色方块物理尺寸 (obj_red_box: 0.06×0.04×0.04 m) ─────────────────────────
BOX_W = 0.06
BOX_H = 0.04

# ── 红色 HSV 阈值 ──────────────────────────────────────────────────────────────
RED_L1 = np.array([  0, 130,  60])
RED_H1 = np.array([  8, 255, 255])
RED_L2 = np.array([162, 130,  60])
RED_H2 = np.array([179, 255, 255])

IMAGE_TOPIC   = '/camera/camera_sensor/image_raw'
DEBUG_TOPIC   = '/red_detector/image'
FEATURE_TOPIC = '/red_detector/feature'   # PointStamped: x=x_norm, y=y_norm, z=depth
CAMERA_FRAME  = 'camera_optical_frame'
BASE_FRAME    = 'arm_base_link'
WINDOW_NAME   = 'Red Box Detector'


def _make_obj_pts(w, h):
    """生成物体坐标系下的四角点（正面朝相机，Z=0 平面）"""
    return np.array([[-w/2, -h/2, 0],
                     [ w/2, -h/2, 0],
                     [ w/2,  h/2, 0],
                     [-w/2,  h/2, 0]], dtype=np.float32)


def _order_corners(pts):
    """将 minAreaRect 的四角点排列为 TL→TR→BR→BL 顺序（与 obj_pts 对应）"""
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).reshape(-1)
    return np.array([pts[np.argmin(s)], pts[np.argmin(d)],
                     pts[np.argmax(s)], pts[np.argmax(d)]], dtype=np.float32)


def _quat_to_rot(x, y, z, w):
    """四元数 → 3×3 旋转矩阵"""
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-z*w),    2*(x*z+y*w)  ],
        [2*(x*y+z*w),    1-2*(x*x+z*z),  2*(y*z-x*w)  ],
        [2*(x*z-y*w),    2*(y*z+x*w),    1-2*(x*x+y*y)],
    ], dtype=np.float64)


class RedBoxDetector(Node):
    def __init__(self):
        super().__init__('red_box_detector')

        self.declare_parameter('image_topic', IMAGE_TOPIC)
        self.declare_parameter('min_area', 300)
        self.declare_parameter('show_window', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.min_area    = int(self.get_parameter('min_area').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        self.sub      = self.create_subscription(
            ImageMsg, self.image_topic, self._on_image, 10)
        self.pub      = self.create_publisher(ImageMsg, DEBUG_TOPIC, 10)
        self.feat_pub = self.create_publisher(PointStamped, FEATURE_TOPIC, 10)

        # TF 监听器
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.show_window:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        self.get_logger().info(
            f'红色方块识别已启动：{self.image_topic} → {DEBUG_TOPIC}'
            f'（弹窗 {"开" if self.show_window else "关"}，min_area={self.min_area}）')

    # ── 相机坐标 → 基座坐标 ───────────────────────────────────────────────────
    def _to_base(self, tx, ty, tz):
        """
        将相机坐标系下的点 (tx, ty, tz) 变换到 base_link 坐标系。
        变换公式：p_base = R @ p_cam + t
        返回 (bx, by, bz) 或 None（TF 尚未就绪时）。
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                BASE_FRAME, CAMERA_FRAME, rclpy.time.Time())
        except Exception:
            return None

        q   = tf.transform.rotation
        t   = tf.transform.translation
        R   = _quat_to_rot(q.x, q.y, q.z, q.w)
        p_b = R @ np.array([tx, ty, tz]) + np.array([t.x, t.y, t.z])
        return float(p_b[0]), float(p_b[1]), float(p_b[2])

    # ── 图像回调 ──────────────────────────────────────────────────────────────
    def _on_image(self, msg: ImageMsg):
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        except ValueError:
            self.get_logger().warn(
                f'图像尺寸不匹配 ({msg.encoding})，已跳过', throttle_duration_sec=5.0)
            return
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # ── HSV 红色掩码 ──────────────────────────────────────────────────────
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_L1, RED_H1),
            cv2.inRange(hsv, RED_L2, RED_H2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        # ── 轮廓检测 + solvePnP + TF 变换 ────────────────────────────────────
        cnts, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        count = 0
        for cnt in cnts:
            if cv2.contourArea(cnt) < self.min_area:
                continue

            rect   = cv2.minAreaRect(cnt)
            box_px = cv2.boxPoints(rect)
            cors   = _order_corners(box_px)

            rw, rh = rect[1]
            if rw < rh:
                rw, rh = rh, rw
            aspect  = rw / rh if rh > 1e-3 else 1.0
            obj_pts = (_make_obj_pts(BOX_W, BOX_H)
                       if abs(aspect - BOX_W / BOX_H) < 0.4
                       else _make_obj_pts((BOX_W + BOX_H) / 2,
                                          (BOX_W + BOX_H) / 2))

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, cors, CAM_K, CAM_DIST, flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                continue

            tx, ty, tz = tvec.ravel()
            distance   = float(tz)
            # solvePnP 偶发返回 NaN/Inf（轮廓退化或角点共线），跳过该帧
            if not math.isfinite(distance) or distance <= 0:
                continue

            count += 1

            cx_px, cy_px = int(rect[0][0]), int(rect[0][1])
            x_norm = (cx_px - float(CAM_K[0, 2])) / float(CAM_K[0, 0])
            y_norm = (cy_px - float(CAM_K[1, 2])) / float(CAM_K[1, 1])

            # 发布特征点话题供 IBVS 控制器订阅
            feat_msg = PointStamped()
            feat_msg.header          = msg.header
            feat_msg.header.frame_id = CAMERA_FRAME
            feat_msg.point.x         = x_norm
            feat_msg.point.y         = y_norm
            feat_msg.point.z         = distance
            self.feat_pub.publish(feat_msg)

            # TF 坐标变换：相机系 → 基座系
            base_pos = self._to_base(tx, ty, tz)

            # ── 绘制 ──────────────────────────────────────────────────────────
            cv2.drawContours(bgr, [box_px.astype(int)], 0, (0, 255, 0), 2)
            cv2.circle(bgr, (cx_px, cy_px), 4, (0, 0, 255), -1)
            try:
                cv2.drawFrameAxes(bgr, CAM_K, CAM_DIST, rvec, tvec, 0.02)
            except AttributeError:
                pass

            # 标注：四行
            lx = int(box_px[:, 0].min())
            ly = max(int(box_px[:, 1].min()) - 58, 14)
            cv2.putText(bgr,
                        f'red ({cx_px},{cy_px})  dist={distance:.3f}m',
                        (lx, ly),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(bgr,
                        f'norm=({x_norm:+.3f},{y_norm:+.3f},{distance:.3f})',
                        (lx, ly + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 2)
            cv2.putText(bgr,
                        f'cam=({tx:+.3f},{ty:+.3f},{tz:+.3f})m',
                        (lx, ly + 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 180, 0), 2)
            if base_pos is not None:
                bx, by, bz = base_pos
                cv2.putText(bgr,
                            f'base=({bx:+.3f},{by:+.3f},{bz:+.3f})m',
                            (lx, ly + 54),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 100, 255), 2)
                base_str = f'base=({bx:+.3f},{by:+.3f},{bz:+.3f})m'
            else:
                cv2.putText(bgr, 'base=TF not ready',
                            (lx, ly + 54),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 255), 2)
                base_str = 'base=TF not ready'

            self.get_logger().info(
                f'[#{count}] px=({cx_px},{cy_px})'
                f'  dist={distance:.3f}m'
                f'  norm=({x_norm:+.3f},{y_norm:+.3f},{distance:.3f})'
                f'  cam=({tx:+.3f},{ty:+.3f},{tz:+.3f})m'
                f'  {base_str}',
                throttle_duration_sec=0.5)

        cv2.putText(bgr, f'detected: {count}', (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # ── 发布 & 显示 ───────────────────────────────────────────────────────
        out = ImageMsg()
        out.header   = msg.header
        out.height, out.width = bgr.shape[:2]
        out.encoding = 'rgb8'
        out.step     = out.width * 3
        out.data     = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).tobytes()
        self.pub.publish(out)

        if self.show_window:
            cv2.imshow(WINDOW_NAME, bgr)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RedBoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
