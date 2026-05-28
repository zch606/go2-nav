"""
stereo_terrain_node.py — D435i 深度图 → 2.5D 局部地形图

输入:  /camera/depth/image_rect_raw + camera_info
输出:  /local_terrain  (高程栅格图)
       /terrain_cost    (平均地形成本)

纯 NumPy + OpenCV, 无大库依赖。
"""

import math
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32
from cv_bridge import CvBridge

from .grid_utils import depth_to_3d, camera_to_ground, points_to_grid, grid_to_slope


class StereoTerrainNode(Node):
    """D435i → 2.5D 地形发布节点"""

    def __init__(self) -> None:
        super().__init__('stereo_terrain_node')

        # ---- 参数声明 ----
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('terrain_topic', '/local_terrain')
        self.declare_parameter('cost_topic', '/terrain_cost')

        self.declare_parameter('grid_resolution', 0.05)    # m/格
        self.declare_parameter('grid_length_m', 4.0)        # 前方 m
        self.declare_parameter('grid_width_m', 3.0)         # 左右 m

        self.declare_parameter('cam_height', 0.35)          # 相机离地高度 m
        self.declare_parameter('cam_pitch_deg', 10.0)       # 俯仰角 °

        self.declare_parameter('depth_min', 0.3)            # 最小有效深度 m
        self.declare_parameter('depth_max', 6.0)            # 最大有效深度 m
        self.declare_parameter('z_min', 0.005)              # 高程下界 m
        self.declare_parameter('z_max', 0.80)               # 高程上界 m

        self.declare_parameter('publish_rate_hz', 5.0)      # 输出频率
        self.declare_parameter('aggregation', 'max')         # 聚合方式

        # ---- 读取参数 ----
        self._depth_topic = self.get_parameter('depth_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value
        self._terrain_topic = self.get_parameter('terrain_topic').value
        self._cost_topic = self.get_parameter('cost_topic').value

        self._grid_res = float(self.get_parameter('grid_resolution').value)
        grid_length = float(self.get_parameter('grid_length_m').value)
        grid_width = float(self.get_parameter('grid_width_m').value)
        self._grid_h = int(grid_length / self._grid_res)
        self._grid_w = int(grid_width / self._grid_res)

        self._cam_height = float(self.get_parameter('cam_height').value)
        self._cam_pitch = math.radians(float(self.get_parameter('cam_pitch_deg').value))

        self._depth_min = float(self.get_parameter('depth_min').value)
        self._depth_max = float(self.get_parameter('depth_max').value)
        self._z_min = float(self.get_parameter('z_min').value)
        self._z_max = float(self.get_parameter('z_max').value)

        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._aggregation = self.get_parameter('aggregation').value

        # ---- 内部状态 ----
        self._bridge = CvBridge()
        self._latest_depth: np.ndarray | None = None
        self._fx: float | None = None
        self._fy: float | None = None
        self._cx: float | None = None
        self._cy: float | None = None
        self._last_publish = 0.0

        # ---- 订阅 ----
        self._depth_sub = self.create_subscription(
            Image, self._depth_topic, self._on_depth, 5,
        )
        self._info_sub = self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info, 5,
        )

        # ---- 发布 ----
        self._terrain_pub = self.create_publisher(Image, self._terrain_topic, 5)
        self._cost_pub = self.create_publisher(Float32, self._cost_topic, 5)

        # ---- 定时器 (处理) ----
        period = 1.0 / max(self._rate_hz, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'启动: {self._grid_h}×{self._grid_w} @ {self._grid_res}m/cell, '
            f'{self._rate_hz}Hz, 相机 H={self._cam_height}m θ={self.get_parameter("cam_pitch_deg").value}°'
        )

    # ------------------------------------------------------------------
    # 回调: 相机内参
    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._fx = msg.k[0]
        self._fy = msg.k[4]
        self._cx = msg.k[2]
        self._cy = msg.k[5]

    # ------------------------------------------------------------------
    # 回调: 深度图
    # ------------------------------------------------------------------
    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            self._latest_depth = depth
        except Exception as e:
            self.get_logger().warn(f'深度图解码失败: {e}', throttle_duration_sec=5.0)

    # ------------------------------------------------------------------
    # 定时器: 处理 + 发布
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        if self._latest_depth is None or self._fx is None:
            return

        now = time.time()
        period = 1.0 / self._rate_hz
        if now - self._last_publish < period * 0.9:
            return
        self._last_publish = now

        try:
            # 1. 深度 → 3D
            xc, yc, zc = depth_to_3d(
                self._latest_depth,
                self._fx, self._fy, self._cx, self._cy,
            )

            # 2. 相机坐标 → 地面坐标
            xg, yg, zg = camera_to_ground(
                xc, yc, zc,
                self._cam_height, self._cam_pitch,
            )

            # 3. 3D → 2.5D 栅格
            grid = points_to_grid(
                xg, yg, zg, zc,
                self._grid_res, self._grid_w, self._grid_h,
                self._z_min, self._z_max,
                self._depth_min, self._depth_max,
                aggregation=self._aggregation,
            )

            # 4. 坡度 + 成本
            _, terrain_cost = grid_to_slope(grid, self._grid_res)

            # 5. 发布高程栅格 (sensor_msgs/Image, 32FC1)
            grid_valid = np.where(np.isfinite(grid), grid, -1.0)
            terrain_msg = self._bridge.cv2_to_imgmsg(
                grid_valid.astype(np.float32), encoding='32FC1',
            )
            terrain_msg.header.frame_id = 'base_link'
            terrain_msg.header.stamp = self.get_clock().now().to_msg()
            self._terrain_pub.publish(terrain_msg)

            # 6. 发布地形成本
            cost_msg = Float32()
            cost_msg.data = float(terrain_cost)
            self._cost_pub.publish(cost_msg)

        except Exception as e:
            self.get_logger().warn(f'处理失败: {e}', throttle_duration_sec=5.0)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StereoTerrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
