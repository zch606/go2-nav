"""
stereo_terrain_node.py — D435i 深度图 → 2.5D 局部地形图

输入:  /camera/depth/image_rect_raw + camera_info
输出:  /local_terrain  (高程栅格图, 2D 俯视)
       /terrain_cost    (平均地形成本)
       /terrain_mesh    (2.5D 地形 Mesh, RViz2 可视化)

纯 NumPy + OpenCV, 无大库依赖。
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge

from .grid_utils import depth_to_3d, camera_to_ground, points_to_grid, grid_to_slope


# ════════════════════════════════════════════════════════════════
# 模块级工具: 栅格 → 三角形 Mesh
# ════════════════════════════════════════════════════════════════

def _height_to_color(z: float, z_min: float, z_max: float) -> tuple[float, float, float, float]:
    """高度(m) → RGBA: 蓝(低 0m) → 绿(中) → 红(高 0.8m), 无效值透明"""
    if np.isnan(z) or np.isinf(z) or z_max <= z_min:
        return (0.0, 0.0, 0.0, 0.0)
    t = max(0.0, min(1.0, (z - z_min) / (z_max - z_min)))
    return (t, 1.0 - abs(t - 0.5) * 2.0, 1.0 - t, 1.0)


def _grid_to_mesh(grid: np.ndarray, grid_res: float,
                  z_min: float, z_max: float,
                  frame_id: str, stamp) -> Marker:
    """
    80×60 高程栅格 → TRIANGLE_LIST Marker。

    grid: (H×W), row=前方, col=左右. 有效值=高度(m), -inf=无效.
    每 2×2 邻格生成 2 个三角形, 顶点按高度着色.
    """
    gh, gw = grid.shape
    half_w = gw // 2
    points: list[Point] = []
    colors: list[ColorRGBA] = []

    for r in range(gh - 1):
        for c in range(gw - 1):
            z00, z10 = grid[r, c], grid[r, c + 1]
            z01, z11 = grid[r + 1, c], grid[r + 1, c + 1]

            if not (np.isfinite(z00) or np.isfinite(z10) or
                    np.isfinite(z01) or np.isfinite(z11)):
                continue

            zs = [v if np.isfinite(v) else 0.0 for v in [z00, z10, z01, z11]]

            y0, y1 = r * grid_res, (r + 1) * grid_res
            x0 = (c - half_w) * grid_res
            x1 = (c + 1 - half_w) * grid_res

            for idx in (0, 1, 2,  1, 3, 2):   # 三角形1 + 三角形2
                px = x0 + (idx % 2) * (x1 - x0)
                py = y0 + (idx // 2) * (y1 - y0)
                pz = zs[idx]
                points.append(Point(x=float(px), y=float(py), z=float(pz)))
                rc, gc, bc, ac = _height_to_color(pz, z_min, z_max)
                colors.append(ColorRGBA(r=rc, g=gc, b=bc, a=ac))

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'terrain'
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.points = points
    marker.colors = colors
    marker.pose.orientation.w = 1.0
    return marker


class StereoTerrainNode(Node):
    """D435i → 2.5D 地形发布节点"""

    def __init__(self) -> None:
        super().__init__('stereo_terrain_node')

        # ---- 参数声明 ----
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('terrain_topic', '/local_terrain')
        self.declare_parameter('cost_topic', '/terrain_cost')
        self.declare_parameter('mesh_topic', '/terrain_mesh')

        self.declare_parameter('grid_resolution', 0.05)    # m/格
        self.declare_parameter('grid_length_m', 4.0)        # 前方 m
        self.declare_parameter('grid_width_m', 3.0)         # 左右 m

        self.declare_parameter('cam_height', 0.35)          # 相机离地高度 m
        self.declare_parameter('cam_pitch_deg', 10.0)       # 俯仰角 °

        self.declare_parameter('depth_min', 0.3)            # 最小有效深度 m
        self.declare_parameter('depth_max', 6.0)            # 最大有效深度 m
        self.declare_parameter('z_min', 0.005)              # 高程下界 m
        self.declare_parameter('z_max', 0.40)               # 高程上界 m

        self.declare_parameter('publish_rate_hz', 5.0)      # 输出频率
        self.declare_parameter('aggregation', 'max')         # 聚合方式

        # ---- 读取参数 ----
        self._depth_topic = self.get_parameter('depth_topic').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value
        self._terrain_topic = self.get_parameter('terrain_topic').value
        self._cost_topic = self.get_parameter('cost_topic').value
        self._mesh_topic = self.get_parameter('mesh_topic').value

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
        self._latest_depth = None
        self._fx = None
        self._fy = None
        self._cx = None
        self._cy = None
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
        self._mesh_pub = self.create_publisher(Marker, self._mesh_topic, 5)

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

            # ---- 调试: 看漏了多少点 ----
            valid_n = int(np.sum(np.isfinite(grid)))
            total_n = self._grid_h * self._grid_w
            self.get_logger().info(
                f'[DEBUG] grid {valid_n}/{total_n} ({100*valid_n/total_n:.0f}%), '
                f'zc={zc.min():.1f}~{zc.max():.1f}m, '
                f'yg={yg.min():.2f}~{yg.max():.2f}m, '
                f'zg={zg.min():.3f}~{zg.max():.3f}m',
                throttle_duration_sec=2.0,
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

            # 7. 发布 2.5D 地形 Mesh (RViz2 可视化)
            mesh = _grid_to_mesh(
                grid, self._grid_res,
                self._z_min, self._z_max,
                'base_link', self.get_clock().now().to_msg(),
            )
            self._mesh_pub.publish(mesh)

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
