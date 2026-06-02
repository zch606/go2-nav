"""
simulated_camera_node.py — 发布模拟 D435i 深度图 + 内参, 供离线测试

方法: 世界坐标建地形 → 相机前向投影 → 深度图
      标准做法, 障碍物遮挡关系自然正确.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


# D435i 内参 (848×480)
FX, FY = 424.0, 424.0
CX, CY = 424.0, 240.0
W, H_IMG = 848, 480

# 相机位姿
CAM_HEIGHT = 0.35
CAM_PITCH = 0.0   # rad

# 栅格参数 (与 stereo_terrain 一致, 但范围稍大确保覆盖)
GRID_RES = 0.05
GRID_LEN = 5.0     # 前方 5m
GRID_WID = 3.0     # 左右 3m


def make_synthetic_depth() -> np.ndarray:
    """
    世界坐标系建完整地形 → 相机模型前向投影 → 深度图.

    步骤:
    1. 构建 100×60 世界高程栅格 (地面+障碍物)
    2. 把每个世界点 (xg,yg,zg) 投影到像素 (u,v) + 计算深度
    3. 取每像素最近深度 → 生成 848×480 深度图 (uint16 mm)
    """
    n_forward = int(GRID_LEN / GRID_RES)   # 100 行
    n_side = int(GRID_WID / GRID_RES)      # 60 列

    # 1. 世界高程栅格
    xg_arr = np.linspace(-GRID_WID / 2, GRID_WID / 2, n_side)   # 左右 -1.5~1.5
    yg_arr = np.linspace(GRID_RES, GRID_LEN, n_forward)          # 前方 0.05~5.0
    xx, yy = np.meshgrid(xg_arr, yg_arr)  # (100,60)
    zz = np.zeros_like(xx)                # 地面 zg=0

    # 加障碍物 (世界坐标 zw 直接赋值)
    # 盒子: x∈[0.2,0.8], y∈[1.5,2.5], zw=0.30
    box = (xx > 0.2) & (xx < 0.8) & (yy > 1.5) & (yy < 2.5)
    zz[box] = 0.30

    # 缓坡: x∈[-1.2,-0.4], y∈[2.5,4.5], zw 0→0.15
    ramp = (xx > -1.2) & (xx < -0.4) & (yy > 2.5) & (yy < 4.5)
    frac = np.clip((yy[ramp] - 2.5) / 2.0, 0, 1)
    zz[ramp] = frac * 0.15

    # 小墩: x∈[-0.3,0.3], y∈[3.0,3.6], zw=0.22
    bump = (xx > -0.3) & (xx < 0.3) & (yy > 3.0) & (yy < 3.6)
    zz[bump] = 0.22

    # 2. 世界 → 相机坐标 (逆变换, pitch=0)
    # camera_to_ground: xg=xc, yg=zc, zg=H-yc
    # 逆: xc=xg, yc=H-zg, zc=yg
    xc = xx
    yc = CAM_HEIGHT - zz          # 地面: yc=0.35, 盒子30cm: yc=0.05
    zc = yy                       # 前方距离 = 深度

    # 3. 相机投影 → 像素
    u = (xc * FX / zc + CX).astype(np.int32)
    v = (yc * FY / zc + CY).astype(np.int32)

    # 4. 每像素取最近面 (深度 mm)
    depth_mm = np.zeros((H_IMG, W), dtype=np.float32)
    valid = (u >= 0) & (u < W) & (v >= 0) & (v < H_IMG) & (zc > 0.1) & (zc < 50)
    depth = (zc * 1000.0).astype(np.float32)   # m → mm

    # 用 minimum 保持最近遮挡关系
    for i in range(n_forward):
        for j in range(n_side):
            if valid[i, j]:
                vi, ui = v[i, j], u[i, j]
                d = depth[i, j]
                cur = depth_mm[vi, ui]
                depth_mm[vi, ui] = d if cur == 0 else min(cur, d)

    return depth_mm.astype(np.uint16)


class SimulatedCameraNode(Node):
    """定时发布模拟 D435i 深度图 + 内参."""

    def __init__(self) -> None:
        super().__init__('simulated_camera_node')
        self.declare_parameter('rate_hz', 5.0)
        self._rate = self.get_parameter('rate_hz').value

        self._bridge = CvBridge()
        self._depth_pub = self.create_publisher(
            Image, '/camera/depth/image_rect_raw', 5)
        self._info_pub = self.create_publisher(
            CameraInfo, '/camera/depth/camera_info', 5)

        self._frame = 0
        self._timer = self.create_timer(1.0 / max(self._rate, 1.0), self._on_timer)
        self.get_logger().info(
            f'模拟 D435i {W}×{H_IMG}, pitch=0°, H={CAM_HEIGHT}m, {self._rate}Hz')

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()

        depth_mm = make_synthetic_depth()
        depth_msg = self._bridge.cv2_to_imgmsg(depth_mm, encoding='mono16')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self._depth_pub.publish(depth_msg)

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_depth_optical_frame'
        info.height = H_IMG
        info.width = W
        info.distortion_model = 'plumb_bob'
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

        self._frame += 1
        if self._frame % 10 == 0:
            self.get_logger().info(
                f'Frame #{self._frame}: {np.count_nonzero(depth_mm)} pixels',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
