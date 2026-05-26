"""
模拟场景库 —— 每个场景定义：路径、里程计、IMU(倾斜)、地形代价 的生成规则。

供 simulated_input_node 按 scenario 参数选择。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path as NavPath
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header as StdHeader


# ════════════════════════════════════════════════════════════════
# 场景注册表
# ════════════════════════════════════════════════════════════════

_SCENARIOS: dict[str, type['_BaseScenario']] = {}


def register(name: str):
    """装饰器：将场景类注册到场景表中。"""
    def _wrap(cls):
        _SCENARIOS[name] = cls
        cls.scenario_name = name
        return cls
    return _wrap


def get_scenario(name: str) -> '_BaseScenario':
    cls = _SCENARIOS.get(name)
    if cls is None:
        available = ', '.join(sorted(_SCENARIOS.keys()))
        raise KeyError(f'未知场景 "{name}"。可选: {available}')
    return cls()


def list_scenarios() -> list[str]:
    return sorted(_SCENARIOS.keys())


# ════════════════════════════════════════════════════════════════
# 基础类
# ════════════════════════════════════════════════════════════════

def _yaw_to_quat(yaw: float) -> Quaternion:
    half = yaw * 0.5
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(half)
    quat.w = math.cos(half)
    return quat


class _BaseScenario:
    """场景基类，子类覆盖属性即可。"""
    scenario_name: str = ''

    # 路径参数
    path_length: int = 12
    path_spacing: float = 0.5

    # 里程计参数
    speed: float = 0.25         # 前进速度因子
    lateral_amp: float = 0.15   # 横向摆动幅度
    lateral_freq: float = 0.6   # 横向摆动频率
    yaw_amp: float = 0.12       # 航向摆动幅度
    yaw_freq: float = 0.4       # 航向摆动频率

    # IMU 参数
    tilt_amp: float = 0.0       # 倾斜幅度(rad)，0=不倾斜
    tilt_freq: float = 0.7      # 倾斜频率 (subclass overrides)

    # 地形代价，返回 baseline
    base_terrain: float = 20.0

    # ---- 时间线配置 ----
    # terrain_timeline: list of (start_sec, terrain_value)
    #   eg: [(0, 20), (7, 150)] → 0~7s=20, 7s起=150
    terrain_timeline: list[tuple[float, float]] = [(0.0, 20.0)]

    # tilt_timeline: list of (start_sec, end_sec, tilt_rad)
    tilt_timeline: list[tuple[float, float, float]] = []

    def terrain_cost(self, t: float) -> float:
        """根据时间线返回当前地形代价。"""
        value = self.base_terrain
        for start_t, v in self.terrain_timeline:
            if t >= start_t:
                value = v
        return value

    def tilt_at(self, t: float) -> float:
        """返回 t 时刻的倾斜角度 (rad)。"""
        for t0, t1, rad in self.tilt_timeline:
            if t0 <= t < t1:
                return rad
        return 0.0

    def build_path(self, stamp) -> NavPath:
        """默认：生成蛇形路径。子类可覆盖。"""
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for i in range(self.path_length):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = i * self.path_spacing
            pose.pose.position.y = 0.25 * math.sin(i * 0.4)
            pose.pose.position.z = 0.0
            pose.pose.orientation = _yaw_to_quat(0.0)
            path.poses.append(pose)
        return path

    def build_odom(self, t: float, stamp) -> Odometry:
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = stamp
        odom.pose.pose.position.x = self.speed * t
        odom.pose.pose.position.y = self.lateral_amp * math.sin(t * self.lateral_freq)
        odom.pose.pose.position.z = 0.0
        yaw = self.yaw_amp * math.sin(t * self.yaw_freq)
        odom.pose.pose.orientation = _yaw_to_quat(yaw)
        odom.pose.covariance[0] = 0.05 + 0.02 * abs(math.sin(t * 0.5))
        return odom

    def build_imu(self, t: float, stamp) -> Imu:
        tilt = self.tilt_at(t)
        half = tilt * 0.5
        imu = Imu()
        imu.header.frame_id = 'base_link'
        imu.header.stamp = stamp
        imu.orientation.x = math.sin(half)
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = math.cos(half)
        return imu


# ════════════════════════════════════════════════════════════════
# 场景 1: 蛇形曲线 + 安全地形 → 全程 NORMAL，无降级
# ════════════════════════════════════════════════════════════════

@register('curved_normal')
class CurvedNormalScenario(_BaseScenario):
    """
    场景 1: 蛇形路径，全速行进，安全地形。

    验证:
      - gait 保持 TROT（高速小跑）
      - speed_level = 4（最高档）
      - 蛇形路径产生周期性 wz 方向变化
    """
    terrain_timeline = [(0.0, 20.0)]   # 始终 20（安全）


# ════════════════════════════════════════════════════════════════
# 场景 2: 直线高速 → 地形恶化 → 安全逐级降级
# ════════════════════════════════════════════════════════════════

@register('terrain_escalation')
class TerrainEscalationScenario(_BaseScenario):
    """
    场景 2: 直线路径，地形代价逐步上升，触发安全状态降级链。

    时间线:
      0~4s   terrain=20    → NORMAL  → Move + TROT
      4~8s   terrain=150   → SLOW    → Move(速度*0.5) + WALK
      8~12s  terrain=200   → PAUSE   → StopMove
      12s+   terrain=240   → STOP    → Damp

    验证:
      - Move → StopMove → Damp 安全转换
      - TROT → WALK → STAND 步态链
      - SpeedLevel 降级
    """
    path_length = 16
    path_spacing = 0.8
    speed = 0.5
    lateral_amp = 0.0
    yaw_amp = 0.0

    terrain_timeline = [
        (0.0, 20.0),
        (4.0, 150.0),
        (8.0, 200.0),
        (12.0, 240.0),
    ]

    def build_path(self, stamp) -> NavPath:
        """纯直线路径。"""
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for i in range(self.path_length):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = i * self.path_spacing
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation = _yaw_to_quat(0.0)
            path.poses.append(pose)
        return path


# ════════════════════════════════════════════════════════════════
# 场景 3: Z 字折返 + 大角度转弯 + 倾斜事件
# ════════════════════════════════════════════════════════════════

@register('zigzag_with_tilt')
class ZigzagTiltScenario(_BaseScenario):
    """
    场景 3: Z 字折返路径，模拟巡逻。包含大角度转弯和短时倾斜事件。

    时间线:
      0~5s   正常行驶（路径 Z 字，产生大角度转向 → CRAWL 步态）
      5~7s   倾斜 10°（≈0.175 rad）→ 触发 SLOW + speed_scale=0.5
      7s+    倾斜消失 → 恢复 NORMAL

    验证:
      - 大角度转弯 → CRAWL 步态
      - 倾斜触发 SLOW，安全降速
      - 倾斜消失后自动恢复
    """
    path_length = 12
    path_spacing = 0.6
    speed = 0.35
    yaw_amp = 0.6     # 大角度航向摆动 → 产生大 angular_speed
    yaw_freq = 0.8

    terrain_timeline = [(0.0, 20.0)]

    # 5~7 秒间倾斜 10°（0.175 rad），触发 SLOW（≥8°）
    tilt_timeline = [(5.0, 7.0, 0.175)]

    def build_path(self, stamp) -> NavPath:
        """Z 字形折返路径。"""
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for i in range(self.path_length):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = i * self.path_spacing
            pose.pose.position.y = (i % 2) * 1.5  # 交替 y=0 / y=1.5
            pose.pose.position.z = 0.0
            pose.pose.orientation = _yaw_to_quat(0.0)
            path.poses.append(pose)
        return path


# ════════════════════════════════════════════════════════════════
# 场景 0: 纯直行测试（最简，验证真机链路）
# ════════════════════════════════════════════════════════════════

@register('test_forward')
class TestForwardScenario(_BaseScenario):
    """
    场景 0: 纯直线前行，速度 0.3m/s，安全地形。

    用于真机最简验证——确保 Go2 收得到 Move(1008) 并往前走。
    """
    path_length = 30
    path_spacing = 0.3
    speed = 0.3
    lateral_amp = 0.0
    yaw_amp = 0.0

    terrain_timeline = [(0.0, 20.0)]

    def build_path(self, stamp) -> NavPath:
        """纯直线路径，沿 x 轴延伸。"""
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for i in range(self.path_length):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = stamp
            pose.pose.position.x = i * self.path_spacing
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation = _yaw_to_quat(0.0)
            path.poses.append(pose)
        return path


# ════════════════════════════════════════════════════════════════
# 场景: 直行往返 + 坐下（跑道形路径，走完自动 Damp）
# ════════════════════════════════════════════════════════════════

@register('round_trip')
class RoundTripScenario(_BaseScenario):
    """
    A-B-A 直线往返：走出去 5m → 水滴小圈掉头 → 走回来 5m → 坐下。

    路径约 12m，速度 0.3m/s，约 42 秒。
    45 秒后 terrain=240 → STOP → Damp(1001)。
    """
    speed = 0.3
    lateral_amp = 0.0
    yaw_amp = 0.0
    terrain_timeline = [(0.0, 20.0), (50.0, 240.0)]

    def build_path(self, stamp) -> NavPath:
        import math
        path = NavPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp

        def add(x, y, yaw):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = stamp
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.position.z = 0.0
            p.pose.orientation = _yaw_to_quat(float(yaw))
            path.poses.append(p)

        L = 5.0  # 出去/回来各 5m
        step = 0.3

        # 去程: 沿 +x, y=0
        for i in range(int(L / step) + 1):
            add(i * step, 0.0, 0.0)

        # 水滴形掉头（半径 1.2m，圆心在 (L+0.2, 0)）
        r = 1.2
        cx = L + 0.2
        cy = 0.0
        n = 20
        for i in range(1, n + 1):
            # 从 heading=0 顺时针转 180° 到 heading=π
            angle = math.pi * i / n  # 0 → π
            x = cx + r * math.cos(angle - math.pi/2)  # 从正上方开始绕
            y = cy + r * math.sin(angle - math.pi/2)
            yaw = angle
            add(x, y, yaw)

        # 回程: 沿 -x, y=0
        for i in range(int(L / step) + 1):
            add(L - i * step, 0.0, math.pi)

        return path

    def build_odom(self, t: float, stamp) -> Odometry:
        import math
        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = stamp

        L = 5.0
        r = 1.2
        arc = math.pi * r        # 半圆弧长 ≈ 3.77m
        total = L * 2 + arc      # ≈ 13.77m
        dist = self.speed * t % total

        if dist <= L:
            # 去程
            x, y, yaw = dist, 0.0, 0.0
        elif dist <= L + arc:
            # 掉头
            progress = (dist - L) / arc
            angle = math.pi * progress
            cx, cy = L + 0.2, 0.0
            x = cx + r * math.cos(angle - math.pi/2)
            y = cy + r * math.sin(angle - math.pi/2)
            yaw = angle
        else:
            # 回程
            back = dist - L - arc
            x, y, yaw = L - back, 0.0, math.pi

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = _yaw_to_quat(yaw)
        odom.pose.covariance[0] = 0.05
        return odom
