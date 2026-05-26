from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node

from dog_nav_interfaces.msg import DogMotion
from unitree_api.msg import Request as UnitreeRequest


# ---- Unitree Sport API IDs (from ros2_sport_client.h) ----
API_DAMP = 1001
API_BALANCE_STAND = 1002
API_STOP_MOVE = 1003
API_STAND_UP = 1004
API_STAND_DOWN = 1005
API_RECOVERY_STAND = 1006
API_MOVE = 1008
API_SPEED_LEVEL = 1015
API_STATIC_WALK = 1061
API_TROT_RUN = 1062
API_ECONOMIC_GAIT = 1063

# DogMotion gait constants (from DogMotion.msg)
GAIT_STAND = 0
GAIT_WALK = 1
GAIT_TROT = 2
GAIT_CRAWL = 3

# SafetyState constants (from SafetyState.msg)
SAFETY_NORMAL = 0
SAFETY_DEGRADED = 1
SAFETY_SLOW = 2
SAFETY_PAUSE = 3
SAFETY_STOP = 4
SAFETY_MANUAL = 5

# Gait → Unitree API mapping
_GAIT_TO_API = {
    GAIT_WALK: API_STATIC_WALK,
    GAIT_TROT: API_TROT_RUN,
    GAIT_CRAWL: API_ECONOMIC_GAIT,
}


class UnitreeBridgeNode(Node):
    """将 DogMotion 翻译为 Unitree Go2 Sport API Request，发布到 /api/sport/request."""

    def __init__(self) -> None:
        super().__init__('unitree_bridge_node')

        # 参数声明
        self.declare_parameter('motion_command_topic', '/motion_command')
        self.declare_parameter('sport_request_topic', '/api/sport/request')
        self.declare_parameter('sport_state_topic', '/lf/sportmodestate')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('sensor_timeout_sec', 1.5)
        self.declare_parameter('speed_level_thresholds', [0.20, 0.35, 0.60])

        self._motion_topic = self.get_parameter('motion_command_topic').value
        self._request_topic = self.get_parameter('sport_request_topic').value
        self._state_topic = self.get_parameter('sport_state_topic').value
        self._rate = float(self.get_parameter('publish_rate_hz').value)
        self._timeout = float(self.get_parameter('sensor_timeout_sec').value)
        self._thresholds: list[float] = list(self.get_parameter('speed_level_thresholds').value)

        # 状态
        self._last_motion: DogMotion | None = None
        self._last_gait: int = -1          # 上一次发送的步态（去抖）
        self._last_speed_level: int = -1   # 上一次发送的速度档位（去抖）
        self._last_api_id: int = -1        # 上一次使用的 API ID
        self._motion_arrived = False
        self._start_time = self.get_clock().now()  # 启动时间，用于缓冲期
        self._startup_grace_sec = 3.0              # 启动后 N 秒内不发任何指令

        # 订阅 /motion_command
        self._motion_sub = self.create_subscription(
            DogMotion, self._motion_topic, self._on_motion, 10,
        )

        # 发布 /api/sport/request
        self._req_pub = self.create_publisher(UnitreeRequest, self._request_topic, 10)

        # 定时器：按 10Hz 连续发送 Move 请求
        period = 1.0 / max(self._rate, 1.0)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'unitree bridge started: motion={self._motion_topic}, '
            f'request={self._request_topic}'
        )

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def _on_motion(self, msg: DogMotion) -> None:
        self._last_motion = msg
        self._motion_arrived = True

    # ------------------------------------------------------------------
    # 定时器：主逻辑
    # ------------------------------------------------------------------
    def _on_timer(self) -> None:
        # 启动缓冲期：前 N 秒不发任何指令，等安全状态稳定
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if elapsed < self._startup_grace_sec:
            return

        if not self._motion_arrived or self._last_motion is None:
            # 尚未收到首条 DogMotion，不发送任何指令（安全）
            return

        motion = self._last_motion
        safety = motion.safety_state

        # ---- 1. 安全状态判断 ----
        if safety in (SAFETY_STOP, SAFETY_MANUAL):
            self._send_damp('safety state STOP/MANUAL')
            return

        if safety == SAFETY_PAUSE:
            self._send_stop_move()
            return

        # SLOW / NORMAL / DEGRADED: 发送速度指令
        speed_scale = 1.0 if safety == SAFETY_NORMAL else 0.5
        vx = float(motion.vx) * speed_scale
        vy = float(motion.vy) * speed_scale
        wz = float(motion.wz) * speed_scale

        self._send_move(vx, vy, wz)

        # ---- 2. 步态切换 (仅在 gait 变化时发送一次) ----
        gait = motion.gait
        if gait != self._last_gait:
            self._send_gait_switch(gait)

        # ---- 3. 速度档位切换 (仅在 speed_limit 变化导致 level 变化时发送) ----
        new_level = self._speed_limit_to_level(float(motion.speed_limit))
        if new_level != self._last_speed_level and new_level > 0:
            self._send_speed_level(new_level)

    # ------------------------------------------------------------------
    # API 发送方法
    # ------------------------------------------------------------------
    def _build_request(self, api_id: int, parameter: dict | None = None) -> UnitreeRequest:
        req = UnitreeRequest()
        req.header.identity.api_id = api_id
        req.header.policy.priority = 0
        req.header.policy.noreply = True
        req.parameter = json.dumps(parameter) if parameter else ''
        return req

    def _publish_request(self, api_id: int, description: str, parameter: dict | None = None) -> None:
        req = self._build_request(api_id, parameter)
        self._req_pub.publish(req)
        if self._last_api_id != api_id:
            self.get_logger().info(f'→ {description}')
            self._last_api_id = api_id

    def _send_damp(self, reason: str) -> None:
        self._publish_request(API_DAMP, f'Damp({API_DAMP}): {reason}')
        self._last_gait = -1
        self._last_speed_level = -1

    def _send_stop_move(self) -> None:
        self._publish_request(API_STOP_MOVE, f'StopMove({API_STOP_MOVE})')
        self._last_gait = -1
        self._last_speed_level = -1

    def _send_move(self, vx: float, vy: float, wz: float) -> None:
        params = {'x': vx, 'y': vy, 'z': wz}
        self._publish_request(
            API_MOVE,
            f'Move({API_MOVE}): vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}',
            parameter=params,
        )

    def _send_gait_switch(self, gait: int) -> None:
        """仅在步态变化时调用一次."""
        api_id = _GAIT_TO_API.get(gait)
        gait_names = {GAIT_STAND: 'STAND', GAIT_WALK: 'WALK', GAIT_TROT: 'TROT', GAIT_CRAWL: 'CRAWL'}
        gait_name = gait_names.get(gait, f'UNKNOWN({gait})')

        if api_id is None:
            # STAND(0): 不需要步态切换
            self._last_gait = gait
            return

        self._publish_request(
            api_id,
            f'GaitSwitch: {gait_name} → api_id={api_id}',
        )
        self._last_gait = gait

    def _send_speed_level(self, level: int) -> None:
        """仅在 speed_level 变化时调用一次."""
        self._publish_request(
            API_SPEED_LEVEL,
            f'SpeedLevel({API_SPEED_LEVEL}): level={level}',
            parameter={'data': level},
        )
        self._last_speed_level = level

    # ------------------------------------------------------------------
    # 工具方法
    # ------------------------------------------------------------------
    def _speed_limit_to_level(self, speed_limit: float) -> int:
        """将 speed_limit 映射到宇树 SpeedLevel."""
        for idx, threshold in enumerate(self._thresholds):
            if speed_limit <= threshold:
                return idx + 1
        return len(self._thresholds) + 1


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = UnitreeBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
