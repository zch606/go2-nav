from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

from dog_nav_interfaces.msg import DogMotion, SafetyState
from .state_machine import MotionGait, SafetyLevel, select_motion_profile


class MotionAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__('motion_adapter_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('safety_topic', '/safety_state')
        self.declare_parameter('motion_topic', '/motion_command')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('default_body_height', 0.24)
        self.declare_parameter('stale_cmd_timeout_sec', 1.0)

        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._safety_topic = self.get_parameter('safety_topic').value
        self._motion_topic = self.get_parameter('motion_topic').value
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._default_body_height = float(self.get_parameter('default_body_height').value)
        self._stale_cmd_timeout_sec = float(self.get_parameter('stale_cmd_timeout_sec').value)

        self._current_cmd = Twist()
        self._current_safety = SafetyState()
        self._current_safety.state = SafetyState.MANUAL
        self._current_safety.severity = 5
        self._current_safety.emergency_stop = True
        self._current_safety.allow_motion = False
        self._current_safety.speed_scale = 0.0
        self._current_safety.reason = 'waiting for safety supervisor'
        self._last_cmd_time = self.get_clock().now()
        self._last_safety_time = self.get_clock().now()

        self._cmd_sub = self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self._safety_sub = self.create_subscription(SafetyState, self._safety_topic, self._on_safety, 10)
        self._motion_pub = self.create_publisher(DogMotion, self._motion_topic, 10)

        timer_period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f'motion adapter started: cmd_vel={self._cmd_vel_topic}, safety={self._safety_topic}, motion={self._motion_topic}'
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._current_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._publish_motion()

    def _on_safety(self, msg: SafetyState) -> None:
        self._current_safety = msg
        self._last_safety_time = self.get_clock().now()
        self._publish_motion()

    def _cmd_is_stale(self) -> bool:
        elapsed = self.get_clock().now() - self._last_cmd_time
        return elapsed.nanoseconds * 1e-9 > self._stale_cmd_timeout_sec

    def _on_timer(self) -> None:
        self._publish_motion()

    def _publish_motion(self) -> None:
        if self._cmd_is_stale():
            cmd = Twist()
        else:
            cmd = self._current_cmd

        safety_state = self._current_safety.state
        safety_scale = float(self._current_safety.speed_scale)

        if not self._current_safety.allow_motion or self._current_safety.emergency_stop:
            cmd = Twist()
            safety_scale = 0.0

        profile = select_motion_profile(
            linear_speed=cmd.linear.x,
            angular_speed=cmd.angular.z,
            safety_state=safety_state,
            speed_scale=safety_scale,
        )

        motion = DogMotion()
        motion.header = Header()
        motion.header.stamp = self.get_clock().now().to_msg()
        motion.header.frame_id = 'base_link'
        motion.vx = float(cmd.linear.x * safety_scale)
        motion.vy = float(cmd.linear.y * safety_scale)
        motion.wz = float(cmd.angular.z * safety_scale)
        motion.gait = profile.gait
        motion.body_height = profile.body_height if safety_state not in (SafetyLevel.NORMAL, SafetyLevel.DEGRADED) else max(profile.body_height, self._default_body_height)
        motion.duration = profile.duration
        motion.speed_limit = profile.speed_limit
        motion.safety_state = safety_state
        motion.source = 'motion_adapter_node'

        self._motion_pub.publish(motion)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotionAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
