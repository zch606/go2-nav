from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_msgs.msg import Header

from dog_nav_interfaces.msg import SafetyState
from .state_machine import SafetyLevel, evaluate_safety


def _quaternion_to_roll_pitch(x: float, y: float, z: float, w: float) -> tuple[float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    return roll, pitch


class SafetySupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_supervisor_node')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('terrain_cost_topic', '/terrain_cost')
        self.declare_parameter('safety_topic', '/safety_state')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('sensor_timeout_sec', 1.5)
        self.declare_parameter('nominal_speed_limit', 0.60)

        self._imu_topic = self.get_parameter('imu_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._terrain_cost_topic = self.get_parameter('terrain_cost_topic').value
        self._safety_topic = self.get_parameter('safety_topic').value
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self._nominal_speed_limit = float(self.get_parameter('nominal_speed_limit').value)

        self._last_imu = None
        self._last_odom = None
        self._last_cmd = Twist()
        self._terrain_cost = 0.0
        self._last_imu_time = self.get_clock().now()
        self._last_odom_time = self.get_clock().now()
        self._last_cmd_time = self.get_clock().now()
        self._last_terrain_time = self.get_clock().now()

        self._imu_sub = self.create_subscription(Imu, self._imu_topic, self._on_imu, 10)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._cmd_sub = self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self._terrain_sub = self.create_subscription(Float32, self._terrain_cost_topic, self._on_terrain_cost, 10)
        self._safety_pub = self.create_publisher(SafetyState, self._safety_topic, 10)

        timer_period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f'safety supervisor started: imu={self._imu_topic}, odom={self._odom_topic}, cmd_vel={self._cmd_vel_topic}, terrain={self._terrain_cost_topic}'
        )

    def _on_imu(self, msg: Imu) -> None:
        self._last_imu = msg
        self._last_imu_time = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg
        self._last_odom_time = self.get_clock().now()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()

    def _on_terrain_cost(self, msg: Float32) -> None:
        self._terrain_cost = float(msg.data)
        self._last_terrain_time = self.get_clock().now()

    def _age_sec(self, stamp) -> float:
        return (self.get_clock().now() - stamp).nanoseconds * 1e-9

    def _on_timer(self) -> None:
        assessment = self._assess()
        self._publish_state(assessment)

    def _assess(self) -> SafetyState:
        state = SafetyState()
        state.header = Header()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'base_link'

        imu_age = self._age_sec(self._last_imu_time)
        odom_age = self._age_sec(self._last_odom_time)
        cmd_age = self._age_sec(self._last_cmd_time)

        if self._last_imu is None or self._last_odom is None:
            state.state = SafetyLevel.MANUAL
            state.severity = 5
            state.emergency_stop = True
            state.allow_motion = False
            state.speed_scale = 0.0
            state.terrain_cost = self._terrain_cost
            state.roll_deg = 0.0
            state.pitch_deg = 0.0
            state.odom_covariance = 1.0
            state.reason = 'waiting for imu or odometry'
            return state

        if imu_age > self._sensor_timeout_sec or odom_age > self._sensor_timeout_sec:
            state.state = SafetyLevel.PAUSE
            state.severity = 3
            state.emergency_stop = False
            state.allow_motion = False
            state.speed_scale = 0.0
            state.terrain_cost = self._terrain_cost
            state.roll_deg = 0.0
            state.pitch_deg = 0.0
            state.odom_covariance = 1.0
            state.reason = 'sensor data timeout'
            return state

        orientation = self._last_imu.orientation
        roll_deg, pitch_deg = _quaternion_to_roll_pitch(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        covariance = self._last_odom.pose.covariance[0] if len(self._last_odom.pose.covariance) > 0 else 0.0
        if cmd_age > self._sensor_timeout_sec:
            covariance = max(covariance, 0.2)

        assessment = evaluate_safety(
            roll_deg=roll_deg,
            pitch_deg=pitch_deg,
            terrain_cost=self._terrain_cost,
            odom_covariance=float(covariance),
            cmd_linear_speed=self._last_cmd.linear.x,
            cmd_angular_speed=self._last_cmd.angular.z,
        )

        state.state = assessment.state
        state.severity = assessment.severity
        state.emergency_stop = assessment.emergency_stop
        state.allow_motion = assessment.allow_motion
        state.speed_scale = assessment.speed_scale
        state.terrain_cost = float(self._terrain_cost)
        state.roll_deg = float(roll_deg)
        state.pitch_deg = float(pitch_deg)
        state.odom_covariance = float(covariance)
        state.reason = assessment.reason
        return state

    def _publish_state(self, msg: SafetyState) -> None:
        self._safety_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SafetySupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
