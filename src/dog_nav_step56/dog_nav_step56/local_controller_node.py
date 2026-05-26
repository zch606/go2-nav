from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('local_controller_node')

        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('min_lookahead_distance', 1.5)
        self.declare_parameter('max_lookahead_distance', 3.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('max_linear_speed', 0.45)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('turn_gain', 2.2)
        self.declare_parameter('speed_gain', 0.75)
        self.declare_parameter('curvature_speed_scale', 0.55)
        self.declare_parameter('heading_slowdown_angle', 0.75)
        self.declare_parameter('heading_stop_angle', 2.5)

        self._path_topic = self.get_parameter('path_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._min_lookahead_distance = float(self.get_parameter('min_lookahead_distance').value)
        self._max_lookahead_distance = float(self.get_parameter('max_lookahead_distance').value)
        self._goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self._max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self._max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self._turn_gain = float(self.get_parameter('turn_gain').value)
        self._speed_gain = float(self.get_parameter('speed_gain').value)
        self._curvature_speed_scale = float(self.get_parameter('curvature_speed_scale').value)
        self._heading_slowdown_angle = float(self.get_parameter('heading_slowdown_angle').value)
        self._heading_stop_angle = float(self.get_parameter('heading_stop_angle').value)

        self._path: Optional[Path] = None
        self._odom: Optional[Odometry] = None
        self._path_sub = self.create_subscription(Path, self._path_topic, self._on_path, 10)
        self._odom_sub = self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._timer = self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self._on_timer)

        self.get_logger().info(
            f'local controller started: path={self._path_topic}, odom={self._odom_topic}, cmd_vel={self._cmd_vel_topic}'
        )

    def _on_path(self, msg: Path) -> None:
        self._path = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _current_pose(self) -> Optional[tuple[float, float, float]]:
        if self._odom is None:
            return None
        pose = self._odom.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return pose.position.x, pose.position.y, yaw

    def _dynamic_lookahead_distance(self, distance_to_goal: float) -> float:
        scaled = self._min_lookahead_distance + min(distance_to_goal, 2.5) * 0.12
        return max(self._min_lookahead_distance, min(self._max_lookahead_distance, scaled))

    def _select_lookahead_point(self, points: List[PoseStamped], x: float, y: float, lookahead_distance: float) -> Optional[PoseStamped]:
        if not points:
            return None

        selected = points[-1]
        for pose in points:
            dx = pose.pose.position.x - x
            dy = pose.pose.position.y - y
            distance = math.hypot(dx, dy)
            if distance >= lookahead_distance:
                selected = pose
                break
        return selected

    def _on_timer(self) -> None:
        twist = Twist()
        if self._path is None or self._odom is None or not self._path.poses:
            self._cmd_pub.publish(twist)
            return

        current = self._current_pose()
        if current is None:
            self._cmd_pub.publish(twist)
            return

        x, y, yaw = current
        goal_pose = self._path.poses[-1].pose.position
        distance_to_goal = math.hypot(goal_pose.x - x, goal_pose.y - y)
        if distance_to_goal <= self._goal_tolerance:
            self._cmd_pub.publish(twist)
            return

        lookahead_distance = self._dynamic_lookahead_distance(distance_to_goal)
        target = self._select_lookahead_point(self._path.poses, x, y, lookahead_distance)
        if target is None:
            self._cmd_pub.publish(twist)
            return

        dx = target.pose.position.x - x
        dy = target.pose.position.y - y
        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(math.sin(target_heading - yaw), math.cos(target_heading - yaw))
        lookahead = max(math.hypot(dx, dy), 1e-3)
        curvature = 2.0 * math.sin(heading_error) / lookahead

        linear_speed = min(self._max_linear_speed, max(0.0, self._speed_gain * distance_to_goal))
        curvature_scale = 1.0 / (1.0 + self._curvature_speed_scale * abs(curvature))
        linear_speed *= curvature_scale

        if abs(heading_error) >= self._heading_stop_angle:
            linear_speed = 0.0
        elif abs(heading_error) >= self._heading_slowdown_angle:
            linear_speed *= 0.35

        angular_speed = self._turn_gain * heading_error + linear_speed * curvature
        angular_speed = max(-self._max_angular_speed, min(self._max_angular_speed, angular_speed))

        angular_speed *= max(0.4, min(1.0, 1.0 - 0.12 * abs(curvature)))

        twist.linear.x = float(linear_speed)
        twist.angular.z = float(angular_speed)
        self._cmd_pub.publish(twist)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LocalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
