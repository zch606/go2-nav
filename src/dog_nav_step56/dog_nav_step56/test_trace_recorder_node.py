from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from dog_nav_interfaces.msg import DogMotion, SafetyState


def _now_stamp(node: Node) -> dict[str, int]:
    stamp = node.get_clock().now().to_msg()
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


def _quat_to_rpy_deg(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.degrees(math.copysign(math.pi / 2.0, sinp))
    else:
        pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


class TestTraceRecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('test_trace_recorder_node')

        self.declare_parameter('trace_file', '/root/ws/step56_test_trace.jsonl')
        self.declare_parameter('global_path_topic', '/global_path')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('terrain_cost_topic', '/terrain_cost')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('safety_topic', '/safety_state')
        self.declare_parameter('motion_topic', '/motion_command')

        trace_file = Path(str(self.get_parameter('trace_file').value)).expanduser()
        trace_file.parent.mkdir(parents=True, exist_ok=True)
        self._trace_file = trace_file
        self._trace_handle = trace_file.open('a', encoding='utf-8')

        self._global_path_topic = self.get_parameter('global_path_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._imu_topic = self.get_parameter('imu_topic').value
        self._terrain_cost_topic = self.get_parameter('terrain_cost_topic').value
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._safety_topic = self.get_parameter('safety_topic').value
        self._motion_topic = self.get_parameter('motion_topic').value

        self._subscriptions = [
            self.create_subscription(NavPath, self._global_path_topic, self._on_global_path, 10),
            self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10),
            self.create_subscription(Imu, self._imu_topic, self._on_imu, 10),
            self.create_subscription(Float32, self._terrain_cost_topic, self._on_terrain_cost, 10),
            self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10),
            self.create_subscription(SafetyState, self._safety_topic, self._on_safety, 10),
            self.create_subscription(DogMotion, self._motion_topic, self._on_motion, 10),
        ]

        self._write_event('trace_started', {
            'trace_file': str(self._trace_file),
            'topics': {
                'global_path': self._global_path_topic,
                'odom': self._odom_topic,
                'imu': self._imu_topic,
                'terrain_cost': self._terrain_cost_topic,
                'cmd_vel': self._cmd_vel_topic,
                'safety_state': self._safety_topic,
                'motion_command': self._motion_topic,
            },
        })
        self.get_logger().info(f'test trace recorder writing to {self._trace_file}')

    def destroy_node(self) -> bool:
        try:
            self._write_event('trace_stopped', {'trace_file': str(self._trace_file)})
            self._trace_handle.flush()
            self._trace_handle.close()
        except Exception:
            pass
        return super().destroy_node()

    def _write_event(self, event_type: str, payload: dict[str, Any]) -> None:
        record = {
            'stamp': _now_stamp(self),
            'event': event_type,
            'payload': payload,
        }
        self._trace_handle.write(json.dumps(record, ensure_ascii=False) + '\n')
        self._trace_handle.flush()

    def _on_global_path(self, msg: NavPath) -> None:
        poses = msg.poses
        first = poses[0].pose.position if poses else None
        last = poses[-1].pose.position if poses else None
        self._write_event('global_path', {
            'frame_id': msg.header.frame_id,
            'pose_count': len(poses),
            'first': None if first is None else {'x': first.x, 'y': first.y, 'z': first.z},
            'last': None if last is None else {'x': last.x, 'y': last.y, 'z': last.z},
        })

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        roll, pitch, yaw = _quat_to_rpy_deg(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        self._write_event('odometry', {
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'position': {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z},
            'rpy_deg': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'covariance_00': float(msg.pose.covariance[0]) if len(msg.pose.covariance) > 0 else 0.0,
        })

    def _on_imu(self, msg: Imu) -> None:
        roll, pitch, yaw = _quat_to_rpy_deg(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self._write_event('imu', {
            'frame_id': msg.header.frame_id,
            'orientation_rpy_deg': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
        })

    def _on_terrain_cost(self, msg: Float32) -> None:
        self._write_event('terrain_cost', {'value': float(msg.data)})

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._write_event('cmd_vel', {
            'linear': {'x': float(msg.linear.x), 'y': float(msg.linear.y), 'z': float(msg.linear.z)},
            'angular': {'x': float(msg.angular.x), 'y': float(msg.angular.y), 'z': float(msg.angular.z)},
        })

    def _on_safety(self, msg: SafetyState) -> None:
        self._write_event('safety_state', {
            'state': int(msg.state),
            'severity': int(msg.severity),
            'emergency_stop': bool(msg.emergency_stop),
            'allow_motion': bool(msg.allow_motion),
            'speed_scale': float(msg.speed_scale),
            'terrain_cost': float(msg.terrain_cost),
            'roll_deg': float(msg.roll_deg),
            'pitch_deg': float(msg.pitch_deg),
            'odom_covariance': float(msg.odom_covariance),
            'reason': msg.reason,
        })

    def _on_motion(self, msg: DogMotion) -> None:
        self._write_event('motion_command', {
            'vx': float(msg.vx),
            'vy': float(msg.vy),
            'wz': float(msg.wz),
            'gait': int(msg.gait),
            'body_height': float(msg.body_height),
            'duration': float(msg.duration),
            'speed_limit': float(msg.speed_limit),
            'safety_state': int(msg.safety_state),
            'source': msg.source,
        })


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TestTraceRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
