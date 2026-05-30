from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .scenarios import get_scenario, list_scenarios


class SimulatedInputNode(Node):
    def __init__(self) -> None:
        super().__init__('simulated_input_node')

        self.declare_parameter('path_topic', '/global_path')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('terrain_cost_topic', '/terrain_cost')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('scenario', 'curved_normal')
        self.declare_parameter('publish_terrain', True)

        self._path_topic = self.get_parameter('path_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._imu_topic = self.get_parameter('imu_topic').value
        self._terrain_cost_topic = self.get_parameter('terrain_cost_topic').value
        self._publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._publish_terrain = self.get_parameter('publish_terrain').value

        # 加载场景
        scenario_name = self.get_parameter('scenario').value
        self._scenario = get_scenario(scenario_name)
        self.get_logger().info(
            f'simulated input started: scenario={scenario_name}, '
            f'path={self._path_topic}, odom={self._odom_topic}, imu={self._imu_topic}, terrain={self._terrain_cost_topic}'
        )

        from nav_msgs.msg import Odometry, Path as NavPath
        from sensor_msgs.msg import Imu

        self._Path_msg = NavPath
        self._Odometry_msg = Odometry
        self._Imu_msg = Imu

        self._path_pub = self.create_publisher(self._Path_msg, self._path_topic, 10)
        self._odom_pub = self.create_publisher(self._Odometry_msg, self._odom_topic, 10)
        self._imu_pub = self.create_publisher(self._Imu_msg, self._imu_topic, 10)
        self._terrain_pub = self.create_publisher(Float32, self._terrain_cost_topic, 10)

        self._time = 0.0
        self._path_msg = None  # 路径只生成一次（静态）
        self._timer = self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self._on_timer)

    def _on_timer(self) -> None:
        self._time += 1.0 / max(self._publish_rate_hz, 1.0)
        stamp = self.get_clock().now().to_msg()

        # 路径只生成一次（静态）
        if self._path_msg is None:
            self._path_msg = self._scenario.build_path(stamp)
        self._path_msg.header.stamp = stamp
        self._path_pub.publish(self._path_msg)

        self._odom_pub.publish(self._scenario.build_odom(self._time, stamp))
        self._imu_pub.publish(self._scenario.build_imu(self._time, stamp))

        if self._publish_terrain:
            terrain = Float32()
            terrain.data = float(self._scenario.terrain_cost(self._time))
            self._terrain_pub.publish(terrain)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulatedInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
