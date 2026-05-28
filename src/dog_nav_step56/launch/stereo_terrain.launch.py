"""
stereo_terrain.launch.py — 仅启动 2.5D 地形感知

可配合 unitree_bridge 或 simulated_input 独立运行。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory('dog_nav_step56')
    config = os.path.join(pkg_dir, 'config', 'stereo_terrain.yaml')

    return LaunchDescription([
        Node(
            package='dog_nav_step56',
            executable='stereo_terrain_node.py',  # 需要 setup.py 注册
            name='stereo_terrain_node',
            output='screen',
            parameters=[config],
        ),
    ])
