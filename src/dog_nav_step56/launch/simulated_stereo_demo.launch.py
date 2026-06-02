"""
simulated_stereo_demo.launch.py — 模拟 D435i → 2.5D 地形 完整演示

启动:  模拟相机节点 (发布假深度图) + stereo_terrain_node (处理)
查看:  rviz2 -d <pkg_share>/rviz/terrain_view.rviz

完全离线, 不需要真机, 不需要 D435i 硬件.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory('dog_nav_step56')
    config = os.path.join(pkg_dir, 'config', 'stereo_terrain.yaml')

    return LaunchDescription([

        # ① 模拟相机: 定时发布 848×480 深度图 + D435i 内参
        Node(
            package='dog_nav_step56',
            executable='simulated_camera_node',
            name='simulated_camera_node',
            output='screen',
            parameters=[{'rate_hz': 5.0}],
        ),

        # ② 地形处理: 深度图 → 2.5D 栅格 + Mesh
        Node(
            package='dog_nav_step56',
            executable='stereo_terrain_node',
            name='stereo_terrain_node',
            output='screen',
            parameters=[config],
        ),
    ])
