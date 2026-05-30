from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scenario = LaunchConfiguration('scenario')
    use_bridge = LaunchConfiguration('use_bridge')
    use_stereo = LaunchConfiguration('use_stereo')

    motion_params = PathJoinSubstitution([
        FindPackageShare('dog_nav_step56'),
        'config',
        'motion_adapter.yaml',
    ])
    safety_params = PathJoinSubstitution([
        FindPackageShare('dog_nav_step56'),
        'config',
        'safety_supervisor.yaml',
    ])
    bridge_params = PathJoinSubstitution([
        FindPackageShare('dog_nav_step56'),
        'config',
        'unitree_bridge.yaml',
    ])
    stereo_params = PathJoinSubstitution([
        FindPackageShare('dog_nav_step56'),
        'config',
        'stereo_terrain.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='curved_normal',
            description='模拟场景: curved_normal / test_forward / zigzag_with_tilt / round_trip',
        ),
        DeclareLaunchArgument(
            'use_bridge',
            default_value='false',
            description='设为 true 启动 unitree_bridge_node 对接 Go2 真机',
        ),
        DeclareLaunchArgument(
            'use_stereo',
            default_value='false',
            description='设为 true 启动 stereo_terrain_node 用 D435i 实时 2.5D 地形（自动关 simulated_input 假 terrain）',
        ),

        # ---- 模拟输入（不发假 terrain 仅当 use_stereo=true 时） ----
        Node(
            package='dog_nav_step56',
            executable='simulated_input_node',
            name='simulated_input_node',
            output='screen',
            parameters=[{
                'scenario': scenario,
                'publish_terrain': False,
            }],
            condition=IfCondition(use_stereo),
        ),
        # ---- 模拟输入（正常模式，含 terrain） ----
        Node(
            package='dog_nav_step56',
            executable='simulated_input_node',
            name='simulated_input_node',
            output='screen',
            parameters=[{
                'scenario': scenario,
            }],
            condition=UnlessCondition(use_stereo),
        ),

        # ---- 通用导航节点 ----
        Node(
            package='dog_nav_step56',
            executable='local_controller_node',
            name='local_controller_node',
            output='screen',
        ),
        Node(
            package='dog_nav_step56',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[safety_params],
        ),
        Node(
            package='dog_nav_step56',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[motion_params],
        ),

        # ---- 真机桥接 ----
        Node(
            package='dog_nav_step56',
            executable='unitree_bridge_node',
            name='unitree_bridge_node',
            output='screen',
            parameters=[bridge_params],
            condition=IfCondition(use_bridge),
        ),

        # ---- D435i 2.5D 地形感知 ----
        Node(
            package='dog_nav_step56',
            executable='stereo_terrain_node',
            name='stereo_terrain_node',
            output='screen',
            parameters=[stereo_params],
            condition=IfCondition(use_stereo),
        ),
    ])
