from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scenario = LaunchConfiguration('scenario')
    use_bridge = LaunchConfiguration('use_bridge')

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario',
            default_value='curved_normal',
            description='模拟场景: curved_normal / terrain_escalation / zigzag_with_tilt',
        ),
        DeclareLaunchArgument(
            'use_bridge',
            default_value='false',
            description='设为 true 启动 unitree_bridge_node 对接 Go2 真机',
        ),
        Node(
            package='dog_nav_step56',
            executable='simulated_input_node',
            name='simulated_input_node',
            output='screen',
            parameters=[{
                'scenario': scenario,
            }],
        ),
        Node(
            package='dog_nav_step56',
            executable='local_controller_node',
            name='local_controller_node',
            output='screen',
            parameters=[{}],
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
        Node(
            package='dog_nav_step56',
            executable='unitree_bridge_node',
            name='unitree_bridge_node',
            output='screen',
            parameters=[bridge_params],
            condition=IfCondition(use_bridge),
        ),
    ])
