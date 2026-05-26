from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    network_iface = LaunchConfiguration('network_iface')
    motion_topic = LaunchConfiguration('motion_command_topic')

    bridge_params = PathJoinSubstitution([
        FindPackageShare('dog_nav_step56'),
        'config',
        'unitree_bridge.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'network_iface',
            default_value='enp3s0',
            description='连接 Go2 的网口名称 (ifconfig查看)',
        ),
        DeclareLaunchArgument(
            'motion_command_topic',
            default_value='/motion_command',
            description='DogMotion 话题名',
        ),
        Node(
            package='dog_nav_step56',
            executable='unitree_bridge_node',
            name='unitree_bridge_node',
            output='screen',
            parameters=[bridge_params, {
                'motion_command_topic': motion_topic,
            }],
        ),
    ])
