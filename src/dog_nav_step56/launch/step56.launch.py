from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    safety_topic = LaunchConfiguration('safety_topic')
    motion_topic = LaunchConfiguration('motion_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    terrain_cost_topic = LaunchConfiguration('terrain_cost_topic')

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

    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('safety_topic', default_value='/safety_state'),
        DeclareLaunchArgument('motion_topic', default_value='/motion_command'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('terrain_cost_topic', default_value='/terrain_cost'),
        Node(
            package='dog_nav_step56',
            executable='motion_adapter_node',
            name='motion_adapter_node',
            output='screen',
            parameters=[motion_params],
            remappings=[
                ('/cmd_vel', cmd_vel_topic),
                ('/safety_state', safety_topic),
                ('/motion_command', motion_topic),
            ],
        ),
        Node(
            package='dog_nav_step56',
            executable='safety_supervisor_node',
            name='safety_supervisor_node',
            output='screen',
            parameters=[safety_params],
            remappings=[
                ('/imu/data', imu_topic),
                ('/odometry/filtered', odom_topic),
                ('/terrain_cost', terrain_cost_topic),
                ('/safety_state', safety_topic),
                ('/cmd_vel', cmd_vel_topic),
            ],
        ),
    ])
