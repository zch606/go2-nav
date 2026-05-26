from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    terrain_cost = LaunchConfiguration('terrain_cost')
    simulate_tilt = LaunchConfiguration('simulate_tilt')
    path_length = LaunchConfiguration('path_length')
    path_spacing = LaunchConfiguration('path_spacing')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    controller_rate_hz = LaunchConfiguration('controller_rate_hz')
    min_lookahead_distance = LaunchConfiguration('min_lookahead_distance')
    max_lookahead_distance = LaunchConfiguration('max_lookahead_distance')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    turn_gain = LaunchConfiguration('turn_gain')
    speed_gain = LaunchConfiguration('speed_gain')
    curvature_speed_scale = LaunchConfiguration('curvature_speed_scale')
    heading_slowdown_angle = LaunchConfiguration('heading_slowdown_angle')
    heading_stop_angle = LaunchConfiguration('heading_stop_angle')
    trace_file = LaunchConfiguration('trace_file')

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
        DeclareLaunchArgument('terrain_cost', default_value='20.0'),
        DeclareLaunchArgument('simulate_tilt', default_value='false'),
        DeclareLaunchArgument('path_length', default_value='12'),
        DeclareLaunchArgument('path_spacing', default_value='0.5'),
        DeclareLaunchArgument('publish_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('controller_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('min_lookahead_distance', default_value='0.45'),
        DeclareLaunchArgument('max_lookahead_distance', default_value='0.90'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.45'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.20'),
        DeclareLaunchArgument('turn_gain', default_value='2.20'),
        DeclareLaunchArgument('speed_gain', default_value='0.75'),
        DeclareLaunchArgument('curvature_speed_scale', default_value='0.55'),
        DeclareLaunchArgument('heading_slowdown_angle', default_value='0.75'),
        DeclareLaunchArgument('heading_stop_angle', default_value='1.35'),
        DeclareLaunchArgument('trace_file', default_value='/root/ws/step56_test_trace.jsonl'),
        Node(
            package='dog_nav_step56',
            executable='simulated_input_node',
            name='simulated_input_node',
            output='screen',
            parameters=[{
                'terrain_cost': terrain_cost,
                'simulate_tilt': simulate_tilt,
                'path_length': path_length,
                'path_spacing': path_spacing,
                'publish_rate_hz': publish_rate_hz,
            }],
        ),
        Node(
            package='dog_nav_step56',
            executable='local_controller_node',
            name='local_controller_node',
            output='screen',
            parameters=[{
                'publish_rate_hz': controller_rate_hz,
                'min_lookahead_distance': min_lookahead_distance,
                'max_lookahead_distance': max_lookahead_distance,
                'max_linear_speed': max_linear_speed,
                'max_angular_speed': max_angular_speed,
                'turn_gain': turn_gain,
                'speed_gain': speed_gain,
                'curvature_speed_scale': curvature_speed_scale,
                'heading_slowdown_angle': heading_slowdown_angle,
                'heading_stop_angle': heading_stop_angle,
            }],
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
            executable='test_trace_recorder_node',
            name='test_trace_recorder_node',
            output='screen',
            parameters=[{
                'trace_file': trace_file,
                'global_path_topic': '/global_path',
                'odom_topic': '/odometry/filtered',
                'imu_topic': '/imu/data',
                'terrain_cost_topic': '/terrain_cost',
                'cmd_vel_topic': '/cmd_vel',
                'safety_topic': '/safety_state',
                'motion_topic': '/motion_command',
            }],
        ),
    ])