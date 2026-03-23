from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/sihan/ws_wiping/outputs',
        description='Directory containing controller outputs',
    )

    coverage_waypoints_csv_arg = DeclareLaunchArgument(
        'coverage_waypoints_csv',
        default_value='',
        description='Optional coverage waypoint CSV to visualize',
    )

    wiping_log_csv_arg = DeclareLaunchArgument(
        'wiping_log_csv',
        default_value='',
        description='Optional wiping log CSV to replay',
    )

    playback_rate_arg = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Playback speed multiplier for RViz replay',
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Whether to loop the replay',
    )

    return LaunchDescription([
        output_dir_arg,
        coverage_waypoints_csv_arg,
        wiping_log_csv_arg,
        playback_rate_arg,
        loop_arg,
        Node(
            package='moveit2_surface_wiping_demo',
            executable='wiping_visualization',
            name='wiping_visualization',
            output='screen',
            parameters=[
                {'output_dir': LaunchConfiguration('output_dir')},
                {'coverage_waypoints_csv': LaunchConfiguration('coverage_waypoints_csv')},
                {'wiping_log_csv': LaunchConfiguration('wiping_log_csv')},
                {'playback_rate': LaunchConfiguration('playback_rate')},
                {'loop': LaunchConfiguration('loop')},
            ],
        ),
    ])
