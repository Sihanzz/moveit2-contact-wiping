from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/sihan/ws_wiping/outputs',
        description='Output directory for controller artifacts',
    )

    coverage_waypoints_csv_arg = DeclareLaunchArgument(
        'coverage_waypoints_csv',
        default_value='',
        description='Optional coverage waypoint CSV to execute',
    )

    output_dir = LaunchConfiguration('output_dir')
    coverage_waypoints_csv = LaunchConfiguration('coverage_waypoints_csv')

    scene_setup_node = Node(
        package='moveit2_surface_wiping_demo',
        executable='scene_setup',
        name='scene_setup',
        output='screen',
    )

    wiping_controller_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='moveit2_surface_wiping_demo',
                executable='wiping_controller',
                name='wiping_controller',
                output='screen',
                parameters=[
                    {'output_dir': output_dir},
                    {'coverage_waypoints_csv': coverage_waypoints_csv},
                ],
            )
        ],
    )

    return LaunchDescription([
        output_dir_arg,
        coverage_waypoints_csv_arg,
        scene_setup_node,
        wiping_controller_node,
    ])
