from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/sihan/ws_surface_wiping_clean/outputs',
        description='Output directory for Section 3 artifacts',
    )
    countertop_waypoints_arg = DeclareLaunchArgument(
        'countertop_waypoints_csv',
        default_value='',
        description='Optional countertop waypoint CSV override',
    )
    mirror_waypoints_arg = DeclareLaunchArgument(
        'mirror_waypoints_csv',
        default_value='',
        description='Optional mirror waypoint CSV override',
    )

    output_dir = LaunchConfiguration('output_dir')
    countertop_waypoints_csv = LaunchConfiguration('countertop_waypoints_csv')
    mirror_waypoints_csv = LaunchConfiguration('mirror_waypoints_csv')

    controller_node = Node(
        package='surface_wiping_nodes',
        executable='contact_wiping_controller',
        name='contact_wiping_controller',
        output='screen',
        parameters=[
            {'output_dir': output_dir},
            {'countertop_waypoints_csv': countertop_waypoints_csv},
            {'mirror_waypoints_csv': mirror_waypoints_csv},
        ],
    )

    return LaunchDescription([
        output_dir_arg,
        countertop_waypoints_arg,
        mirror_waypoints_arg,
        controller_node,
    ])
