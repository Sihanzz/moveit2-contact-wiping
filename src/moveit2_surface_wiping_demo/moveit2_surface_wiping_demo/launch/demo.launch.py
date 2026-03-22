from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for surface wiping demo.

    This launches:
    1. Scene setup node - publishes collision objects
    2. Reachability map node - generates reachability analysis

    Prerequisites:
    - MoveIt2 move_group node should be running
    - Robot description should be loaded
    """

    # Declare arguments
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='ur_manipulator',
        description='MoveIt planning group name'
    )

    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='tool0',
        description='End-effector link name'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/sihan/ws_wiping/outputs',
        description='Output directory for CSV and heatmap'
    )

    # Get launch configurations
    planning_group = LaunchConfiguration('planning_group')
    end_effector_link = LaunchConfiguration('end_effector_link')
    output_dir = LaunchConfiguration('output_dir')

    # Scene setup node - publishes collision objects
    scene_setup_node = Node(
        package='moveit2_surface_wiping_demo',
        executable='scene_setup',
        name='scene_setup',
        output='screen',
        parameters=[],
    )

    ik_service_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='moveit2_surface_wiping_demo',
                executable='ik_service',
                name='ik_service',
                output='screen',
                parameters=[
                    {'planning_group': planning_group},
                    {'end_effector_link': end_effector_link},
                ],
            )
        ]
    )

    # Reachability map node - generates analysis
    # Delayed start to ensure scene is set up first
    reachability_map_node = TimerAction(
        period=5.0,  # Wait 5 seconds for scene to be published
        actions=[
            Node(
                package='moveit2_surface_wiping_demo',
                executable='reachability_map',
                name='reachability_map',
                output='screen',
                parameters=[
                    {'output_dir': output_dir},
                ],
            )
        ]
    )

    return LaunchDescription([
        planning_group_arg,
        end_effector_link_arg,
        output_dir_arg,
        scene_setup_node,
        ik_service_node,
        reachability_map_node,
    ])
