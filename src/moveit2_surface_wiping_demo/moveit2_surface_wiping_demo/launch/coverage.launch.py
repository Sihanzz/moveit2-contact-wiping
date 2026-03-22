from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='ur_manipulator',
        description='MoveIt planning group name',
    )

    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='tool0',
        description='End-effector link name',
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/sihan/ws_wiping/outputs',
        description='Output directory for planner artifacts',
    )

    reachability_csv_arg = DeclareLaunchArgument(
        'reachability_csv',
        default_value='',
        description='Optional reachability CSV to use as feasibility mask',
    )

    planning_group = LaunchConfiguration('planning_group')
    end_effector_link = LaunchConfiguration('end_effector_link')
    output_dir = LaunchConfiguration('output_dir')
    reachability_csv = LaunchConfiguration('reachability_csv')

    scene_setup_node = Node(
        package='moveit2_surface_wiping_demo',
        executable='scene_setup',
        name='scene_setup',
        output='screen',
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
        ],
    )

    coverage_planner_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='moveit2_surface_wiping_demo',
                executable='coverage_planner',
                name='coverage_planner',
                output='screen',
                parameters=[
                    {'output_dir': output_dir},
                    {'reachability_csv': reachability_csv},
                ],
            )
        ],
    )

    return LaunchDescription([
        planning_group_arg,
        end_effector_link_arg,
        output_dir_arg,
        reachability_csv_arg,
        scene_setup_node,
        ik_service_node,
        coverage_planner_node,
    ])
