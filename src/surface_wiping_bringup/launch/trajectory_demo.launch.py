import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Universal Robots arm type',
    )
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.56.101',
        description='Robot IP used by ur_robot_driver',
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Launch UR driver with mock hardware',
    )
    launch_driver_arg = DeclareLaunchArgument(
        'launch_driver',
        default_value='true',
        description='Include ur_robot_driver',
    )
    launch_moveit_arg = DeclareLaunchArgument(
        'launch_moveit',
        default_value='true',
        description='Include ur_moveit_config',
    )
    surface_arg = DeclareLaunchArgument(
        'surface',
        default_value='countertop',
        description='Surface trajectory to replay: countertop or mirror',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/sihan/ws_surface_wiping_clean/outputs',
        description='Output directory for saved trajectory CSVs',
    )
    trajectory_csv_arg = DeclareLaunchArgument(
        'trajectory_csv',
        default_value='',
        description='Optional explicit trajectory CSV override',
    )
    state_log_csv_arg = DeclareLaunchArgument(
        'state_log_csv',
        default_value='',
        description='Optional explicit Section 3 state log CSV override',
    )
    point_interval_arg = DeclareLaunchArgument(
        'point_interval_sec',
        default_value='0.35',
        description='Playback interval between trajectory points',
    )
    publish_period_arg = DeclareLaunchArgument(
        'publish_period_sec',
        default_value='12.0',
        description='How often to replay the display trajectory',
    )

    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    launch_driver = LaunchConfiguration('launch_driver')
    launch_moveit = LaunchConfiguration('launch_moveit')
    surface = LaunchConfiguration('surface')
    output_dir = LaunchConfiguration('output_dir')
    trajectory_csv = LaunchConfiguration('trajectory_csv')
    state_log_csv = LaunchConfiguration('state_log_csv')
    point_interval_sec = LaunchConfiguration('point_interval_sec')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_robot_driver'),
                'launch',
                'ur_control.launch.py',
            )
        ),
        condition=IfCondition(launch_driver),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_mock_hardware': use_mock_hardware,
            'launch_rviz': 'false',
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ur_moveit_config'),
                'launch',
                'ur_moveit.launch.py',
            )
        ),
        condition=IfCondition(launch_moveit),
        launch_arguments={
            'ur_type': ur_type,
            'launch_rviz': 'false',
        }.items(),
    )

    scene_setup_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='surface_wiping_nodes',
                executable='scene_setup',
                name='scene_setup',
                output='screen',
            )
        ],
    )

    playback_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='surface_wiping_nodes',
                executable='trajectory_playback',
                name='trajectory_playback',
                output='screen',
                parameters=[
                    {'surface': surface},
                    {'output_dir': output_dir},
                    {'trajectory_csv': trajectory_csv},
                    {'point_interval_sec': point_interval_sec},
                    {'publish_period_sec': publish_period_sec},
                ],
            )
        ],
    )

    return LaunchDescription([
        ur_type_arg,
        robot_ip_arg,
        use_mock_hardware_arg,
        launch_driver_arg,
        launch_moveit_arg,
        surface_arg,
        output_dir_arg,
        trajectory_csv_arg,
        state_log_csv_arg,
        point_interval_arg,
        publish_period_arg,
        ur_driver_launch,
        ur_moveit_launch,
        scene_setup_node,
        playback_node,
    ])
