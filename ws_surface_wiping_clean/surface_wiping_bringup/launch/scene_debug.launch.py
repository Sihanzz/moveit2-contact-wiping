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

    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    launch_driver = LaunchConfiguration('launch_driver')
    launch_moveit = LaunchConfiguration('launch_moveit')

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

    return LaunchDescription([
        ur_type_arg,
        robot_ip_arg,
        use_mock_hardware_arg,
        launch_driver_arg,
        launch_moveit_arg,
        ur_driver_launch,
        ur_moveit_launch,
        scene_setup_node,
    ])
