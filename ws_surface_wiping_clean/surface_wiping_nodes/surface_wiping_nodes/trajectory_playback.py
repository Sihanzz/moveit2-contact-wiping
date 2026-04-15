import csv
import glob
import os
from copy import deepcopy

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import DisplayTrajectory, RobotState, RobotTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPlayback(Node):
    """Publishes a recorded joint trajectory to RViz via /display_planned_path."""

    def __init__(self):
        super().__init__('trajectory_playback')

        self.declare_parameter('output_dir', '/home/sihan/ws_surface_wiping_clean/outputs')
        self.declare_parameter('surface', 'countertop')
        self.declare_parameter('trajectory_csv', '')
        self.declare_parameter('publish_period_sec', 12.0)
        self.declare_parameter('point_interval_sec', 0.35)

        self.output_dir = self.get_parameter('output_dir').value
        self.surface = self.get_parameter('surface').value
        self.trajectory_csv = self.get_parameter('trajectory_csv').value
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.point_interval_sec = float(self.get_parameter('point_interval_sec').value)

        self.publisher = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        self.message = self.build_message()

        self.timer = self.create_timer(self.publish_period_sec, self.publish_once)
        self.publish_once()

    def resolve_csv_path(self):
        if self.trajectory_csv:
            return self.trajectory_csv if os.path.exists(self.trajectory_csv) else ''

        pattern_map = {
            'countertop': 'coverage_joint_trajectory_countertop_raster_*.csv',
            'mirror': 'coverage_joint_trajectory_mirror_spiral_*.csv',
        }
        pattern = pattern_map.get(self.surface, '')
        if not pattern:
            return ''
        matches = sorted(glob.glob(os.path.join(self.output_dir, pattern)))
        return matches[-1] if matches else ''

    def build_message(self):
        csv_path = self.resolve_csv_path()
        if not csv_path:
            raise RuntimeError(f'No trajectory CSV found for surface={self.surface}')

        with open(csv_path, newline='', encoding='utf-8') as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            raise RuntimeError(f'Empty trajectory CSV: {csv_path}')

        joint_names = rows[0]['joint_names'].split(';')
        points = []
        for index, row in enumerate(rows):
            positions = [float(value) for value in row['joint_positions'].split(';') if value]
            point = JointTrajectoryPoint()
            point.positions = positions
            t = (index + 1) * self.point_interval_sec
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            points.append(point)

        joint_traj = JointTrajectory()
        joint_traj.joint_names = joint_names
        joint_traj.points = points

        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = joint_traj

        start_state = RobotState()
        start_joint_state = JointState()
        start_joint_state.name = joint_names
        start_joint_state.position = list(points[0].positions)
        start_state.joint_state = start_joint_state

        display = DisplayTrajectory()
        display.model_id = 'ur5e'
        display.trajectory_start = start_state
        display.trajectory = [robot_traj]

        self.get_logger().info(
            f'Loaded playback trajectory: surface={self.surface} '
            f'points={len(points)} csv={csv_path} '
            f'point_interval={self.point_interval_sec:.2f}s'
        )
        return display

    def publish_once(self):
        self.publisher.publish(deepcopy(self.message))
        self.get_logger().info(
            f'Published display trajectory for {self.surface} '
            f'(republish every {self.publish_period_sec:.1f}s)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayback()
    try:
        rclpy.spin(node)
    finally:
        if node.context.ok():
            node.destroy_node()
            rclpy.shutdown()
