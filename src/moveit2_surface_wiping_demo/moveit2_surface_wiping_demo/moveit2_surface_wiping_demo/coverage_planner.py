import csv
import glob
import math
import os
import threading
from collections import Counter
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from moveit2_surface_wiping_interfaces.srv import ReachabilityQuery


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')
        self.callback_group = ReentrantCallbackGroup()

        package_share = get_package_share_directory('moveit2_surface_wiping_demo')
        scene_path = os.path.join(package_share, 'config', 'scene.yaml')
        reachability_path = os.path.join(package_share, 'config', 'reachability.yaml')
        coverage_path = os.path.join(package_share, 'config', 'coverage.yaml')

        with open(scene_path, 'r') as handle:
            self.scene_config = yaml.safe_load(handle)
        with open(reachability_path, 'r') as handle:
            self.reachability_config = yaml.safe_load(handle)
        with open(coverage_path, 'r') as handle:
            self.coverage_config = yaml.safe_load(handle)

        self.declare_parameter('output_dir', '/home/sihan/ws_wiping/outputs')
        self.declare_parameter('reachability_csv', '')
        self.declare_parameter('service_timeout_sec', 5.0)

        self.output_dir = self.get_parameter('output_dir').value
        self.reachability_csv = self.get_parameter('reachability_csv').value
        self.service_timeout_sec = self.get_parameter('service_timeout_sec').value

        self.surface_name = self.coverage_config['surface']
        self.surface = self.scene_config[self.surface_name]
        self.base_frame = self.scene_config['world_frame']

        top_surface = self.surface['position'][2] + self.surface['size'][2] / 2.0
        self.surface_z = top_surface + self.reachability_config['surface_z_offset']

        self.pad_length = float(self.coverage_config['pad_length'])
        self.pad_width = float(self.coverage_config['pad_width'])
        self.keepout_margin = float(self.coverage_config['keepout_margin'])
        self.overlap = float(self.coverage_config['overlap'])
        self.query_yaw = float(self.coverage_config['query_yaw'])
        self.enforce_yaw = bool(self.coverage_config['enforce_yaw'])
        self.sweep_axis = self.coverage_config['sweep_axis']
        self.waypoint_step = float(self.coverage_config['waypoint_step'])
        self.joint_speed_rad_s = float(self.coverage_config['joint_speed_rad_s'])

        self.patch_center = np.array(self.reachability_config['patch_center'], dtype=float)
        self.patch_size = np.array(self.reachability_config['patch_size'], dtype=float)
        self.eval_resolution = float(self.reachability_config['resolution'])

        self.client = self.create_client(
            ReachabilityQuery,
            'reachability_query',
            callback_group=self.callback_group,
        )

        self.generated = False
        self.in_progress = False

        self.get_logger().info(f'Coverage planner configured for surface={self.surface_name}')
        self.get_logger().info(f'Pad={self.pad_length:.3f} x {self.pad_width:.3f} m, overlap={self.overlap:.2f}')
        os.makedirs(self.output_dir, exist_ok=True)

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Reachability service not available')
            return

        self.create_timer(2.0, self.plan_once, callback_group=self.callback_group)

    def plan_once(self):
        if self.generated or self.in_progress:
            return

        self.in_progress = True
        self.get_logger().info('Starting raster coverage planning...')

        csv_mask = self.load_reachability_mask_from_csv()
        if csv_mask is not None:
            x_coords, y_coords, reachability_grid, solution_lookup = csv_mask
            failure_counts = Counter()
        else:
            x_coords, y_coords, reachability_grid, solution_lookup, failure_counts = self.build_feasibility_mask()
        total_cells = reachability_grid.size
        reachable_cells = int(np.sum(reachability_grid))
        self.get_logger().info(
            f'Feasibility mask complete: reachable cells={reachable_cells}/{total_cells} '
            f'failures={dict(failure_counts)}'
        )

        waypoints, joint_trajectory = self.generate_raster_waypoints(
            x_coords,
            y_coords,
            reachability_grid,
            solution_lookup,
        )
        planned_points = waypoints
        self.get_logger().info(f'Generated {len(waypoints)} reachable raster waypoints')

        metrics = self.compute_metrics(planned_points, joint_trajectory)
        self.export_waypoints(planned_points, joint_trajectory, metrics)
        self.save_plot(x_coords, y_coords, reachability_grid, planned_points)

        self.get_logger().info(
            f'Coverage planning complete: reachable waypoints={len(planned_points)}/{len(waypoints)}, '
            f'coverage={metrics["coverage_percent"]:.1f}%, '
            f'path_length={metrics["path_length_m"]:.3f} m, '
            f'est_time={metrics["estimated_time_s"]:.1f} s'
        )
        self.get_logger().info(f'Failure breakdown: {dict(failure_counts)}')

        self.generated = True
        self.in_progress = False

    def resolve_reachability_csv(self):
        if self.reachability_csv:
            return self.reachability_csv if os.path.exists(self.reachability_csv) else None

        pattern = os.path.join(self.output_dir, 'reachability_map_*.csv')
        matches = sorted(glob.glob(pattern))
        return matches[-1] if matches else None

    def load_reachability_mask_from_csv(self):
        csv_path = self.resolve_reachability_csv()
        if not csv_path:
            self.get_logger().info('No reachability CSV found, recomputing feasibility mask online')
            return None

        self.get_logger().info(f'Loading feasibility mask from: {csv_path}')
        rows = []
        with open(csv_path, newline='') as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows.append(row)

        if not rows:
            self.get_logger().warn('Reachability CSV is empty, recomputing feasibility mask online')
            return None

        x_coords = np.array(sorted({float(row['x']) for row in rows}), dtype=float)
        y_coords = np.array(sorted({float(row['y']) for row in rows}), dtype=float)
        x_index = {round(x, 6): idx for idx, x in enumerate(x_coords)}
        y_index = {round(y, 6): idx for idx, y in enumerate(y_coords)}

        reachability_grid = np.zeros((len(x_coords), len(y_coords)), dtype=bool)
        solution_lookup = {}
        for row in rows:
            x = float(row['x'])
            y = float(row['y'])
            reachable = int(row['reachable'])
            if reachable != 1:
                continue
            reachability_grid[x_index[round(x, 6)], y_index[round(y, 6)]] = True
            if row.get('joint_names') and row.get('joint_positions'):
                joint_state = JointState()
                joint_state.name = row['joint_names'].split(';')
                joint_state.position = [float(value) for value in row['joint_positions'].split(';') if value]
                solution_lookup[(round(x, 6), round(y, 6))] = joint_state

        missing_solutions = int(np.sum(reachability_grid)) - len(solution_lookup)
        if missing_solutions > 0:
            self.get_logger().warn(
                f'Reachability CSV is missing {missing_solutions} joint solutions, '
                f'falling back to online reconstruction for those cells'
            )
            self.fill_missing_solutions(x_coords, y_coords, reachability_grid, solution_lookup)

        return self.apply_keepout_margin(x_coords, y_coords, reachability_grid, solution_lookup)

    def apply_keepout_margin(self, x_coords, y_coords, reachability_grid, solution_lookup):
        patch_x_min = self.patch_center[0] - self.patch_size[0] / 2.0
        patch_x_max = self.patch_center[0] + self.patch_size[0] / 2.0
        patch_y_min = self.patch_center[1] - self.patch_size[1] / 2.0
        patch_y_max = self.patch_center[1] + self.patch_size[1] / 2.0

        x_min = patch_x_min + self.keepout_margin
        x_max = patch_x_max - self.keepout_margin
        y_min = patch_y_min + self.keepout_margin
        y_max = patch_y_max - self.keepout_margin

        x_mask = (x_coords >= x_min - 1e-9) & (x_coords <= x_max + 1e-9)
        y_mask = (y_coords >= y_min - 1e-9) & (y_coords <= y_max + 1e-9)

        trimmed_x = x_coords[x_mask]
        trimmed_y = y_coords[y_mask]
        trimmed_grid = reachability_grid[np.ix_(x_mask, y_mask)]
        trimmed_lookup = {}

        for x_idx, x in enumerate(trimmed_x):
            for y_idx, y in enumerate(trimmed_y):
                if not trimmed_grid[x_idx, y_idx]:
                    continue
                key = (round(float(x), 6), round(float(y), 6))
                if key in solution_lookup:
                    trimmed_lookup[key] = solution_lookup[key]

        return trimmed_x, trimmed_y, trimmed_grid, trimmed_lookup

    def fill_missing_solutions(self, x_coords, y_coords, reachability_grid, solution_lookup):
        reachable_cells = int(np.sum(reachability_grid))
        processed = 0

        for x_idx, x in enumerate(x_coords):
            for y_idx, y in enumerate(y_coords):
                if not reachability_grid[x_idx, y_idx]:
                    continue
                key = (round(x, 6), round(y, 6))
                if key in solution_lookup:
                    continue
                point = (float(x), float(y), float(self.surface_z))
                reachable, solution, failure_reason = self.query_ik(point)
                processed += 1
                if reachable:
                    solution_lookup[key] = solution
                else:
                    reachability_grid[x_idx, y_idx] = False
                    self.get_logger().warn(
                        f'Cached reachable cell failed when reconstructing joint solution: '
                        f'({x:.3f}, {y:.3f}, {self.surface_z:.3f}) reason={failure_reason}'
                    )
                if processed % 50 == 0:
                    self.get_logger().info(
                        f'Joint solution reconstruction: {processed}/{reachable_cells}'
                    )

    def build_feasibility_mask(self):
        patch_x_min = self.patch_center[0] - self.patch_size[0] / 2.0
        patch_x_max = self.patch_center[0] + self.patch_size[0] / 2.0
        patch_y_min = self.patch_center[1] - self.patch_size[1] / 2.0
        patch_y_max = self.patch_center[1] + self.patch_size[1] / 2.0

        x_min = patch_x_min + self.keepout_margin
        x_max = patch_x_max - self.keepout_margin
        y_min = patch_y_min + self.keepout_margin
        y_max = patch_y_max - self.keepout_margin

        x_lattice = np.arange(
            patch_x_min,
            patch_x_max + self.eval_resolution / 2.0,
            self.eval_resolution,
        )
        y_lattice = np.arange(
            patch_y_min,
            patch_y_max + self.eval_resolution / 2.0,
            self.eval_resolution,
        )

        xs = x_lattice[(x_lattice >= x_min - 1e-9) & (x_lattice <= x_max + 1e-9)]
        ys = y_lattice[(y_lattice >= y_min - 1e-9) & (y_lattice <= y_max + 1e-9)]

        reachability_grid = np.zeros((len(xs), len(ys)), dtype=bool)
        solution_lookup = {}
        failure_counts = Counter()

        total = len(xs) * len(ys)
        processed = 0
        for x_idx, x in enumerate(xs):
            for y_idx, y in enumerate(ys):
                point = (float(x), float(y), float(self.surface_z))
                reachable, solution, failure_reason = self.query_ik(point)
                if reachable:
                    reachability_grid[x_idx, y_idx] = True
                    solution_lookup[(round(x, 6), round(y, 6))] = solution
                else:
                    failure_counts[failure_reason] += 1

                processed += 1
                if processed % 50 == 0:
                    self.get_logger().info(
                        f'Feasibility progress: {processed}/{total} - '
                        f'reachable={int(np.sum(reachability_grid))} '
                        f'failures={dict(failure_counts)}'
                    )

        return xs, ys, reachability_grid, solution_lookup, failure_counts

    def generate_raster_waypoints(self, xs, ys, reachability_grid, solution_lookup):
        x_min = xs[0]
        x_max = xs[-1]
        y_min = ys[0]
        y_max = ys[-1]

        stripe_spacing = self.pad_width * (1.0 - self.overlap)
        stripe_spacing = max(stripe_spacing, self.eval_resolution)
        stripe_stride = max(int(round(stripe_spacing / self.eval_resolution)), 1)
        y_indices = np.arange(0, len(ys), stripe_stride)

        point_stride = max(int(round(self.waypoint_step / self.eval_resolution)), 1)
        x_indices = np.arange(0, len(xs), point_stride)

        waypoints = []
        joint_trajectory = []
        if self.sweep_axis == 'y':
            for stripe_idx, x_idx in enumerate(x_indices):
                stripe_y_indices = y_indices if stripe_idx % 2 == 0 else y_indices[::-1]
                for y_idx in stripe_y_indices:
                    if not reachability_grid[x_idx, y_idx]:
                        continue
                    x = float(xs[x_idx])
                    y = float(ys[y_idx])
                    waypoints.append((x, y, float(self.surface_z)))
                    joint_trajectory.append(solution_lookup[(round(x, 6), round(y, 6))])
        else:
            for stripe_idx, y_idx in enumerate(y_indices):
                stripe_x_indices = x_indices if stripe_idx % 2 == 0 else x_indices[::-1]
                for x_idx in stripe_x_indices:
                    if not reachability_grid[x_idx, y_idx]:
                        continue
                    x = float(xs[x_idx])
                    y = float(ys[y_idx])
                    waypoints.append((x, y, float(self.surface_z)))
                    joint_trajectory.append(solution_lookup[(round(x, 6), round(y, 6))])
        return waypoints, joint_trajectory

    def query_ik(self, point):
        request = ReachabilityQuery.Request()
        request.target_point = Point(x=point[0], y=point[1], z=point[2])
        request.frame_id = self.base_frame
        request.yaw = self.query_yaw
        request.enforce_yaw = self.enforce_yaw

        future = self.client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=self.service_timeout_sec):
            return False, None, 'service_call_timeout'

        response = future.result()
        if response is None:
            return False, None, 'service_call_failed'
        if response.reachable:
            return True, response.solution, ''
        return False, None, response.failure_reason or 'ik_failed'

    def compute_metrics(self, planned_points, joint_trajectory):
        coverage_percent = self.compute_coverage_percent(planned_points)
        path_length_m = 0.0
        for first, second in zip(planned_points, planned_points[1:]):
            path_length_m += math.dist(first[:3], second[:3])

        estimated_time_s = 0.0
        for first, second in zip(joint_trajectory, joint_trajectory[1:]):
            first_map = dict(zip(first.name, first.position))
            second_map = dict(zip(second.name, second.position))
            common_names = [name for name in first.name if name in second_map]
            if not common_names:
                continue
            max_delta = max(abs(second_map[name] - first_map[name]) for name in common_names)
            estimated_time_s += max_delta / max(self.joint_speed_rad_s, 1e-6)

        return {
            'coverage_percent': coverage_percent,
            'path_length_m': path_length_m,
            'estimated_time_s': estimated_time_s,
            'reachable_waypoints': len(planned_points),
        }

    def compute_coverage_percent(self, planned_points):
        if not planned_points:
            return 0.0

        x_min = self.patch_center[0] - self.patch_size[0] / 2.0 + self.keepout_margin
        x_max = self.patch_center[0] + self.patch_size[0] / 2.0 - self.keepout_margin
        y_min = self.patch_center[1] - self.patch_size[1] / 2.0 + self.keepout_margin
        y_max = self.patch_center[1] + self.patch_size[1] / 2.0 - self.keepout_margin

        x_coords = np.arange(x_min, x_max + self.eval_resolution / 2.0, self.eval_resolution)
        y_coords = np.arange(y_min, y_max + self.eval_resolution / 2.0, self.eval_resolution)

        covered = 0
        total = len(x_coords) * len(y_coords)
        half_length = self.pad_length / 2.0
        half_width = self.pad_width / 2.0

        for x in x_coords:
            for y in y_coords:
                is_covered = any(
                    abs(x - waypoint[0]) <= half_length and abs(y - waypoint[1]) <= half_width
                    for waypoint in planned_points
                )
                if is_covered:
                    covered += 1

        return 100.0 * covered / max(total, 1)

    def export_waypoints(self, planned_points, joint_trajectory, metrics):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(self.output_dir, exist_ok=True)

        metrics_path = os.path.join(self.output_dir, f'coverage_metrics_{timestamp}.csv')
        with open(metrics_path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['metric', 'value'])
            for key, value in metrics.items():
                writer.writerow([key, value])

        waypoint_path = os.path.join(self.output_dir, f'coverage_waypoints_{timestamp}.csv')
        with open(waypoint_path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['index', 'x', 'y', 'z'])
            for idx, point in enumerate(planned_points):
                writer.writerow([idx, point[0], point[1], point[2]])

        trajectory_path = os.path.join(self.output_dir, f'joint_trajectory_{timestamp}.csv')
        joint_names = joint_trajectory[0].name if joint_trajectory else []
        cumulative_time = 0.0
        with open(trajectory_path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['index', 'time_from_start_s', *joint_names])
            previous = None
            for idx, joint_state in enumerate(joint_trajectory):
                if previous is not None:
                    deltas = [abs(a - b) for a, b in zip(joint_state.position, previous.position)]
                    cumulative_time += max(deltas, default=0.0) / max(self.joint_speed_rad_s, 1e-6)
                writer.writerow([idx, cumulative_time, *joint_state.position])
                previous = joint_state

        self.get_logger().info(f'Metrics exported to: {metrics_path}')
        self.get_logger().info(f'Waypoints exported to: {waypoint_path}')
        self.get_logger().info(f'Joint trajectory exported to: {trajectory_path}')

    def save_plot(self, x_coords, y_coords, reachability_grid, planned_points):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        plot_path = os.path.join(self.output_dir, f'coverage_plan_{timestamp}.png')

        fig, ax = plt.subplots(figsize=(8, 6))
        reachable_x = []
        reachable_y = []
        for x_idx, x in enumerate(x_coords):
            for y_idx, y in enumerate(y_coords):
                if reachability_grid[x_idx, y_idx]:
                    reachable_x.append(x)
                    reachable_y.append(y)
        ax.scatter(reachable_x, reachable_y, s=12, c='#d0d0d0', label='Reachable mask cells')

        if planned_points:
            plan_x = [point[0] for point in planned_points]
            plan_y = [point[1] for point in planned_points]
            ax.plot(plan_x, plan_y, color='#006d77', linewidth=1.5, label='Reachable raster path')
            ax.scatter(plan_x, plan_y, s=16, c='#ee6c4d')

        x_min = self.patch_center[0] - self.patch_size[0] / 2.0 + self.keepout_margin
        x_max = self.patch_center[0] + self.patch_size[0] / 2.0 - self.keepout_margin
        y_min = self.patch_center[1] - self.patch_size[1] / 2.0 + self.keepout_margin
        y_max = self.patch_center[1] + self.patch_size[1] / 2.0 - self.keepout_margin
        ax.set_xlim(x_min - 0.05, x_max + 0.05)
        ax.set_ylim(y_min - 0.05, y_max + 0.05)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_title(f'Raster Coverage Plan - {self.surface_name}')
        ax.legend()
        ax.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Coverage plot saved to: {plot_path}')


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
