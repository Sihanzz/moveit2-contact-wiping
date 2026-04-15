import csv
import glob
import math
import os
import threading
from datetime import datetime
from collections import Counter

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from surface_wiping_interfaces.srv import SolveSurfaceIK


AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}


def normalize(vector):
    norm = np.linalg.norm(vector)
    if norm < 1e-9:
        raise ValueError('zero-length vector')
    return vector / norm


def rotation_matrix_from_axis_angle(axis, angle):
    axis = normalize(np.asarray(axis, dtype=float))
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array([
        [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
        [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
        [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
    ], dtype=float)


def quaternion_from_rotation_matrix(rotation):
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        qw = (rotation[2, 1] - rotation[1, 2]) / scale
        qx = 0.25 * scale
        qy = (rotation[0, 1] + rotation[1, 0]) / scale
        qz = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        qw = (rotation[0, 2] - rotation[2, 0]) / scale
        qx = (rotation[0, 1] + rotation[1, 0]) / scale
        qy = 0.25 * scale
        qz = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        qw = (rotation[1, 0] - rotation[0, 1]) / scale
        qx = (rotation[0, 2] + rotation[2, 0]) / scale
        qy = (rotation[1, 2] + rotation[2, 1]) / scale
        qz = 0.25 * scale
    return qx, qy, qz, qw


class CoveragePlanner(Node):
    """Generates countertop raster and mirror spiral wiping plans."""

    def __init__(self):
        super().__init__('coverage_planner')

        package_share = get_package_share_directory('surface_wiping_nodes')
        scene_path = os.path.join(package_share, 'config', 'scene.yaml')
        coverage_path = os.path.join(package_share, 'config', 'coverage.yaml')

        with open(scene_path, 'r', encoding='utf-8') as handle:
            self.scene_config = yaml.safe_load(handle)
        with open(coverage_path, 'r', encoding='utf-8') as handle:
            self.coverage_config = yaml.safe_load(handle)

        self.declare_parameter('output_dir', '/home/sihan/ws_surface_wiping_clean/outputs')
        self.declare_parameter('reachability_csv', '')
        self.declare_parameter('service_name', 'solve_surface_ik')
        self.declare_parameter('service_timeout_sec', 5.0)

        self.output_dir = self.get_parameter('output_dir').value
        self.reachability_csv = self.get_parameter('reachability_csv').value
        self.service_name = self.get_parameter('service_name').value
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)

        self.pad_length = float(self.coverage_config['pad_length'])
        self.pad_width = float(self.coverage_config['pad_width'])
        self.keepout_margin = float(self.coverage_config['keepout_margin'])
        self.overlap = float(self.coverage_config['overlap'])
        self.waypoint_step = float(self.coverage_config['waypoint_step'])
        self.joint_speed_rad_s = float(self.coverage_config['joint_speed_rad_s'])

        os.makedirs(self.output_dir, exist_ok=True)

        self.client_group = ReentrantCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()

        self.ik_client = self.create_client(
            SolveSurfaceIK,
            self.service_name,
            callback_group=self.client_group,
        )
        self.completed = False
        self.in_progress = False
        self.create_timer(1.0, self.plan_once, callback_group=self.timer_group)

    def plan_once(self):
        if self.completed or self.in_progress:
            return
        self.in_progress = True

        try:
            reachability_data = self.load_reachability_data()
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            comparison_rows = []

            for surface_name, surface_cfg in self.coverage_config['surfaces'].items():
                self.get_logger().info(
                    f"Planning surface={surface_name} strategy={surface_cfg['strategy']}"
                )
                plan = self.plan_surface(surface_name, surface_cfg, reachability_data)
                self.export_surface_outputs(surface_name, timestamp, plan)
                comparison_rows.append(plan['metrics'])
                self.get_logger().info(
                    f"Surface complete: {surface_name} strategy={plan['metrics']['strategy']} "
                    f"waypoints={plan['metrics']['waypoints']} "
                    f"coverage={plan['metrics']['coverage_percent']:.1f}% "
                    f"path_length={plan['metrics']['path_length_m']:.3f} m "
                    f"est_time={plan['metrics']['estimated_time_s']:.1f} s"
                )

            self.export_comparison(timestamp, comparison_rows)
            self.completed = True
        finally:
            self.in_progress = False

    def load_reachability_data(self):
        csv_path = self.resolve_reachability_csv()
        if not csv_path:
            self.get_logger().warn('No reachability CSV found, countertop planning will not use cached solutions')
            return None

        with open(csv_path, newline='', encoding='utf-8') as handle:
            rows = list(csv.DictReader(handle))

        x_coords = np.array(sorted({float(row['x']) for row in rows}), dtype=float)
        y_coords = np.array(sorted({float(row['y']) for row in rows}), dtype=float)
        x_index = {round(x, 6): idx for idx, x in enumerate(x_coords)}
        y_index = {round(y, 6): idx for idx, y in enumerate(y_coords)}
        reachability_grid = np.zeros((len(x_coords), len(y_coords)), dtype=bool)
        solution_lookup = {}

        for row in rows:
            x = float(row['x'])
            y = float(row['y'])
            if int(row['reachable']) != 1:
                continue
            reachability_grid[x_index[round(x, 6)], y_index[round(y, 6)]] = True
            if row.get('joint_names') and row.get('joint_positions'):
                joint_state = JointState()
                joint_state.name = row['joint_names'].split(';')
                joint_state.position = [float(v) for v in row['joint_positions'].split(';') if v]
                solution_lookup[(round(x, 6), round(y, 6))] = joint_state

        return {
            'csv_path': csv_path,
            'x_coords': x_coords,
            'y_coords': y_coords,
            'grid': reachability_grid,
            'lookup': solution_lookup,
        }

    def resolve_reachability_csv(self):
        if self.reachability_csv:
            return self.reachability_csv if os.path.exists(self.reachability_csv) else ''
        matches = sorted(glob.glob(os.path.join(self.output_dir, 'reachability_map_*.csv')))
        return matches[-1] if matches else ''

    def plan_surface(self, surface_name, surface_cfg, reachability_data):
        strategy = surface_cfg['strategy']
        if surface_name == 'countertop' and reachability_data is not None:
            return self.plan_countertop_with_reachability(surface_name, surface_cfg, reachability_data)
        return self.plan_surface_online(surface_name, surface_cfg, strategy)

    def plan_countertop_with_reachability(self, surface_name, surface_cfg, reachability_data):
        x_coords = reachability_data['x_coords']
        y_coords = reachability_data['y_coords']
        reachability_grid = reachability_data['grid'].copy()
        solution_lookup = dict(reachability_data['lookup'])

        patch_center = np.array(surface_cfg['patch_center'], dtype=float)
        patch_size = np.array(surface_cfg['patch_size'], dtype=float)
        x_min = patch_center[0] - patch_size[0] / 2.0 + self.keepout_margin
        x_max = patch_center[0] + patch_size[0] / 2.0 - self.keepout_margin
        y_min = patch_center[1] - patch_size[1] / 2.0 + self.keepout_margin
        y_max = patch_center[1] + patch_size[1] / 2.0 - self.keepout_margin

        x_mask = (x_coords >= x_min - 1e-9) & (x_coords <= x_max + 1e-9)
        y_mask = (y_coords >= y_min - 1e-9) & (y_coords <= y_max + 1e-9)
        xs = x_coords[x_mask]
        ys = y_coords[y_mask]
        grid = reachability_grid[np.ix_(x_mask, y_mask)]

        waypoints, joint_trajectory = self.generate_raster_waypoints(
            xs, ys, grid, solution_lookup, surface_name, surface_cfg
        )

        metrics = self.compute_metrics(
            surface_name, surface_cfg['strategy'], patch_size, xs, ys, grid, waypoints, joint_trajectory
        )

        return {
            'surface': surface_name,
            'strategy': surface_cfg['strategy'],
            'x_coords': xs,
            'y_coords': ys,
            'grid': grid,
            'waypoints': waypoints,
            'joint_trajectory': joint_trajectory,
            'metrics': metrics,
        }

    def plan_surface_online(self, surface_name, surface_cfg, strategy):
        patch_center = np.array(surface_cfg['patch_center'], dtype=float)
        patch_size = np.array(surface_cfg['patch_size'], dtype=float)
        if strategy == 'spiral':
            waypoints = self.generate_spiral_waypoints(surface_name, surface_cfg)
        elif strategy == 'raster':
            waypoints = self.generate_online_raster_waypoints(surface_name, surface_cfg)
        else:
            raise ValueError(f'Unsupported strategy: {strategy}')

        feasible_waypoints = []
        joint_trajectory = []
        failure_counts = Counter()
        total = len(waypoints)
        for index, waypoint in enumerate(waypoints, start=1):
            pose, response = self.solve_waypoint(surface_cfg, waypoint)
            if response is None or not response.success:
                failure_counts[(response.message if response else 'service_call_timeout')] += 1
                if index % 25 == 0 or index == total:
                    self.get_logger().info(
                        f'{surface_name} progress {index}/{total} '
                        f'accepted={len(feasible_waypoints)} '
                        f'rejected={index - len(feasible_waypoints)} '
                        f'failures={dict(failure_counts)}'
                    )
                continue
            feasible_waypoints.append(waypoint)
            joint_trajectory.append({
                'point': waypoint,
                'joint_state': response.joint_state,
            })
            if index % 25 == 0 or index == total:
                self.get_logger().info(
                    f'{surface_name} progress {index}/{total} '
                    f'accepted={len(feasible_waypoints)} '
                    f'rejected={index - len(feasible_waypoints)} '
                    f'failures={dict(failure_counts)}'
                )

        metrics = self.compute_metrics(
            surface_name,
            strategy,
            patch_size,
            np.array([]),
            np.array([]),
            np.array([]),
            feasible_waypoints,
            joint_trajectory,
        )

        return {
            'surface': surface_name,
            'strategy': strategy,
            'x_coords': np.array([]),
            'y_coords': np.array([]),
            'grid': np.array([]),
            'waypoints': feasible_waypoints,
            'joint_trajectory': joint_trajectory,
            'metrics': metrics,
        }

    def generate_raster_waypoints(self, x_coords, y_coords, grid, solution_lookup, surface_name, surface_cfg):
        row_step = max(self.waypoint_step, self.pad_width * (1.0 - self.overlap))
        y_resolution = abs(y_coords[1] - y_coords[0]) if len(y_coords) > 1 else self.waypoint_step
        row_stride = max(1, int(round(row_step / y_resolution)))
        selected_y_indices = list(range(0, len(y_coords), row_stride))
        if selected_y_indices and selected_y_indices[-1] != len(y_coords) - 1:
            selected_y_indices.append(len(y_coords) - 1)

        waypoints = []
        joint_trajectory = []
        direction = 1
        for y_idx in selected_y_indices:
            reachable_x = [x_idx for x_idx in range(len(x_coords)) if grid[x_idx, y_idx]]
            if not reachable_x:
                continue
            ordered = reachable_x if direction > 0 else list(reversed(reachable_x))
            for x_idx in ordered:
                x = float(x_coords[x_idx])
                y = float(y_coords[y_idx])
                key = (round(x, 6), round(y, 6))
                joint_state = solution_lookup.get(key)
                if joint_state is None:
                    point = {'x': x, 'y': y, 'z': self.surface_height(surface_name, surface_cfg)}
                    _, response = self.solve_waypoint(surface_cfg, point)
                    if response is None or not response.success:
                        continue
                    joint_state = response.joint_state
                point = {'x': x, 'y': y, 'z': self.surface_height(surface_name, surface_cfg)}
                waypoints.append(point)
                joint_trajectory.append({'point': point, 'joint_state': joint_state})
            direction *= -1
        return waypoints, joint_trajectory

    def generate_online_raster_waypoints(self, surface_name, surface_cfg):
        patch_center = np.array(surface_cfg['patch_center'], dtype=float)
        patch_size = np.array(surface_cfg['patch_size'], dtype=float)
        y_min = patch_center[1] - patch_size[1] / 2.0 + self.keepout_margin
        y_max = patch_center[1] + patch_size[1] / 2.0 - self.keepout_margin
        x_min = patch_center[0] - patch_size[0] / 2.0 + self.keepout_margin
        x_max = patch_center[0] + patch_size[0] / 2.0 - self.keepout_margin

        row_step = max(self.waypoint_step, self.pad_width * (1.0 - self.overlap))
        xs = np.arange(x_min, x_max + self.waypoint_step / 2.0, self.waypoint_step)
        ys = np.arange(y_min, y_max + row_step / 2.0, row_step)

        waypoints = []
        direction = 1
        for y in ys:
            ordered_xs = xs if direction > 0 else list(reversed(xs))
            for x in ordered_xs:
                waypoints.append({
                    'x': float(x),
                    'y': float(y),
                    'z': self.surface_height(surface_name, surface_cfg),
                })
            direction *= -1
        return waypoints

    def generate_spiral_waypoints(self, surface_name, surface_cfg):
        center = np.array(surface_cfg['patch_center'], dtype=float)
        size = np.array(surface_cfg['patch_size'], dtype=float)
        plane_axes = surface_cfg['plane_axes']
        axis_u = plane_axes[0]
        axis_v = plane_axes[1]
        u_min = center[AXIS_INDEX[axis_u]] - size[0] / 2.0 + self.keepout_margin
        u_max = center[AXIS_INDEX[axis_u]] + size[0] / 2.0 - self.keepout_margin
        v_min = center[AXIS_INDEX[axis_v]] - size[1] / 2.0 + self.keepout_margin
        v_max = center[AXIS_INDEX[axis_v]] + size[1] / 2.0 - self.keepout_margin

        step = max(self.waypoint_step, self.pad_width * (1.0 - self.overlap))
        waypoints = []

        while u_min <= u_max and v_min <= v_max:
            for u in np.arange(u_min, u_max + self.waypoint_step / 2.0, self.waypoint_step):
                waypoints.append(self.make_plane_point(surface_name, surface_cfg, axis_u, axis_v, u, v_min))
            for v in np.arange(v_min + step, v_max + self.waypoint_step / 2.0, self.waypoint_step):
                waypoints.append(self.make_plane_point(surface_name, surface_cfg, axis_u, axis_v, u_max, v))
            if v_min < v_max:
                for u in np.arange(u_max - self.waypoint_step, u_min - self.waypoint_step / 2.0, -self.waypoint_step):
                    waypoints.append(self.make_plane_point(surface_name, surface_cfg, axis_u, axis_v, u, v_max))
            if u_min < u_max:
                for v in np.arange(v_max - self.waypoint_step, v_min + step - self.waypoint_step / 2.0, -self.waypoint_step):
                    waypoints.append(self.make_plane_point(surface_name, surface_cfg, axis_u, axis_v, u_min, v))
            u_min += step
            u_max -= step
            v_min += step
            v_max -= step

        return waypoints

    def make_plane_point(self, surface_name, surface_cfg, axis_u, axis_v, u, v):
        center = np.array(surface_cfg['patch_center'], dtype=float)
        point = {
            'x': float(center[0]),
            'y': float(center[1]),
            'z': float(center[2]),
        }
        point[axis_u] = float(u)
        point[axis_v] = float(v)
        point[self.normal_axis_name(surface_cfg)] = self.surface_height(surface_name, surface_cfg)
        return point

    def surface_height(self, surface_name, surface_cfg):
        surface = self.scene_config[surface_name]
        normal_axis = self.normal_axis_name(surface_cfg)
        center = float(surface['position'][AXIS_INDEX[normal_axis]])
        half = float(surface['size'][AXIS_INDEX[normal_axis]]) / 2.0
        sign = float(surface_cfg['surface_normal_sign'])
        return center - sign * half - sign * float(surface_cfg['contact_offset'])

    def normal_axis_name(self, surface_cfg):
        return surface_cfg['surface_normal_axis_world']

    def make_surface_pose(self, surface_cfg, point):
        return self.make_surface_pose_with_yaw(surface_cfg, point, 0.0)

    def make_surface_pose_with_yaw(self, surface_cfg, point, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.scene_config['world_frame']
        pose.pose.position.x = float(point['x'])
        pose.pose.position.y = float(point['y'])
        pose.pose.position.z = float(point['z'])
        qx, qy, qz, qw = self.surface_aligned_quaternion(surface_cfg, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def solve_waypoint(self, surface_cfg, point):
        query_yaw = float(surface_cfg.get('query_yaw', 0.0))
        search_yaw = bool(surface_cfg.get('search_yaw', False))
        configured_candidates_deg = surface_cfg.get('yaw_candidates_deg', [])
        if configured_candidates_deg:
            yaw_candidates = [
                query_yaw + math.radians(float(delta_deg))
                for delta_deg in configured_candidates_deg
            ]
        elif search_yaw:
            yaw_candidates = [
                query_yaw,
                query_yaw + math.pi / 2.0,
                query_yaw - math.pi / 2.0,
                query_yaw + math.pi,
            ]
        else:
            yaw_candidates = [query_yaw]

        last_response = None
        last_pose = None
        for yaw in yaw_candidates:
            pose = self.make_surface_pose_with_yaw(surface_cfg, point, yaw)
            response = self.call_ik(pose)
            last_pose = pose
            last_response = response
            if response is not None and response.success:
                return pose, response
        return last_pose, last_response

    def surface_aligned_quaternion(self, surface_cfg, yaw):
        normal_world = self.axis_vector(
            surface_cfg['surface_normal_axis_world'],
            float(surface_cfg['surface_normal_sign']),
        )
        tangent_world = self.axis_vector(
            surface_cfg['reference_axis_world'],
            float(surface_cfg['reference_axis_sign']),
        )
        normal_world = normalize(normal_world)
        tangent_world = tangent_world - np.dot(tangent_world, normal_world) * normal_world
        tangent_world = normalize(tangent_world)
        bitangent_world = normalize(np.cross(normal_world, tangent_world))

        yaw_rotation = rotation_matrix_from_axis_angle(normal_world, yaw)
        tangent_world = normalize(yaw_rotation @ tangent_world)
        bitangent_world = normalize(np.cross(normal_world, tangent_world))

        world_axes = [None, None, None]
        world_axes[AXIS_INDEX[surface_cfg['tool_normal_axis']]] = normal_world
        world_axes[AXIS_INDEX[surface_cfg['tool_tangent_axis']]] = tangent_world
        remaining_index = next(index for index in range(3) if world_axes[index] is None)

        trial_axes = list(world_axes)
        trial_axes[remaining_index] = bitangent_world
        rotation = np.column_stack(trial_axes)
        sign = 1.0 if np.linalg.det(rotation) > 0.0 else -1.0
        world_axes[remaining_index] = sign * bitangent_world
        rotation = np.column_stack(world_axes)
        return quaternion_from_rotation_matrix(rotation)

    def axis_vector(self, axis_name, sign):
        vector = np.zeros(3, dtype=float)
        vector[AXIS_INDEX[axis_name]] = float(sign)
        return vector

    def call_ik(self, pose):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            return None
        request = SolveSurfaceIK.Request()
        request.target_pose = pose
        future = self.ik_client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=self.service_timeout_sec):
            return None
        return future.result()

    def compute_metrics(self, surface_name, strategy, patch_size, x_coords, y_coords, grid, waypoints, joint_trajectory):
        total_area = patch_size[0] * patch_size[1]
        covered_area = len(waypoints) * self.waypoint_step * self.pad_width
        coverage_percent = 100.0 * min(covered_area / max(total_area, 1e-9), 1.0)

        path_length = 0.0
        for idx in range(1, len(waypoints)):
            prev = np.array([waypoints[idx - 1]['x'], waypoints[idx - 1]['y'], waypoints[idx - 1]['z']])
            curr = np.array([waypoints[idx]['x'], waypoints[idx]['y'], waypoints[idx]['z']])
            path_length += float(np.linalg.norm(curr - prev))

        estimated_time_s = 0.0
        for idx in range(1, len(joint_trajectory)):
            prev = np.array(joint_trajectory[idx - 1]['joint_state'].position, dtype=float)
            curr = np.array(joint_trajectory[idx]['joint_state'].position, dtype=float)
            deltas = np.abs(curr - prev)
            deltas = np.minimum(deltas, 2.0 * math.pi - deltas)
            max_delta = float(np.max(deltas)) if len(deltas) else 0.0
            estimated_time_s += max_delta / max(self.joint_speed_rad_s, 1e-6)

        return {
            'surface': surface_name,
            'strategy': strategy,
            'waypoints': len(waypoints),
            'coverage_percent': coverage_percent,
            'path_length_m': path_length,
            'estimated_time_s': estimated_time_s,
        }

    def export_surface_outputs(self, surface_name, timestamp, plan):
        suffix = f'{surface_name}_{plan["strategy"]}_{timestamp}'
        waypoints_csv = os.path.join(self.output_dir, f'coverage_waypoints_{suffix}.csv')
        with open(waypoints_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['x', 'y', 'z'])
            for point in plan['waypoints']:
                writer.writerow([point['x'], point['y'], point['z']])

        traj_csv = os.path.join(self.output_dir, f'coverage_joint_trajectory_{suffix}.csv')
        with open(traj_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['x', 'y', 'z', 'joint_names', 'joint_positions'])
            for row in plan['joint_trajectory']:
                js = row['joint_state']
                point = row['point']
                writer.writerow([
                    point['x'], point['y'], point['z'],
                    ';'.join(js.name),
                    ';'.join(f'{value:.10f}' for value in js.position),
                ])

        metrics_csv = os.path.join(self.output_dir, f'coverage_metrics_{suffix}.csv')
        with open(metrics_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['metric', 'value'])
            for key, value in plan['metrics'].items():
                writer.writerow([key, value])

        plot_path = os.path.join(self.output_dir, f'coverage_plan_{suffix}.png')
        self.save_plot(surface_name, plan, plot_path)

    def export_comparison(self, timestamp, comparison_rows):
        path = os.path.join(self.output_dir, f'coverage_strategy_comparison_{timestamp}.csv')
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['surface', 'strategy', 'waypoints', 'coverage_percent', 'path_length_m', 'estimated_time_s'])
            for row in comparison_rows:
                writer.writerow([
                    row['surface'],
                    row['strategy'],
                    row['waypoints'],
                    row['coverage_percent'],
                    row['path_length_m'],
                    row['estimated_time_s'],
                ])

    def save_plot(self, surface_name, plan, plot_path):
        fig, ax = plt.subplots(figsize=(8, 6))
        points = plan['waypoints']
        if surface_name == 'mirror':
            xs = [point['y'] for point in points]
            ys = [point['z'] for point in points]
            ax.set_xlabel('y (m)')
            ax.set_ylabel('z (m)')
            ax.set_title(f'{surface_name.capitalize()} {plan["strategy"].capitalize()} Plan')
        else:
            xs = [point['y'] for point in points]
            ys = [point['x'] for point in points]
            ax.set_xlabel('y (m)')
            ax.set_ylabel('x (m)')
            ax.set_title(f'{surface_name.capitalize()} {plan["strategy"].capitalize()} Plan')

        if points:
            ax.plot(xs, ys, color='tab:blue', linewidth=1.5, marker='o', markersize=2)
        fig.tight_layout()
        fig.savefig(plot_path, dpi=200)
        plt.close(fig)


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
