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
from moveit_msgs.msg import Constraints, MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from surface_wiping_interfaces.srv import SolveSurfaceIK
from .joint_state_helper import JointStateHelper


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


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def angular_distance(a, b):
    return abs(wrap_to_pi(a - b))


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
        self.declare_parameter('cartesian_service_name', '/compute_cartesian_path')
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('service_timeout_sec', 5.0)

        self.output_dir = self.get_parameter('output_dir').value
        self.reachability_csv = self.get_parameter('reachability_csv').value
        self.service_name = self.get_parameter('service_name').value
        self.cartesian_service_name = self.get_parameter('cartesian_service_name').value
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)

        self.pad_length = float(self.coverage_config['pad_length'])
        self.pad_width = float(self.coverage_config['pad_width'])
        self.keepout_margin = float(self.coverage_config['keepout_margin'])
        self.overlap = float(self.coverage_config['overlap'])
        self.waypoint_step = float(self.coverage_config['waypoint_step'])
        self.joint_speed_rad_s = float(self.coverage_config['joint_speed_rad_s'])
        self.cartesian_max_step = float(self.coverage_config.get('cartesian_max_step', 0.01))
        self.cartesian_jump_threshold = float(
            self.coverage_config.get('cartesian_jump_threshold', 0.0)
        )
        self.cartesian_prismatic_jump_threshold = float(
            self.coverage_config.get('cartesian_prismatic_jump_threshold', 0.0)
        )
        self.cartesian_revolute_jump_threshold = float(
            self.coverage_config.get('cartesian_revolute_jump_threshold', 0.0)
        )
        self.cartesian_max_velocity_scaling_factor = float(
            self.coverage_config.get('cartesian_max_velocity_scaling_factor', 0.2)
        )
        self.cartesian_max_acceleration_scaling_factor = float(
            self.coverage_config.get('cartesian_max_acceleration_scaling_factor', 0.2)
        )
        self.cartesian_max_speed_m_s = float(
            self.coverage_config.get('cartesian_max_speed_m_s', 0.0)
        )

        os.makedirs(self.output_dir, exist_ok=True)

        self.client_group = ReentrantCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()
        self.joint_state_helper = JointStateHelper(self)

        self.ik_client = self.create_client(
            SolveSurfaceIK,
            self.service_name,
            callback_group=self.client_group,
        )
        self.cartesian_client = self.create_client(
            GetCartesianPath,
            self.cartesian_service_name,
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
                solution_lookup[(round(x, 6), round(y, 6))] = {
                    'joint_state': joint_state,
                    'yaw': float(row['yaw']) if row.get('yaw') not in ('', None) else 0.0,
                    'orientation': {
                        'qx': float(row['qx']) if row.get('qx') not in ('', None) else 0.0,
                        'qy': float(row['qy']) if row.get('qy') not in ('', None) else 0.0,
                        'qz': float(row['qz']) if row.get('qz') not in ('', None) else 0.0,
                        'qw': float(row['qw']) if row.get('qw') not in ('', None) else 1.0,
                    },
                }

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
        cartesian_waypoints, cartesian_trajectory = self.build_cartesian_plan(
            surface_name,
            waypoints,
            joint_trajectory,
        )
        if cartesian_trajectory:
            waypoints = cartesian_waypoints
            joint_trajectory = cartesian_trajectory

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
        previous_solution = None
        for index, waypoint in enumerate(waypoints, start=1):
            solution = self.solve_waypoint(surface_cfg, waypoint, previous_solution)
            response = solution['response']
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
            enriched_waypoint = dict(waypoint)
            enriched_waypoint['yaw'] = solution['yaw']
            enriched_waypoint['orientation'] = solution['orientation']
            feasible_waypoints.append(enriched_waypoint)
            joint_trajectory.append({
                'point': enriched_waypoint,
                'joint_state': response.joint_state,
                'yaw': solution['yaw'],
                'orientation': solution['orientation'],
            })
            previous_solution = solution
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
        cartesian_waypoints, cartesian_trajectory = self.build_cartesian_plan(
            surface_name,
            feasible_waypoints,
            joint_trajectory,
        )
        if cartesian_trajectory:
            feasible_waypoints = cartesian_waypoints
            joint_trajectory = cartesian_trajectory
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
        previous_solution = None
        for y_idx in selected_y_indices:
            reachable_x = [x_idx for x_idx in range(len(x_coords)) if grid[x_idx, y_idx]]
            if not reachable_x:
                continue
            ordered = reachable_x if direction > 0 else list(reversed(reachable_x))
            for x_idx in ordered:
                x = float(x_coords[x_idx])
                y = float(y_coords[y_idx])
                key = (round(x, 6), round(y, 6))
                cached = solution_lookup.get(key)
                if cached is None:
                    point = {'x': x, 'y': y, 'z': self.surface_height(surface_name, surface_cfg)}
                    solution = self.solve_waypoint(surface_cfg, point, previous_solution)
                    response = solution['response']
                    if response is None or not response.success:
                        continue
                    joint_state = response.joint_state
                    orientation = solution['orientation']
                    yaw = solution['yaw']
                    previous_solution = solution
                else:
                    point = {'x': x, 'y': y, 'z': self.surface_height(surface_name, surface_cfg)}
                    solution = self.solve_waypoint(
                        surface_cfg,
                        point,
                        previous_solution,
                        query_yaw_override=cached['yaw'],
                    )
                    response = solution['response']
                    if response is not None and response.success:
                        joint_state = response.joint_state
                        orientation = solution['orientation']
                        yaw = solution['yaw']
                        previous_solution = solution
                    else:
                        joint_state = cached['joint_state']
                        yaw = cached['yaw']
                        orientation = cached['orientation']
                        previous_solution = {'yaw': yaw, 'orientation': orientation}
                point = {
                    'x': x,
                    'y': y,
                    'z': self.surface_height(surface_name, surface_cfg),
                    'yaw': yaw,
                    'orientation': orientation,
                }
                waypoints.append(point)
                joint_trajectory.append({
                    'point': point,
                    'joint_state': joint_state,
                    'yaw': yaw,
                    'orientation': orientation,
                })
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

    def solve_waypoint(self, surface_cfg, point, previous_solution=None, query_yaw_override=None):
        query_yaw = float(
            surface_cfg.get('query_yaw', 0.0)
            if query_yaw_override is None else query_yaw_override
        )
        yaw_candidates = self.build_yaw_candidates(surface_cfg, previous_solution, query_yaw)

        last_response = None
        last_pose = None
        last_yaw = None
        for yaw in yaw_candidates:
            pose = self.make_surface_pose_with_yaw(surface_cfg, point, yaw)
            response = self.call_ik(pose)
            last_pose = pose
            last_response = response
            last_yaw = yaw
            if response is not None and response.success:
                orientation = {
                    'qx': pose.pose.orientation.x,
                    'qy': pose.pose.orientation.y,
                    'qz': pose.pose.orientation.z,
                    'qw': pose.pose.orientation.w,
                }
                return {
                    'pose': pose,
                    'response': response,
                    'yaw': wrap_to_pi(yaw),
                    'orientation': orientation,
                }
        orientation = None
        if last_pose is not None:
            orientation = {
                'qx': last_pose.pose.orientation.x,
                'qy': last_pose.pose.orientation.y,
                'qz': last_pose.pose.orientation.z,
                'qw': last_pose.pose.orientation.w,
            }
        return {
            'pose': last_pose,
            'response': last_response,
            'yaw': last_yaw,
            'orientation': orientation,
        }

    def build_yaw_candidates(self, surface_cfg, previous_solution, query_yaw):
        if not bool(surface_cfg.get('search_yaw', False)):
            return [wrap_to_pi(query_yaw)]

        local_candidates = []
        if previous_solution is not None and previous_solution.get('yaw') is not None:
            previous_yaw = float(previous_solution['yaw'])
            continuation_window = math.radians(float(surface_cfg.get('continuation_window_deg', 20.0)))
            continuation_step = math.radians(float(surface_cfg.get('continuation_step_deg', 5.0)))
            local_candidates = self.angle_sweep(
                previous_yaw - continuation_window,
                previous_yaw + continuation_window + continuation_step * 0.5,
                continuation_step,
                previous_yaw,
            )

        fallback_candidates = self.angle_sweep(
            math.radians(float(surface_cfg.get('yaw_min_deg', -180.0))),
            math.radians(float(surface_cfg.get('yaw_max_deg', 180.0))),
            math.radians(float(surface_cfg.get('yaw_step_deg', 15.0))),
            query_yaw,
            base=query_yaw,
        )
        refined_candidates = self.angle_sweep(
            -math.radians(float(surface_cfg.get('yaw_step_deg', 15.0))),
            math.radians(float(surface_cfg.get('yaw_step_deg', 15.0))) +
            math.radians(float(surface_cfg.get('yaw_refinement_deg', 5.0))) * 0.5,
            math.radians(float(surface_cfg.get('yaw_refinement_deg', 5.0))),
            query_yaw,
            base=query_yaw,
        )
        return self.deduplicate_angles(local_candidates + refined_candidates + fallback_candidates)

    def angle_sweep(self, angle_min, angle_max, angle_step, center, base=0.0):
        if angle_step <= 0.0:
            return [wrap_to_pi(center)]
        values = []
        angle = angle_min
        while angle < angle_max - 1e-9:
            values.append(wrap_to_pi(base + angle))
            angle += angle_step
        values.sort(key=lambda yaw: (angular_distance(yaw, center), yaw))
        return self.deduplicate_angles(values)

    def deduplicate_angles(self, angles):
        unique = []
        for yaw in angles:
            wrapped = wrap_to_pi(yaw)
            if all(angular_distance(wrapped, existing) > 1e-6 for existing in unique):
                unique.append(wrapped)
        return unique

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

    def build_cartesian_plan(self, surface_name, waypoints, fallback_trajectory):
        if len(waypoints) < 2:
            return waypoints, fallback_trajectory
        response = self.call_cartesian_path(waypoints, fallback_trajectory)
        if response is None:
            self.get_logger().warn(
                f'Cartesian path service unavailable for {surface_name}, using discrete IK trajectory'
            )
            return waypoints, fallback_trajectory
        if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction <= 0.0:
            self.get_logger().warn(
                f'Cartesian path failed for {surface_name}: '
                f'code={response.error_code.val} fraction={response.fraction:.3f}. '
                f'Using discrete IK trajectory'
            )
            return waypoints, fallback_trajectory

        accepted_count = len(waypoints)
        if response.fraction < 0.999:
            accepted_count = max(1, int(math.floor(response.fraction * len(waypoints))))
            self.get_logger().warn(
                f'Cartesian path for {surface_name} is partial: '
                f'fraction={response.fraction:.3f} accepted_waypoints={accepted_count}/{len(waypoints)}'
            )
        accepted_waypoints = waypoints[:accepted_count]
        cartesian_rows = self.robot_trajectory_to_rows(
            response.solution.joint_trajectory.joint_names,
            response.solution.joint_trajectory.points,
            accepted_waypoints,
        )
        if not cartesian_rows:
            self.get_logger().warn(
                f'Cartesian path returned no trajectory points for {surface_name}, using discrete IK trajectory'
            )
            return waypoints, fallback_trajectory
        return accepted_waypoints, cartesian_rows

    def call_cartesian_path(self, waypoints, fallback_trajectory):
        if not fallback_trajectory:
            return None
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            return None

        request = GetCartesianPath.Request()
        request.header.frame_id = self.scene_config['world_frame']
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state = self.make_robot_state_from_joint_state(
            fallback_trajectory[0]['joint_state']
        )
        request.group_name = self.planning_group
        request.link_name = self.end_effector_link
        request.waypoints = [self.pose_from_waypoint(waypoint) for waypoint in waypoints]
        request.max_step = self.cartesian_max_step
        request.jump_threshold = self.cartesian_jump_threshold
        request.prismatic_jump_threshold = self.cartesian_prismatic_jump_threshold
        request.revolute_jump_threshold = self.cartesian_revolute_jump_threshold
        request.avoid_collisions = True
        request.path_constraints = Constraints()
        request.max_velocity_scaling_factor = self.cartesian_max_velocity_scaling_factor
        request.max_acceleration_scaling_factor = self.cartesian_max_acceleration_scaling_factor
        request.cartesian_speed_limited_link = self.end_effector_link if self.cartesian_max_speed_m_s > 0.0 else ''
        request.max_cartesian_speed = self.cartesian_max_speed_m_s

        future = self.cartesian_client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=self.service_timeout_sec):
            return None
        return future.result()

    def make_robot_state_from_joint_state(self, joint_state):
        robot_state = self.joint_state_helper.get_robot_state()
        robot_state.joint_state = joint_state
        robot_state.is_diff = False
        return robot_state

    def pose_from_waypoint(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = self.scene_config['world_frame']
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        pose.pose.position.z = float(waypoint['z'])
        orientation = waypoint.get('orientation', {})
        pose.pose.orientation.x = float(orientation.get('qx', 0.0))
        pose.pose.orientation.y = float(orientation.get('qy', 0.0))
        pose.pose.orientation.z = float(orientation.get('qz', 0.0))
        pose.pose.orientation.w = float(orientation.get('qw', 1.0))
        return pose.pose

    def robot_trajectory_to_rows(self, joint_names, trajectory_points, waypoints):
        rows = []
        if not trajectory_points:
            return rows
        total_points = len(trajectory_points)
        total_waypoints = len(waypoints)
        for index, point in enumerate(trajectory_points):
            if total_waypoints == 0:
                waypoint = None
            elif total_points == 1:
                waypoint = waypoints[0]
            else:
                waypoint_index = min(
                    total_waypoints - 1,
                    int(round(index * (total_waypoints - 1) / max(total_points - 1, 1))),
                )
                waypoint = waypoints[waypoint_index]
            joint_state = JointState()
            joint_state.name = list(joint_names)
            joint_state.position = list(point.positions)
            rows.append({
                'point': waypoint,
                'joint_state': joint_state,
                'yaw': waypoint.get('yaw', '') if waypoint else '',
                'orientation': waypoint.get('orientation', {}) if waypoint else {},
            })
        return rows

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
        max_joint_delta = 0.0
        for idx in range(1, len(joint_trajectory)):
            prev = np.array(joint_trajectory[idx - 1]['joint_state'].position, dtype=float)
            curr = np.array(joint_trajectory[idx]['joint_state'].position, dtype=float)
            deltas = np.abs(curr - prev)
            deltas = np.minimum(deltas, 2.0 * math.pi - deltas)
            max_delta = float(np.max(deltas)) if len(deltas) else 0.0
            estimated_time_s += max_delta / max(self.joint_speed_rad_s, 1e-6)
            max_joint_delta = max(max_joint_delta, max_delta)

        yaw_changes = []
        for idx in range(1, len(waypoints)):
            prev_yaw = float(waypoints[idx - 1].get('yaw', 0.0))
            curr_yaw = float(waypoints[idx].get('yaw', 0.0))
            yaw_changes.append(angular_distance(curr_yaw, prev_yaw))
        avg_yaw_change = float(np.mean(yaw_changes)) if yaw_changes else 0.0
        max_yaw_jump = float(np.max(yaw_changes)) if yaw_changes else 0.0

        return {
            'surface': surface_name,
            'strategy': strategy,
            'waypoints': len(waypoints),
            'coverage_percent': coverage_percent,
            'path_length_m': path_length,
            'estimated_time_s': estimated_time_s,
            'avg_yaw_change_deg': math.degrees(avg_yaw_change),
            'max_yaw_jump_deg': math.degrees(max_yaw_jump),
            'max_joint_delta_rad': max_joint_delta,
        }

    def export_surface_outputs(self, surface_name, timestamp, plan):
        suffix = f'{surface_name}_{plan["strategy"]}_{timestamp}'
        waypoints_csv = os.path.join(self.output_dir, f'coverage_waypoints_{suffix}.csv')
        with open(waypoints_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['x', 'y', 'z', 'yaw', 'qx', 'qy', 'qz', 'qw'])
            for point in plan['waypoints']:
                orientation = point.get('orientation', {})
                writer.writerow([
                    point['x'],
                    point['y'],
                    point['z'],
                    point.get('yaw', ''),
                    orientation.get('qx', ''),
                    orientation.get('qy', ''),
                    orientation.get('qz', ''),
                    orientation.get('qw', ''),
                ])

        traj_csv = os.path.join(self.output_dir, f'coverage_joint_trajectory_{suffix}.csv')
        with open(traj_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'x', 'y', 'z', 'yaw', 'qx', 'qy', 'qz', 'qw',
                'joint_names', 'joint_positions',
            ])
            for row in plan['joint_trajectory']:
                js = row['joint_state']
                point = row['point']
                orientation = row.get('orientation', point.get('orientation', {}))
                writer.writerow([
                    point['x'],
                    point['y'],
                    point['z'],
                    row.get('yaw', point.get('yaw', '')),
                    orientation.get('qx', ''),
                    orientation.get('qy', ''),
                    orientation.get('qz', ''),
                    orientation.get('qw', ''),
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
