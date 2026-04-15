import csv
import math
import os
import threading
from collections import Counter
from datetime import datetime

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

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


class ReachabilityMapNode(Node):
    """Samples a surface patch and exports a collision-aware IK reachability map."""

    def __init__(self):
        super().__init__('reachability_map')
        self.callback_group = ReentrantCallbackGroup()

        package_share = get_package_share_directory('surface_wiping_nodes')
        scene_path = os.path.join(package_share, 'config', 'scene.yaml')
        reachability_path = os.path.join(package_share, 'config', 'reachability.yaml')

        with open(scene_path, 'r', encoding='utf-8') as handle:
            self.scene_config = yaml.safe_load(handle)
        with open(reachability_path, 'r', encoding='utf-8') as handle:
            self.reachability_config = yaml.safe_load(handle)

        config_query_yaw = float(self.reachability_config.get('query_yaw', 0.0))
        config_search_yaw = bool(self.reachability_config.get('search_yaw', True))

        self.declare_parameter('output_dir', '/home/sihan/ws_surface_wiping_clean/outputs')
        self.declare_parameter('service_name', 'solve_surface_ik')
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('query_yaw', config_query_yaw)
        self.declare_parameter('search_yaw', config_search_yaw)

        self.output_dir = self.get_parameter('output_dir').value
        self.service_name = self.get_parameter('service_name').value
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.query_yaw = float(self.get_parameter('query_yaw').value)
        self.search_yaw = bool(self.get_parameter('search_yaw').value)

        self.surface_name = self.reachability_config['surface']
        self.patch_center = np.array(self.reachability_config['patch_center'], dtype=float)
        self.patch_size = np.array(self.reachability_config['patch_size'], dtype=float)
        self.resolution = float(self.reachability_config['resolution'])
        self.surface_z_offset = float(self.reachability_config['surface_z_offset'])
        self.base_frame = self.scene_config['world_frame']
        self.tool_normal_axis = self.reachability_config['tool_normal_axis']
        self.tool_tangent_axis = self.reachability_config['tool_tangent_axis']
        self.surface_normal_world = self._axis_vector(
            self.reachability_config['surface_normal_axis_world'],
            float(self.reachability_config['surface_normal_sign']),
        )
        self.reference_axis_world = self._axis_vector(
            self.reachability_config['reference_axis_world'],
            float(self.reachability_config['reference_axis_sign']),
        )

        surface_config = self.scene_config[self.surface_name]
        top_surface = surface_config['position'][2] + surface_config['size'][2] / 2.0
        self.surface_z = top_surface + self.surface_z_offset

        os.makedirs(self.output_dir, exist_ok=True)

        self.client = self.create_client(
            SolveSurfaceIK,
            self.service_name,
            callback_group=self.callback_group,
        )

        self.generated = False
        self.in_progress = False

        self.get_logger().info(
            f'Reachability map configured: surface={self.surface_name} '
            f'patch_center={self.patch_center.tolist()} patch_size={self.patch_size.tolist()} '
            f'resolution={self.resolution:.3f} surface_z={self.surface_z:.3f} '
            f'tool_normal_axis={self.tool_normal_axis} '
            f'tool_tangent_axis={self.tool_tangent_axis}'
        )

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Service not available: {self.service_name}')
            return

        self.create_timer(2.0, self.generate_map_once, callback_group=self.callback_group)

    def generate_map_once(self):
        if self.generated or self.in_progress:
            return

        self.in_progress = True

        self.get_logger().info('Starting reachability map generation...')

        poses, x_coords, y_coords, grid_shape = self.generate_grid()
        results = []
        reachable_count = 0
        failure_counts = Counter()

        for index, point in enumerate(poses, start=1):
            result = self.evaluate_point(point)
            results.append(result)
            if result['reachable']:
                reachable_count += 1
            else:
                failure_counts[result['message']] += 1

            if index % 25 == 0 or index == len(poses):
                self.get_logger().info(
                    f'Progress {index}/{len(poses)} '
                    f'reachable={reachable_count} failures={dict(failure_counts)}'
                )

        reachability_grid = np.array(
            [1 if item['reachable'] else 0 for item in results],
            dtype=int,
        ).reshape(grid_shape)

        csv_path = self.export_csv(results)
        png_path = self.visualize_heatmap(x_coords, y_coords, reachability_grid)

        self.get_logger().info(
            f'Reachability complete: reachable={reachable_count}/{len(results)} '
            f'ratio={100.0 * reachable_count / len(results):.1f}% '
            f'csv={csv_path} heatmap={png_path}'
        )

        self.generated = True
        self.in_progress = False

    def generate_grid(self):
        half_size = self.patch_size / 2.0
        x_min = self.patch_center[0] - half_size[0]
        x_max = self.patch_center[0] + half_size[0]
        y_min = self.patch_center[1] - half_size[1]
        y_max = self.patch_center[1] + half_size[1]

        x_coords = np.arange(x_min, x_max + self.resolution / 2.0, self.resolution)
        y_coords = np.arange(y_min, y_max + self.resolution / 2.0, self.resolution)

        points = []
        for x in x_coords:
            for y in y_coords:
                points.append((float(x), float(y), float(self.surface_z)))

        self.get_logger().info(
            f'Grid size: {len(x_coords)} x {len(y_coords)} = {len(points)} samples'
        )
        return points, x_coords, y_coords, (len(x_coords), len(y_coords))

    def evaluate_point(self, point):
        yaw_candidates = [self.query_yaw]
        if self.search_yaw:
            yaw_candidates = [
                self.query_yaw,
                self.query_yaw + math.pi / 2.0,
                self.query_yaw - math.pi / 2.0,
                self.query_yaw + math.pi,
            ]

        messages = []
        for yaw in yaw_candidates:
            pose = self.make_surface_pose(point, yaw)
            response = self.call_solver(pose)
            if response is None:
                messages.append('service_call_timeout')
                continue
            if response.success:
                return {
                    'x': point[0],
                    'y': point[1],
                    'z': point[2],
                    'reachable': True,
                    'message': response.message,
                    'yaw': yaw,
                    'pose': pose,
                    'joint_state': response.joint_state,
                }
            messages.append(response.message or 'ik_failed')

        return {
            'x': point[0],
            'y': point[1],
            'z': point[2],
            'reachable': False,
            'message': self.summarize_failure(messages),
            'yaw': float('nan'),
            'pose': None,
            'joint_state': None,
        }

    def make_surface_pose(self, point, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(point[0])
        pose.pose.position.y = float(point[1])
        pose.pose.position.z = float(point[2])
        qx, qy, qz, qw = self._surface_aligned_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def call_solver(self, pose):
        request = SolveSurfaceIK.Request()
        request.target_pose = pose

        future = self.client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=self.service_timeout_sec):
            return None
        return future.result()

    def summarize_failure(self, messages):
        if not messages:
            return 'ik_failed'
        counts = Counter(messages)
        return counts.most_common(1)[0][0]

    def _axis_vector(self, axis_name, sign):
        vector = np.zeros(3, dtype=float)
        vector[AXIS_INDEX[axis_name]] = float(sign)
        return vector

    def _surface_aligned_quaternion(self, yaw):
        normal_world = normalize(self.surface_normal_world)
        tangent_world = self.reference_axis_world - np.dot(
            self.reference_axis_world,
            normal_world,
        ) * normal_world
        tangent_world = normalize(tangent_world)
        bitangent_world = normalize(np.cross(normal_world, tangent_world))

        yaw_rotation = rotation_matrix_from_axis_angle(normal_world, yaw)
        tangent_world = normalize(yaw_rotation @ tangent_world)
        bitangent_world = normalize(np.cross(normal_world, tangent_world))

        world_axes = [None, None, None]
        world_axes[AXIS_INDEX[self.tool_normal_axis]] = normal_world
        world_axes[AXIS_INDEX[self.tool_tangent_axis]] = tangent_world

        remaining_index = next(
            index for index in range(3)
            if world_axes[index] is None
        )
        tool_axis_sign = 1.0
        candidate = bitangent_world

        # Preserve right-handed tool coordinates.
        trial_axes = list(world_axes)
        trial_axes[remaining_index] = candidate
        rotation = np.column_stack(trial_axes)
        if np.linalg.det(rotation) < 0.0:
            tool_axis_sign = -1.0

        world_axes[remaining_index] = tool_axis_sign * bitangent_world
        rotation = np.column_stack(world_axes)
        return quaternion_from_rotation_matrix(rotation)

    def export_csv(self, results):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(self.output_dir, f'reachability_map_{timestamp}.csv')

        with open(csv_path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'x', 'y', 'z', 'reachable', 'message',
                'yaw',
                'qx', 'qy', 'qz', 'qw',
                'joint_names', 'joint_positions',
            ])

            for result in results:
                pose = result['pose']
                joint_state = result['joint_state']
                if pose is not None:
                    qx = pose.pose.orientation.x
                    qy = pose.pose.orientation.y
                    qz = pose.pose.orientation.z
                    qw = pose.pose.orientation.w
                else:
                    qx = qy = qz = qw = ''

                if joint_state is not None:
                    joint_names = ';'.join(joint_state.name)
                    joint_positions = ';'.join(
                        f'{value:.10f}' for value in joint_state.position
                    )
                else:
                    joint_names = ''
                    joint_positions = ''

                writer.writerow([
                    result['x'],
                    result['y'],
                    result['z'],
                    int(result['reachable']),
                    result['message'],
                    result['yaw'],
                    qx,
                    qy,
                    qz,
                    qw,
                    joint_names,
                    joint_positions,
                ])

        return csv_path

    def visualize_heatmap(self, x_coords, y_coords, reachability_grid):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().warn('matplotlib not available, skipping heatmap export')
            return ''

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        png_path = os.path.join(self.output_dir, f'reachability_heatmap_{timestamp}.png')

        fig, ax = plt.subplots(figsize=(8, 6))
        extent = [y_coords[0], y_coords[-1], x_coords[0], x_coords[-1]]
        image = ax.imshow(
            reachability_grid,
            origin='lower',
            extent=extent,
            aspect='auto',
            cmap='RdYlGn',
            vmin=0,
            vmax=1,
        )
        ax.set_title('Countertop Reachability Heatmap')
        ax.set_xlabel('y (m)')
        ax.set_ylabel('x (m)')
        cbar = fig.colorbar(image, ax=ax, ticks=[0, 1])
        cbar.ax.set_yticklabels(['blocked', 'reachable'])
        fig.tight_layout()
        fig.savefig(png_path, dpi=200)
        plt.close(fig)
        return png_path


def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityMapNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
