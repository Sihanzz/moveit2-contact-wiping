import os
import csv
from collections import Counter
import threading
import math
import numpy as np
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Point

from moveit2_surface_wiping_interfaces.srv import ReachabilityQuery


class ReachabilityMapNode(Node):
    """
    Generates a reachability heatmap for a surface patch.

    This node:
    1. Samples poses on a grid over the target surface
    2. Tests IK feasibility for each pose (with collision checking)
    3. Exports results to CSV
    4. Generates a visualization heatmap
    """

    def __init__(self):
        super().__init__('reachability_map')
        self.callback_group = ReentrantCallbackGroup()

        # Load configuration
        package_share = get_package_share_directory('moveit2_surface_wiping_demo')
        config_path = os.path.join(package_share, 'config', 'reachability.yaml')
        scene_config_path = os.path.join(package_share, 'config', 'scene.yaml')

        self.get_logger().info(f'Loading config from: {config_path}')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        with open(scene_config_path, 'r') as f:
            self.scene_config = yaml.safe_load(f)

        # Parameters
        self.declare_parameter('output_dir', '/home/sihan/ws_wiping/outputs')
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('query_yaw', 0.0)
        self.declare_parameter('enforce_yaw', True)

        self.output_dir = self.get_parameter('output_dir').value
        self.service_timeout_sec = self.get_parameter('service_timeout_sec').value
        self.query_yaw = self.get_parameter('query_yaw').value
        self.enforce_yaw = self.get_parameter('enforce_yaw').value

        # Parse configuration
        self.surface_name = self.config['surface']
        self.patch_center = np.array(self.config['patch_center'])
        self.patch_size = np.array(self.config['patch_size'])
        self.resolution = self.config['resolution']
        self.surface_z_offset = self.config['surface_z_offset']
        # Get surface Z position from scene config
        surface_config = self.scene_config[self.surface_name]
        surface_z = surface_config['position'][2] + surface_config['size'][2] / 2
        self.surface_z = surface_z + self.surface_z_offset

        self.get_logger().info(f'Surface: {self.surface_name}')
        self.get_logger().info(f'Patch center: {self.patch_center}')
        self.get_logger().info(f'Patch size: {self.patch_size}')
        self.get_logger().info(f'Resolution: {self.resolution} m')
        self.get_logger().info(f'Surface Z: {self.surface_z} m')

        self.reachability_client = self.create_client(
            ReachabilityQuery,
            'reachability_query',
            callback_group=self.callback_group
        )

        self.get_logger().info('Waiting for reachability_query service...')
        if not self.reachability_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Reachability service not available!')
            return
        else:
            self.get_logger().info('Reachability service available')

        # Generate reachability map
        self.create_timer(2.0, self.generate_map_once, callback_group=self.callback_group)
        self.map_generated = False
        self.map_generation_in_progress = False

    def generate_grid_poses(self):
        """
        Generate a grid of test poses over the surface patch.

        Returns:
            tuple: (poses_list, x_coords, y_coords, grid_shape)
        """
        # Calculate grid dimensions
        half_size = self.patch_size / 2.0
        x_min = self.patch_center[0] - half_size[0]
        x_max = self.patch_center[0] + half_size[0]
        y_min = self.patch_center[1] - half_size[1]
        y_max = self.patch_center[1] + half_size[1]

        # Create grid
        x_coords = np.arange(x_min, x_max + self.resolution / 2, self.resolution)
        y_coords = np.arange(y_min, y_max + self.resolution / 2, self.resolution)

        self.get_logger().info(f'Grid size: {len(x_coords)} x {len(y_coords)} = {len(x_coords) * len(y_coords)} poses')

        poses = []
        for x in x_coords:
            for y in y_coords:
                poses.append((float(x), float(y), float(self.surface_z)))

        return poses, x_coords, y_coords, (len(x_coords), len(y_coords))

    def test_ik_for_pose(self, point):
        """
        Test if a pose is reachable via IK.

        Args:
            pose_stamped: Target pose

        Returns:
            tuple: (reachable, failure_reason, joint_solution)
        """
        if not self.reachability_client.service_is_ready():
            return False, 'service_not_ready', None

        request = ReachabilityQuery.Request()
        request.target_point = Point(x=point[0], y=point[1], z=point[2])
        request.frame_id = self.scene_config['world_frame']
        request.yaw = self.query_yaw
        request.enforce_yaw = self.enforce_yaw

        try:
            future = self.reachability_client.call_async(request)
            done_event = threading.Event()
            future.add_done_callback(lambda _: done_event.set())
            if not done_event.wait(timeout=self.service_timeout_sec):
                return False, 'service_call_timeout', None

            if future.result() is None:
                return False, 'service_call_failed', None

            response = future.result()
            if response.reachable:
                return True, '', response.solution
            return False, response.failure_reason or 'ik_failed', None

        except Exception:
            return False, 'service_call_failed', None

    def generate_map_once(self):
        """Generate the reachability map (called once via timer)."""
        if self.map_generated:
            return
        if self.map_generation_in_progress:
            return

        self.map_generation_in_progress = True

        self.get_logger().info('Starting reachability map generation...')

        # Generate grid poses
        poses, x_coords, y_coords, grid_shape = self.generate_grid_poses()

        # Test each pose
        results = []
        reachable_count = 0
        failure_counts = Counter()

        for idx, pose in enumerate(poses):
            is_reachable, failure_reason, solution = self.test_ik_for_pose(pose)
            results.append({
                'x': pose[0],
                'y': pose[1],
                'z': pose[2],
                'reachable': is_reachable,
                'failure_reason': failure_reason,
                'solution': solution,
            })
            if is_reachable:
                reachable_count += 1
            else:
                failure_counts[failure_reason] += 1

            if (idx + 1) % 25 == 0:
                progress = (idx + 1) / len(poses) * 100
                self.get_logger().info(
                    f'Progress: {idx + 1}/{len(poses)} ({progress:.1f}%) - '
                    f'Reachable: {reachable_count}/{idx + 1} - '
                    f'Failures: {dict(failure_counts)}'
                )

        # Reshape results to 2D grid
        reachability_grid = np.array([int(item['reachable']) for item in results], dtype=int).reshape(grid_shape)

        self.get_logger().info(f'Reachability map complete!')
        self.get_logger().info(f'Total poses: {len(poses)}')
        self.get_logger().info(f'Reachable: {reachable_count} ({reachable_count / len(poses) * 100:.1f}%)')
        self.get_logger().info(f'Unreachable: {len(poses) - reachable_count}')
        self.get_logger().info(f'Failure breakdown: {dict(failure_counts)}')

        # Export to CSV
        self.export_csv(results)

        # Generate visualization
        self.visualize_heatmap(x_coords, y_coords, reachability_grid)

        self.map_generated = True
        self.map_generation_in_progress = False

    def export_csv(self, results):
        """Export reachability data to CSV."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(self.output_dir, f'reachability_map_{timestamp}.csv')

        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)

            # Header
            writer.writerow([
                'x', 'y', 'z', 'reachable', 'failure_reason',
                'joint_names', 'joint_positions',
            ])

            for result in results:
                solution = result['solution']
                if solution is not None:
                    joint_names = ';'.join(solution.name)
                    joint_positions = ';'.join(str(value) for value in solution.position)
                else:
                    joint_names = ''
                    joint_positions = ''
                writer.writerow([
                    result['x'],
                    result['y'],
                    result['z'],
                    int(result['reachable']),
                    result['failure_reason'],
                    joint_names,
                    joint_positions,
                ])

        self.get_logger().info(f'CSV exported to: {csv_path}')

    def visualize_heatmap(self, x_coords, y_coords, reachability_grid):
        """Generate a heatmap visualization."""
        try:
            import matplotlib.pyplot as plt
            import matplotlib
            matplotlib.use('Agg')  # Non-interactive backend

            fig, ax = plt.subplots(figsize=(10, 8))

            # Create heatmap
            im = ax.imshow(
                reachability_grid.T,
                extent=[x_coords[0], x_coords[-1], y_coords[0], y_coords[-1]],
                origin='lower',
                cmap='RdYlGn',
                aspect='auto',
                interpolation='nearest'
            )

            # Colorbar
            cbar = plt.colorbar(im, ax=ax)
            cbar.set_label('Reachable (1=Yes, 0=No)', rotation=270, labelpad=20)

            # Labels and title
            ax.set_xlabel('X position (m)')
            ax.set_ylabel('Y position (m)')
            ax.set_title(f'Reachability Heatmap - {self.surface_name.capitalize()}\n'
                         f'Resolution: {self.resolution}m, Z={self.surface_z:.3f}m')

            # Grid
            ax.grid(True, alpha=0.3)

            # Save figure
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            plot_path = os.path.join(self.output_dir, f'reachability_heatmap_{timestamp}.png')
            plt.tight_layout()
            plt.savefig(plot_path, dpi=150)
            plt.close()

            self.get_logger().info(f'Heatmap saved to: {plot_path}')

        except ImportError:
            self.get_logger().warn('matplotlib not available, skipping visualization')
        except Exception as e:
            self.get_logger().error(f'Visualization error: {e}')


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
