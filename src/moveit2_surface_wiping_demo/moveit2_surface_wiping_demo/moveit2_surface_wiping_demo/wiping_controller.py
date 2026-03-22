import csv
import glob
import math
import os
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class WipingController(Node):
    def __init__(self):
        super().__init__('wiping_controller')

        package_share = get_package_share_directory('moveit2_surface_wiping_demo')
        scene_path = os.path.join(package_share, 'config', 'scene.yaml')
        control_path = os.path.join(package_share, 'config', 'control.yaml')

        with open(scene_path, 'r') as handle:
            self.scene_config = yaml.safe_load(handle)
        with open(control_path, 'r') as handle:
            self.control_config = yaml.safe_load(handle)

        self.declare_parameter('output_dir', '/home/sihan/ws_wiping/outputs')
        self.declare_parameter('coverage_waypoints_csv', self.control_config.get('coverage_waypoints_csv', ''))

        self.output_dir = self.get_parameter('output_dir').value
        self.coverage_waypoints_csv = self.get_parameter('coverage_waypoints_csv').value
        os.makedirs(self.output_dir, exist_ok=True)

        self.surface_name = self.control_config['surface']
        self.surface_cfg = self.scene_config[self.surface_name]
        self.dt = float(self.control_config['dt'])
        self.position_tolerance = float(self.control_config['position_tolerance'])
        self.lookahead_distance = float(self.control_config.get('lookahead_distance', 0.02))
        self.approach_height = float(self.control_config['approach_height'])
        self.approach_speed = float(self.control_config['approach_speed'])
        self.xy_kp = float(self.control_config['xy_kp'])
        self.contact_threshold = float(self.control_config['contact_threshold_n'])
        self.force_limit = float(self.control_config['force_limit_n'])
        self.contact_stiffness = float(self.control_config['contact_stiffness_n_per_m'])
        self.force_kp_z = float(self.control_config['force_kp_z'])
        self.vz_limit = float(self.control_config['vz_limit'])
        self.backoff_distance = float(self.control_config['backoff_distance'])
        self.backoff_speed = float(self.control_config['backoff_speed'])
        self.obstacle_margin = float(self.control_config['obstacle_margin'])
        self.skip_mode = self.control_config['skip_replan_mode']
        self.plot_title = self.control_config['plot_title']

        if self.surface_name == 'mirror':
            self.target_force = float(self.control_config['mirror_target_force_n'])
            self.target_speed = float(self.control_config['mirror_speed_mps'])
        else:
            self.target_force = float(self.control_config['countertop_target_force_n'])
            self.target_speed = float(self.control_config['countertop_speed_mps'])

        self.surface_z = self.surface_cfg['position'][2] + self.surface_cfg['size'][2] / 2.0
        self.faucet_xy_bounds = self.compute_xy_keepout(self.scene_config.get('faucet'))

        self.waypoints = self.load_waypoints()
        if not self.waypoints:
            self.get_logger().error('No coverage waypoints available')
            self.ready = False
            return

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints for {self.surface_name}')
        self.get_logger().info(
            f'Target force={self.target_force:.1f} N, target speed={self.target_speed:.2f} m/s'
        )

        self.ready = True
        self.sim_ran = False
        self.create_timer(1.0, self.run_once)

    def compute_xy_keepout(self, obstacle_cfg):
        if not obstacle_cfg:
            return None
        size = obstacle_cfg['size']
        pos = obstacle_cfg['position']
        return (
            pos[0] - size[0] / 2.0 - self.obstacle_margin,
            pos[0] + size[0] / 2.0 + self.obstacle_margin,
            pos[1] - size[1] / 2.0 - self.obstacle_margin,
            pos[1] + size[1] / 2.0 + self.obstacle_margin,
        )

    def resolve_waypoints_csv(self):
        if self.coverage_waypoints_csv:
            return self.coverage_waypoints_csv if os.path.exists(self.coverage_waypoints_csv) else None
        pattern = os.path.join(self.output_dir, 'coverage_waypoints_*.csv')
        matches = sorted(glob.glob(pattern))
        return matches[-1] if matches else None

    def load_waypoints(self):
        csv_path = self.resolve_waypoints_csv()
        if not csv_path:
            self.get_logger().error('No coverage_waypoints CSV found in outputs/')
            return []
        self.get_logger().info(f'Loading waypoints from: {csv_path}')
        waypoints = []
        with open(csv_path, newline='') as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                waypoints.append(np.array([
                    float(row['x']),
                    float(row['y']),
                    float(row['z']),
                ], dtype=float))
        return waypoints

    def run_once(self):
        if not self.ready or self.sim_ran:
            return
        self.sim_ran = True

        state = 'APPROACH'
        current = self.waypoints[0].copy()
        current[2] += self.approach_height
        backoff_remaining = 0.0

        logs = []
        waypoint_index = 0
        skipped_waypoints = 0
        backoff_events = 0
        contact_switches = 0
        t = 0.0

        while waypoint_index < len(self.waypoints):
            target = self.waypoints[waypoint_index]

            if self.is_in_obstacle_keepout(target):
                logs.append(self.make_log_row(
                    t, 'SKIP', waypoint_index, current, np.zeros(3), 0.0, 0.0, 0.0, True
                ))
                waypoint_index += 1
                skipped_waypoints += 1
                t += self.dt
                continue

            surface_force = self.compute_contact_force(current[2])

            if state == 'APPROACH':
                velocity = np.zeros(3)
                xy_error = target[:2] - current[:2]
                xy_distance = np.linalg.norm(xy_error)
                if xy_distance > 1e-9:
                    xy_direction = xy_error / xy_distance
                    xy_speed = min(self.target_speed, self.xy_kp * xy_distance)
                    velocity[:2] = xy_direction * xy_speed
                velocity[2] = -self.approach_speed
                if abs(surface_force) > self.contact_threshold:
                    state = 'CONTACT'
                    contact_switches += 1
            elif state == 'CONTACT':
                velocity = np.zeros(3)
                waypoint_index = self.advance_waypoint_index(current, waypoint_index)
                if waypoint_index >= len(self.waypoints):
                    break
                target = self.waypoints[waypoint_index]
                xy_error = target[:2] - current[:2]
                xy_distance = np.linalg.norm(xy_error)
                if xy_distance > 1e-9:
                    xy_direction = xy_error / xy_distance
                    velocity[:2] = xy_direction * self.target_speed
                force_error = self.target_force - surface_force
                velocity[2] = np.clip(-self.force_kp_z * force_error, -self.vz_limit, self.vz_limit)

                if abs(surface_force) > self.force_limit:
                    state = 'BACKOFF'
                    backoff_remaining = self.backoff_distance
                    backoff_events += 1
            else:  # BACKOFF
                velocity = np.array([0.0, 0.0, self.backoff_speed], dtype=float)
                backoff_remaining -= abs(velocity[2]) * self.dt
                if backoff_remaining <= 0.0:
                    state = 'APPROACH'
                    waypoint_index += 1

            current = current + velocity * self.dt
            speed_xy = float(np.linalg.norm(velocity[:2]))
            logs.append(self.make_log_row(
                t,
                state,
                waypoint_index,
                current,
                velocity,
                speed_xy,
                surface_force,
                self.target_force,
                False,
            ))
            t += self.dt

            if t > 600.0:
                self.get_logger().warn('Simulation stopped after 600 s cap')
                break

        metrics = {
            'surface': self.surface_name,
            'target_force_n': self.target_force,
            'target_speed_mps': self.target_speed,
            'timesteps': len(logs),
            'duration_s': logs[-1]['time_s'] if logs else 0.0,
            'waypoints_total': len(self.waypoints),
            'waypoints_skipped': skipped_waypoints,
            'contact_switches': contact_switches,
            'backoff_events': backoff_events,
        }

        self.export_logs(logs, metrics)
        self.get_logger().info(
            f'Wiping control complete: duration={metrics["duration_s"]:.1f}s '
            f'skipped={skipped_waypoints} backoffs={backoff_events}'
        )

    def advance_waypoint_index(self, current, waypoint_index):
        threshold = max(self.position_tolerance, self.lookahead_distance, self.target_speed * self.dt * 1.25)
        while waypoint_index < len(self.waypoints):
            target = self.waypoints[waypoint_index]
            if self.is_in_obstacle_keepout(target):
                return waypoint_index
            xy_distance = np.linalg.norm(target[:2] - current[:2])
            if xy_distance > threshold:
                return waypoint_index
            waypoint_index += 1
        return waypoint_index

    def compute_contact_force(self, current_z):
        penetration = max(0.0, self.surface_z - current_z)
        return self.contact_stiffness * penetration

    def is_in_obstacle_keepout(self, target):
        if not self.faucet_xy_bounds:
            return False
        x_min, x_max, y_min, y_max = self.faucet_xy_bounds
        return x_min <= target[0] <= x_max and y_min <= target[1] <= y_max

    def make_log_row(self, t, state, waypoint_index, current, velocity, speed_xy, force_z, target_force, skipped):
        return {
            'time_s': t,
            'state': state,
            'waypoint_index': waypoint_index,
            'x': current[0],
            'y': current[1],
            'z': current[2],
            'vx': velocity[0],
            'vy': velocity[1],
            'vz': velocity[2],
            'speed_xy': speed_xy,
            'force_z': force_z,
            'target_force_z': target_force,
            'force_error': target_force - force_z,
            'skipped_obstacle': int(skipped),
        }

    def export_logs(self, logs, metrics):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(self.output_dir, f'wiping_log_{timestamp}.csv')
        metrics_path = os.path.join(self.output_dir, f'wiping_metrics_{timestamp}.csv')
        plot_path = os.path.join(self.output_dir, f'wiping_force_velocity_{timestamp}.png')

        with open(log_path, 'w', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(logs[0].keys()) if logs else [
                'time_s', 'state', 'waypoint_index', 'x', 'y', 'z',
                'vx', 'vy', 'vz', 'speed_xy', 'force_z',
                'target_force_z', 'force_error', 'skipped_obstacle',
            ])
            writer.writeheader()
            for row in logs:
                writer.writerow(row)

        with open(metrics_path, 'w', newline='') as handle:
            writer = csv.writer(handle)
            writer.writerow(['metric', 'value'])
            for key, value in metrics.items():
                writer.writerow([key, value])

        self.save_plot(logs, plot_path)

        self.get_logger().info(f'Wiping log exported to: {log_path}')
        self.get_logger().info(f'Wiping metrics exported to: {metrics_path}')
        self.get_logger().info(f'Force/velocity plot saved to: {plot_path}')

    def save_plot(self, logs, plot_path):
        if not logs:
            return
        times = [row['time_s'] for row in logs]
        forces = [row['force_z'] for row in logs]
        target_forces = [row['target_force_z'] for row in logs]
        speeds = [row['speed_xy'] for row in logs]

        fig, (ax_force, ax_speed) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax_force.plot(times, forces, label='Measured Fz', color='#006d77')
        ax_force.plot(times, target_forces, label='Target Fz', color='#ee6c4d', linestyle='--')
        ax_force.axhline(self.contact_threshold, color='#999999', linestyle=':', label='Contact threshold')
        ax_force.axhline(self.force_limit, color='#b00020', linestyle=':', label='Force limit')
        ax_force.set_ylabel('Force (N)')
        ax_force.set_title(self.plot_title)
        ax_force.legend()
        ax_force.grid(True, alpha=0.3)

        ax_speed.plot(times, speeds, label='Tangential speed', color='#3d5a80')
        ax_speed.axhline(self.target_speed, color='#ee6c4d', linestyle='--', label='Target speed')
        ax_speed.set_xlabel('Time (s)')
        ax_speed.set_ylabel('Speed (m/s)')
        ax_speed.legend()
        ax_speed.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = WipingController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
