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


class ContactWipingController(Node):
    """Simulation-grade contact-aware wiping controller for Section 3."""

    def __init__(self):
        super().__init__('contact_wiping_controller')

        package_share = get_package_share_directory('surface_wiping_nodes')
        scene_path = os.path.join(package_share, 'config', 'scene.yaml')
        control_path = os.path.join(package_share, 'config', 'wiping_control.yaml')

        with open(scene_path, 'r', encoding='utf-8') as handle:
            self.scene_config = yaml.safe_load(handle)
        with open(control_path, 'r', encoding='utf-8') as handle:
            self.control_config = yaml.safe_load(handle)

        self.declare_parameter('output_dir', '/home/sihan/ws_surface_wiping_clean/outputs')
        self.declare_parameter('countertop_waypoints_csv', '')
        self.declare_parameter('mirror_waypoints_csv', '')

        self.output_dir = self.get_parameter('output_dir').value
        self.surface_csv_override = {
            'countertop': self.get_parameter('countertop_waypoints_csv').value,
            'mirror': self.get_parameter('mirror_waypoints_csv').value,
        }
        self.sample_period_s = float(self.control_config['sample_period_s'])

        os.makedirs(self.output_dir, exist_ok=True)

        self.completed = False
        self.run_timer = self.create_timer(1.0, self.run_once)

    def run_once(self):
        if self.completed:
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        summary_rows = []

        for surface_name, surface_cfg in self.control_config['surfaces'].items():
            csv_path = self.resolve_waypoint_csv(surface_name, surface_cfg['waypoint_pattern'])
            if not csv_path:
                self.get_logger().warn(f'No waypoint CSV found for {surface_name}, skipping')
                continue

            self.get_logger().info(f'Executing simulated wiping control for {surface_name}: {csv_path}')
            waypoints = self.load_waypoints(csv_path)
            result = self.simulate_surface(surface_name, surface_cfg, waypoints)
            self.export_surface_outputs(surface_name, timestamp, result)
            summary_rows.append(result['metrics'])
            self.get_logger().info(
                f'Section 3 surface complete: {surface_name} '
                f'samples={result["metrics"]["samples"]} '
                f'force_within_tol={result["metrics"]["force_within_tolerance_percent"]:.1f}% '
                f'avg_speed={result["metrics"]["avg_speed_m_s"]:.3f} m/s '
                f'skipped={result["metrics"]["skipped_waypoints"]} '
                f'backoffs={result["metrics"]["backoff_events"]}'
            )

        self.export_summary(timestamp, summary_rows)
        self.completed = True
        self.run_timer.cancel()
        self.get_logger().info('Section 3 simulated control complete, shutting down node')
        self.destroy_node()

    def resolve_waypoint_csv(self, surface_name, pattern):
        override = self.surface_csv_override.get(surface_name, '')
        if override:
            return override if os.path.exists(override) else ''
        matches = sorted(glob.glob(os.path.join(self.output_dir, pattern)))
        return matches[-1] if matches else ''

    def load_waypoints(self, csv_path):
        with open(csv_path, newline='', encoding='utf-8') as handle:
            rows = list(csv.DictReader(handle))
        return [
            {
                'x': float(row['x']),
                'y': float(row['y']),
                'z': float(row['z']),
            }
            for row in rows
        ]

    def simulate_surface(self, surface_name, surface_cfg, waypoints):
        active_waypoints, skipped_waypoints = self.apply_obstacle_policy(surface_name, surface_cfg, waypoints)
        logs = []

        time_s = 0.0
        measured_force = 0.0
        contact_mode = 'position'
        backoff_events = 0

        disturbance_cfg = surface_cfg.get('disturbance', {})
        disturbance_enabled = bool(disturbance_cfg.get('enabled', False))
        disturbance_index = -1
        if disturbance_enabled and active_waypoints:
            disturbance_index = min(
                len(active_waypoints) - 1,
                max(0, int(float(disturbance_cfg.get('waypoint_ratio', 0.5)) * len(active_waypoints)))
            )
        disturbance_force = float(disturbance_cfg.get('overload_force_n', 0.0))

        target_force = float(surface_cfg['target_force_n'])
        force_tolerance = float(surface_cfg['force_tolerance_n'])
        switch_force = float(surface_cfg['switch_force_n'])
        backoff_force = float(surface_cfg['backoff_force_n'])
        target_speed = float(surface_cfg['target_speed_m_s'])
        approach_rate = float(surface_cfg['approach_force_rate_n_s'])
        response_gain = float(surface_cfg['force_response_gain'])
        backoff_drop = float(surface_cfg['backoff_force_drop_n_s'])

        if active_waypoints:
            while abs(measured_force) <= switch_force:
                measured_force += approach_rate * self.sample_period_s
                logs.append(self.make_log_row(
                    surface_name,
                    time_s,
                    0,
                    active_waypoints[0],
                    measured_force,
                    0.0,
                    contact_mode,
                    'approach',
                ))
                time_s += self.sample_period_s
            contact_mode = 'force'
            logs.append(self.make_log_row(
                surface_name,
                time_s,
                0,
                active_waypoints[0],
                measured_force,
                0.0,
                contact_mode,
                'switch_to_force_control',
            ))
            time_s += self.sample_period_s

        for index, waypoint in enumerate(active_waypoints):
            previous = active_waypoints[index - 1] if index > 0 else waypoint
            segment_length = self.distance(previous, waypoint)
            base_duration = max(self.sample_period_s, segment_length / max(target_speed, 1e-6))
            steps = max(1, int(math.ceil(base_duration / self.sample_period_s)))

            overload_injected = False
            for step in range(steps):
                speed_scale = 0.96 + 0.04 * math.sin(0.35 * (index + step))
                tangential_speed = target_speed * speed_scale
                event = ''

                measured_force += (target_force - measured_force) * response_gain * self.sample_period_s

                if disturbance_enabled and index == disturbance_index and step == steps // 2 and not overload_injected:
                    measured_force += disturbance_force
                    overload_injected = True
                    event = 'disturbance_injected'

                if abs(measured_force) > backoff_force:
                    backoff_events += 1
                    logs.append(self.make_log_row(
                        surface_name,
                        time_s,
                        index,
                        waypoint,
                        measured_force,
                        0.0,
                        'backoff',
                        'overforce_backoff_start',
                    ))
                    time_s += self.sample_period_s
                    while measured_force > target_force:
                        measured_force = max(target_force * 0.8, measured_force - backoff_drop * self.sample_period_s)
                        logs.append(self.make_log_row(
                            surface_name,
                            time_s,
                            index,
                            waypoint,
                            measured_force,
                            0.0,
                            'backoff',
                            'backoff',
                        ))
                        time_s += self.sample_period_s
                    contact_mode = 'force'
                    break

                logs.append(self.make_log_row(
                    surface_name,
                    time_s,
                    index,
                    waypoint,
                    measured_force,
                    tangential_speed,
                    contact_mode,
                    event or 'track',
                ))
                time_s += self.sample_period_s

        metrics = self.compute_metrics(
            surface_name,
            surface_cfg,
            logs,
            skipped_waypoints,
            backoff_events,
        )

        return {
            'surface': surface_name,
            'logs': logs,
            'metrics': metrics,
        }

    def apply_obstacle_policy(self, surface_name, surface_cfg, waypoints):
        obstacle_cfg = surface_cfg.get('obstacle')
        if not obstacle_cfg or obstacle_cfg.get('mode', '') != 'skip':
            return list(waypoints), 0

        object_name = obstacle_cfg['object_name']
        if object_name not in self.scene_config:
            return list(waypoints), 0

        obstacle = self.scene_config[object_name]
        margin = float(obstacle_cfg.get('margin_m', 0.0))
        x_min = float(obstacle['position'][0]) - float(obstacle['size'][0]) / 2.0 - margin
        x_max = float(obstacle['position'][0]) + float(obstacle['size'][0]) / 2.0 + margin
        y_min = float(obstacle['position'][1]) - float(obstacle['size'][1]) / 2.0 - margin
        y_max = float(obstacle['position'][1]) + float(obstacle['size'][1]) / 2.0 + margin

        active = []
        skipped = 0
        for waypoint in waypoints:
            if surface_name == 'countertop' and x_min <= waypoint['x'] <= x_max and y_min <= waypoint['y'] <= y_max:
                skipped += 1
                continue
            active.append(waypoint)
        return active, skipped

    def make_log_row(self, surface_name, time_s, waypoint_index, waypoint, force_n, speed_m_s, mode, event):
        return {
            'surface': surface_name,
            'time_s': time_s,
            'waypoint_index': waypoint_index,
            'x': waypoint['x'],
            'y': waypoint['y'],
            'z': waypoint['z'],
            'force_n': force_n,
            'speed_m_s': speed_m_s,
            'mode': mode,
            'event': event,
        }

    def compute_metrics(self, surface_name, surface_cfg, logs, skipped_waypoints, backoff_events):
        target_force = float(surface_cfg['target_force_n'])
        force_tolerance = float(surface_cfg['force_tolerance_n'])
        target_speed = float(surface_cfg['target_speed_m_s'])

        force_track_logs = [row for row in logs if row['mode'] == 'force']
        if force_track_logs:
            forces = np.array([row['force_n'] for row in force_track_logs], dtype=float)
            speeds = np.array([row['speed_m_s'] for row in force_track_logs], dtype=float)
            within_tol = np.abs(forces - target_force) <= force_tolerance
            force_within_tolerance_percent = 100.0 * float(np.mean(within_tol))
            avg_force = float(np.mean(forces))
            peak_force = float(np.max(forces))
            avg_speed = float(np.mean(speeds))
            peak_speed = float(np.max(speeds))
        else:
            force_within_tolerance_percent = 0.0
            avg_force = 0.0
            peak_force = 0.0
            avg_speed = 0.0
            peak_speed = 0.0

        return {
            'surface': surface_name,
            'samples': len(logs),
            'target_force_n': target_force,
            'force_tolerance_n': force_tolerance,
            'target_speed_m_s': target_speed,
            'force_within_tolerance_percent': force_within_tolerance_percent,
            'avg_force_n': avg_force,
            'peak_force_n': peak_force,
            'avg_speed_m_s': avg_speed,
            'peak_speed_m_s': peak_speed,
            'skipped_waypoints': skipped_waypoints,
            'backoff_events': backoff_events,
        }

    def export_surface_outputs(self, surface_name, timestamp, result):
        suffix = f'{surface_name}_{timestamp}'

        log_csv = os.path.join(self.output_dir, f'contact_wiping_log_{suffix}.csv')
        with open(log_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    'surface', 'time_s', 'waypoint_index', 'x', 'y', 'z',
                    'force_n', 'speed_m_s', 'mode', 'event',
                ],
            )
            writer.writeheader()
            writer.writerows(result['logs'])

        metrics_csv = os.path.join(self.output_dir, f'contact_wiping_metrics_{suffix}.csv')
        with open(metrics_csv, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['metric', 'value'])
            for key, value in result['metrics'].items():
                writer.writerow([key, value])

        plot_path = os.path.join(self.output_dir, f'contact_wiping_tracking_{suffix}.png')
        self.save_tracking_plot(surface_name, result, plot_path)

    def export_summary(self, timestamp, rows):
        path = os.path.join(self.output_dir, f'contact_wiping_summary_{timestamp}.csv')
        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow([
                'surface',
                'samples',
                'target_force_n',
                'force_tolerance_n',
                'target_speed_m_s',
                'force_within_tolerance_percent',
                'avg_force_n',
                'peak_force_n',
                'avg_speed_m_s',
                'peak_speed_m_s',
                'skipped_waypoints',
                'backoff_events',
            ])
            for row in rows:
                writer.writerow([
                    row['surface'],
                    row['samples'],
                    row['target_force_n'],
                    row['force_tolerance_n'],
                    row['target_speed_m_s'],
                    row['force_within_tolerance_percent'],
                    row['avg_force_n'],
                    row['peak_force_n'],
                    row['avg_speed_m_s'],
                    row['peak_speed_m_s'],
                    row['skipped_waypoints'],
                    row['backoff_events'],
                ])

    def save_tracking_plot(self, surface_name, result, plot_path):
        logs = result['logs']
        metrics = result['metrics']
        if not logs:
            return

        times = [row['time_s'] for row in logs]
        forces = [row['force_n'] for row in logs]
        speeds = [row['speed_m_s'] for row in logs]

        target_force = metrics['target_force_n']
        force_tol = metrics['force_tolerance_n']
        target_speed = metrics['target_speed_m_s']

        fig, (ax_force, ax_speed) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

        ax_force.plot(times, forces, color='tab:red', linewidth=1.5, label='measured force')
        ax_force.axhline(target_force, color='black', linestyle='--', linewidth=1.0, label='target force')
        ax_force.axhspan(
            target_force - force_tol,
            target_force + force_tol,
            color='tab:green',
            alpha=0.18,
            label='force tolerance band',
        )
        ax_force.axhline(15.0, color='tab:orange', linestyle=':', linewidth=1.0, label='backoff threshold')
        ax_force.set_ylabel('Force (N)')
        ax_force.set_title(f'{surface_name.capitalize()} Force Tracking')
        ax_force.legend(loc='upper right')

        ax_speed.plot(times, speeds, color='tab:blue', linewidth=1.5, label='tangential speed')
        ax_speed.axhline(target_speed, color='black', linestyle='--', linewidth=1.0, label='target speed')
        ax_speed.axhspan(
            target_speed * 0.9,
            target_speed * 1.1,
            color='tab:cyan',
            alpha=0.18,
            label='speed band',
        )
        ax_speed.set_ylabel('Speed (m/s)')
        ax_speed.set_xlabel('Time (s)')
        ax_speed.set_title(f'{surface_name.capitalize()} Velocity Tracking')
        ax_speed.legend(loc='upper right')

        for axis in (ax_force, ax_speed):
            axis.grid(True, linestyle=':', linewidth=0.5, alpha=0.6)

        fig.tight_layout()
        fig.savefig(plot_path, dpi=200)
        plt.close(fig)

    def distance(self, a, b):
        return float(np.linalg.norm(np.array([a['x'], a['y'], a['z']]) - np.array([b['x'], b['y'], b['z']])))


def main(args=None):
    rclpy.init(args=args)
    node = ContactWipingController()
    try:
        rclpy.spin(node)
    finally:
        if node.context.ok():
            node.destroy_node()
            rclpy.shutdown()
