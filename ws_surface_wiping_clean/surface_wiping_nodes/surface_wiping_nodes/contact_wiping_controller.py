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
    """Simulation-grade impedance-like wiping controller for Section 3."""

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
        state = {
            'surface_distance_m': float(surface_cfg['initial_surface_distance_m']),
            'normal_velocity_m_s': 0.0,
            'commanded_normal_offset_m': 0.0,
        }
        contact_mode = 'approach'
        backoff_events = 0

        disturbance_cfg = surface_cfg.get('disturbance', {})
        disturbance_enabled = bool(disturbance_cfg.get('enabled', False))
        disturbance_index = -1
        disturbance_remaining = 0
        disturbance_consumed = False
        disturbance_force = 0.0
        if disturbance_enabled and active_waypoints:
            disturbance_index = min(
                len(active_waypoints) - 1,
                max(0, int(float(disturbance_cfg.get('waypoint_ratio', 0.5)) * len(active_waypoints)))
            )
            disturbance_force = float(disturbance_cfg.get('overload_force_n', 0.0))

        target_force = float(surface_cfg['target_force_n'])
        target_speed = float(surface_cfg['target_speed_m_s'])
        contact_force_threshold = float(surface_cfg['contact_force_threshold_n'])
        backoff_force = float(surface_cfg['backoff_force_n'])
        approach_speed = float(surface_cfg['approach_speed_m_s'])
        nominal_surface_distance = float(surface_cfg['nominal_surface_distance_m'])
        target_penetration = float(surface_cfg['target_penetration_m'])
        max_penetration = float(surface_cfg['max_penetration_m'])
        max_normal_offset = float(surface_cfg['max_normal_offset_m'])
        surface_stiffness = float(surface_cfg['surface_stiffness_n_m'])
        surface_damping = float(surface_cfg['surface_damping_n_s_m'])
        tool_impedance_stiffness = float(surface_cfg['tool_impedance_stiffness_n_m'])
        tool_impedance_damping = float(surface_cfg['tool_impedance_damping_n_s_m'])
        force_to_offset_gain = float(surface_cfg['force_to_offset_gain_m_n_s'])
        backoff_step = float(surface_cfg['backoff_step_m'])

        if active_waypoints:
            while True:
                measurement = self.compute_contact_measurement(
                    state,
                    nominal_surface_distance,
                    surface_stiffness,
                    surface_damping,
                    0.0,
                )
                if measurement['force_n'] >= contact_force_threshold:
                    contact_mode = 'contact_track'
                    logs.append(self.make_log_row(
                        surface_name,
                        time_s,
                        0,
                        active_waypoints[0],
                        measurement,
                        0.0,
                        contact_mode,
                        'contact_detected',
                    ))
                    time_s += self.sample_period_s
                    break

                state['surface_distance_m'] = max(
                    0.0,
                    state['surface_distance_m'] - approach_speed * self.sample_period_s,
                )
                state['normal_velocity_m_s'] = -approach_speed
                measurement = self.compute_contact_measurement(
                    state,
                    nominal_surface_distance,
                    surface_stiffness,
                    surface_damping,
                    0.0,
                )
                logs.append(self.make_log_row(
                    surface_name,
                    time_s,
                    0,
                    active_waypoints[0],
                    measurement,
                    0.0,
                    contact_mode,
                    'approach',
                ))
                time_s += self.sample_period_s

        for index, waypoint in enumerate(active_waypoints):
            previous = active_waypoints[index - 1] if index > 0 else waypoint
            segment_length = self.distance(previous, waypoint)
            base_duration = max(self.sample_period_s, segment_length / max(target_speed, 1e-6))
            steps = max(1, int(math.ceil(base_duration / self.sample_period_s)))

            for step in range(steps):
                speed_scale = 0.96 + 0.04 * math.sin(0.35 * (index + step))
                tangential_speed = target_speed * speed_scale
                event = 'track'

                if (
                    disturbance_enabled and
                    index == disturbance_index and
                    disturbance_remaining == 0 and
                    not disturbance_consumed
                ):
                    disturbance_remaining = int(disturbance_cfg.get('duration_samples', 6))
                    disturbance_consumed = True

                disturbance_term = disturbance_force if disturbance_remaining > 0 else 0.0
                if disturbance_remaining > 0:
                    disturbance_remaining -= 1
                    event = 'disturbance_injected'

                measurement = self.compute_contact_measurement(
                    state,
                    nominal_surface_distance,
                    surface_stiffness,
                    surface_damping,
                    disturbance_term,
                )

                force_error = target_force - measurement['force_n']
                desired_penetration = np.clip(
                    target_penetration + force_error / max(tool_impedance_stiffness, 1e-6),
                    0.0,
                    max_penetration,
                )
                desired_distance = max(0.0, nominal_surface_distance - desired_penetration)
                desired_distance -= force_error * force_to_offset_gain * self.sample_period_s
                desired_distance = np.clip(
                    desired_distance,
                    max(0.0, nominal_surface_distance - max_penetration),
                    nominal_surface_distance + max_normal_offset,
                )
                state['commanded_normal_offset_m'] = nominal_surface_distance - desired_distance

                accel = (
                    tool_impedance_stiffness * (desired_distance - state['surface_distance_m']) -
                    tool_impedance_damping * state['normal_velocity_m_s']
                )
                state['normal_velocity_m_s'] += accel * self.sample_period_s
                state['surface_distance_m'] += state['normal_velocity_m_s'] * self.sample_period_s
                state['surface_distance_m'] = np.clip(
                    state['surface_distance_m'],
                    0.0,
                    nominal_surface_distance + max_normal_offset,
                )

                measurement = self.compute_contact_measurement(
                    state,
                    nominal_surface_distance,
                    surface_stiffness,
                    surface_damping,
                    disturbance_term,
                )

                if measurement['force_n'] > backoff_force:
                    backoff_events += 1
                    contact_mode = 'backoff'
                    state['surface_distance_m'] = min(
                        nominal_surface_distance + max_normal_offset,
                        state['surface_distance_m'] + backoff_step,
                    )
                    state['normal_velocity_m_s'] = 0.0
                    state['commanded_normal_offset_m'] = nominal_surface_distance - state['surface_distance_m']
                    measurement = self.compute_contact_measurement(
                        state,
                        nominal_surface_distance,
                        surface_stiffness,
                        surface_damping,
                        0.0,
                    )
                    logs.append(self.make_log_row(
                        surface_name,
                        time_s,
                        index,
                        waypoint,
                        measurement,
                        0.0,
                        contact_mode,
                        'overforce_backoff_start',
                    ))
                    time_s += self.sample_period_s
                    contact_mode = 'contact_track'
                    continue

                contact_mode = 'contact_track'
                logs.append(self.make_log_row(
                    surface_name,
                    time_s,
                    index,
                    waypoint,
                    measurement,
                    tangential_speed,
                    contact_mode,
                    event,
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

    def compute_contact_measurement(
        self,
        state,
        nominal_surface_distance,
        surface_stiffness,
        surface_damping,
        disturbance_force,
    ):
        penetration = max(0.0, nominal_surface_distance - state['surface_distance_m'])
        penetration_rate = max(0.0, -state['normal_velocity_m_s']) if penetration > 0.0 else 0.0
        force_n = penetration * surface_stiffness + penetration_rate * surface_damping + disturbance_force
        force_n = max(0.0, force_n)
        return {
            'surface_distance_m': state['surface_distance_m'],
            'normal_velocity_m_s': state['normal_velocity_m_s'],
            'penetration_m': penetration,
            'commanded_normal_offset_m': state['commanded_normal_offset_m'],
            'force_n': force_n,
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

    def make_log_row(self, surface_name, time_s, waypoint_index, waypoint, measurement, speed_m_s, mode, event):
        return {
            'surface': surface_name,
            'time_s': time_s,
            'waypoint_index': waypoint_index,
            'x': waypoint['x'],
            'y': waypoint['y'],
            'z': waypoint['z'],
            'force_n': measurement['force_n'],
            'speed_m_s': speed_m_s,
            'mode': mode,
            'event': event,
            'surface_distance_m': measurement['surface_distance_m'],
            'penetration_m': measurement['penetration_m'],
            'normal_velocity_m_s': measurement['normal_velocity_m_s'],
            'commanded_normal_offset_m': measurement['commanded_normal_offset_m'],
        }

    def compute_metrics(self, surface_name, surface_cfg, logs, skipped_waypoints, backoff_events):
        target_force = float(surface_cfg['target_force_n'])
        force_tolerance = float(surface_cfg['force_tolerance_n'])
        target_speed = float(surface_cfg['target_speed_m_s'])

        force_track_logs = [row for row in logs if row['mode'] == 'contact_track']
        if force_track_logs:
            forces = np.array([row['force_n'] for row in force_track_logs], dtype=float)
            speeds = np.array([row['speed_m_s'] for row in force_track_logs], dtype=float)
            penetrations = np.array([row['penetration_m'] for row in force_track_logs], dtype=float)
            distances = np.array([row['surface_distance_m'] for row in force_track_logs], dtype=float)
            within_tol = np.abs(forces - target_force) <= force_tolerance
            force_within_tolerance_percent = 100.0 * float(np.mean(within_tol))
            avg_force = float(np.mean(forces))
            peak_force = float(np.max(forces))
            avg_speed = float(np.mean(speeds))
            peak_speed = float(np.max(speeds))
            avg_penetration = float(np.mean(penetrations))
            max_penetration = float(np.max(penetrations))
            avg_surface_distance = float(np.mean(distances))
        else:
            force_within_tolerance_percent = 0.0
            avg_force = 0.0
            peak_force = 0.0
            avg_speed = 0.0
            peak_speed = 0.0
            avg_penetration = 0.0
            max_penetration = 0.0
            avg_surface_distance = 0.0

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
            'avg_penetration_m': avg_penetration,
            'max_penetration_m': max_penetration,
            'avg_surface_distance_m': avg_surface_distance,
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
                    'surface_distance_m', 'penetration_m',
                    'normal_velocity_m_s', 'commanded_normal_offset_m',
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
                'avg_penetration_m',
                'max_penetration_m',
                'avg_surface_distance_m',
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
                    row['avg_penetration_m'],
                    row['max_penetration_m'],
                    row['avg_surface_distance_m'],
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
        penetrations = [row['penetration_m'] for row in logs]
        commanded_offsets = [row['commanded_normal_offset_m'] for row in logs]

        target_force = metrics['target_force_n']
        force_tol = metrics['force_tolerance_n']
        target_speed = metrics['target_speed_m_s']

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        ax_force, ax_penetration, ax_speed = axes

        ax_force.plot(times, forces, color='tab:red', linewidth=1.5, label='measured force')
        ax_force.axhline(target_force, color='black', linestyle='--', linewidth=1.0, label='target force')
        ax_force.axhspan(
            target_force - force_tol,
            target_force + force_tol,
            color='tab:green',
            alpha=0.18,
            label='force tolerance band',
        )
        ax_force.set_ylabel('Force (N)')
        ax_force.set_title(f'{surface_name.capitalize()} Contact Force')
        ax_force.legend(loc='upper right')

        ax_penetration.plot(times, penetrations, color='tab:purple', linewidth=1.5, label='penetration')
        ax_penetration.plot(times, commanded_offsets, color='tab:orange', linewidth=1.2, label='commanded offset')
        ax_penetration.set_ylabel('Normal (m)')
        ax_penetration.set_title(f'{surface_name.capitalize()} Normal Interaction')
        ax_penetration.legend(loc='upper right')

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
        ax_speed.set_title(f'{surface_name.capitalize()} Tangential Velocity')
        ax_speed.legend(loc='upper right')

        for axis in axes:
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
