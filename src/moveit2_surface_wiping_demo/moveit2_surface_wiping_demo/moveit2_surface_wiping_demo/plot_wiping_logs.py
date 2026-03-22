import csv
import glob
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node


class PlotWipingLogs(Node):
    def __init__(self):
        super().__init__('plot_wiping_logs')
        self.declare_parameter('output_dir', '/home/sihan/ws_wiping/outputs')
        self.declare_parameter('log_csv', '')
        self.output_dir = self.get_parameter('output_dir').value
        self.log_csv = self.get_parameter('log_csv').value
        self.run()

    def resolve_log(self):
        if self.log_csv:
            return self.log_csv if os.path.exists(self.log_csv) else None
        matches = sorted(glob.glob(os.path.join(self.output_dir, 'wiping_log_*.csv')))
        return matches[-1] if matches else None

    def run(self):
        log_path = self.resolve_log()
        if not log_path:
            self.get_logger().error('No wiping log CSV found')
            return

        rows = []
        with open(log_path, newline='') as handle:
            reader = csv.DictReader(handle)
            rows = list(reader)
        if not rows:
            self.get_logger().error('Wiping log CSV is empty')
            return

        times = [float(row['time_s']) for row in rows]
        forces = [float(row['force_z']) for row in rows]
        target_forces = [float(row['target_force_z']) for row in rows]
        speeds = [float(row['speed_xy']) for row in rows]

        plot_path = os.path.join(self.output_dir, 'wiping_force_velocity_latest.png')
        fig, (ax_force, ax_speed) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        ax_force.plot(times, forces, label='Measured Fz')
        ax_force.plot(times, target_forces, '--', label='Target Fz')
        ax_force.set_ylabel('Force (N)')
        ax_force.legend()
        ax_force.grid(True, alpha=0.3)

        ax_speed.plot(times, speeds, label='Tangential speed')
        ax_speed.set_xlabel('Time (s)')
        ax_speed.set_ylabel('Speed (m/s)')
        ax_speed.legend()
        ax_speed.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        self.get_logger().info(f'Plot saved to: {plot_path}')


def main(args=None):
    rclpy.init(args=args)
    node = PlotWipingLogs()
    node.destroy_node()
    rclpy.shutdown()
