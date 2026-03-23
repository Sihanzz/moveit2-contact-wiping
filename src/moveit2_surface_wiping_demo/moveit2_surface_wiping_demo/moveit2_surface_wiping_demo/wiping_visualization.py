import csv
import glob
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class WipingVisualization(Node):
    def __init__(self):
        super().__init__('wiping_visualization')

        package_share = get_package_share_directory('moveit2_surface_wiping_demo')
        scene_path = os.path.join(package_share, 'config', 'scene.yaml')
        control_path = os.path.join(package_share, 'config', 'control.yaml')

        with open(scene_path, 'r') as handle:
            self.scene_config = yaml.safe_load(handle)
        with open(control_path, 'r') as handle:
            self.control_config = yaml.safe_load(handle)

        self.declare_parameter('output_dir', '/home/sihan/ws_wiping/outputs')
        self.declare_parameter('coverage_waypoints_csv', '')
        self.declare_parameter('wiping_log_csv', '')
        self.declare_parameter('playback_rate', 1.0)
        self.declare_parameter('timer_period_sec', 0.05)
        self.declare_parameter('loop', True)

        self.output_dir = self.get_parameter('output_dir').value
        self.coverage_waypoints_csv = self.get_parameter('coverage_waypoints_csv').value
        self.wiping_log_csv = self.get_parameter('wiping_log_csv').value
        self.playback_rate = float(self.get_parameter('playback_rate').value)
        self.timer_period_sec = float(self.get_parameter('timer_period_sec').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.frame_id = self.scene_config['world_frame']
        self.surface_name = self.control_config['surface']
        self.surface_cfg = self.scene_config[self.surface_name]
        self.surface_z = self.surface_cfg['position'][2] + self.surface_cfg['size'][2] / 2.0

        self.waypoints = self.load_waypoints()
        self.logs = self.load_logs()
        if not self.waypoints:
            raise RuntimeError('No coverage waypoints available for visualization')
        if not self.logs:
            raise RuntimeError('No wiping log available for visualization')

        self.dynamic_pub = self.create_publisher(MarkerArray, 'wiping_demo_markers', 10)
        self.static_pub = self.create_publisher(MarkerArray, 'wiping_demo_static_markers', 10)

        self.path_points = [self.make_point(wp['x'], wp['y'], wp['z']) for wp in self.waypoints]
        self.skipped_points = [
            self.make_point(row['x'], row['y'], self.surface_z + 0.01)
            for row in self.logs
            if row['skipped_obstacle']
        ]

        self.static_markers = self.build_static_markers()
        self.static_publish_count = 0

        self.current_sim_time = 0.0
        self.log_index = 0

        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints and {len(self.logs)} log samples for RViz replay'
        )
        self.create_timer(self.timer_period_sec, self.on_timer)

    def resolve_csv(self, explicit_path, pattern):
        if explicit_path:
            return explicit_path if os.path.exists(explicit_path) else None
        matches = sorted(glob.glob(os.path.join(self.output_dir, pattern)))
        return matches[-1] if matches else None

    def load_waypoints(self):
        csv_path = self.resolve_csv(self.coverage_waypoints_csv, 'coverage_waypoints_*.csv')
        if not csv_path:
            return []
        self.get_logger().info(f'Loading waypoints from: {csv_path}')
        rows = []
        with open(csv_path, newline='') as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows.append({
                    'index': int(row['index']),
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'z': float(row['z']),
                })
        return rows

    def load_logs(self):
        csv_path = self.resolve_csv(self.wiping_log_csv, 'wiping_log_*.csv')
        if not csv_path:
            return []
        self.get_logger().info(f'Loading wiping log from: {csv_path}')
        rows = []
        with open(csv_path, newline='') as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows.append({
                    'time_s': float(row['time_s']),
                    'state': row['state'],
                    'waypoint_index': int(row['waypoint_index']),
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'z': float(row['z']),
                    'speed_xy': float(row['speed_xy']),
                    'force_z': float(row['force_z']),
                    'target_force_z': float(row['target_force_z']),
                    'skipped_obstacle': int(row['skipped_obstacle']) == 1,
                })
        return rows

    def make_point(self, x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def make_color(self, r, g, b, a):
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    def build_box_marker(self, marker_id, entry, color, z_offset=0.0):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.ns = 'scene'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(entry['position'][0])
        marker.pose.position.y = float(entry['position'][1])
        marker.pose.position.z = float(entry['position'][2] + z_offset)
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(entry['size'][0])
        marker.scale.y = float(entry['size'][1])
        marker.scale.z = float(entry['size'][2])
        marker.color = color
        return marker

    def build_static_markers(self):
        markers = []
        markers.append(self.build_box_marker(0, self.scene_config['countertop'], self.make_color(0.82, 0.70, 0.52, 0.35)))
        markers.append(self.build_box_marker(1, self.scene_config['faucet'], self.make_color(0.30, 0.36, 0.42, 0.65)))
        markers.append(self.build_box_marker(2, self.scene_config['mirror'], self.make_color(0.70, 0.82, 0.90, 0.35)))

        path_marker = Marker()
        path_marker.header.frame_id = self.frame_id
        path_marker.ns = 'plan'
        path_marker.id = 10
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.008
        path_marker.color = self.make_color(0.92, 0.36, 0.24, 0.95)
        path_marker.points = self.path_points
        markers.append(path_marker)

        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = self.frame_id
        waypoint_marker.ns = 'plan'
        waypoint_marker.id = 11
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.pose.orientation.w = 1.0
        waypoint_marker.scale.x = 0.012
        waypoint_marker.scale.y = 0.012
        waypoint_marker.scale.z = 0.012
        waypoint_marker.color = self.make_color(0.88, 0.50, 0.20, 0.85)
        waypoint_marker.points = self.path_points
        markers.append(waypoint_marker)

        if self.skipped_points:
            skipped_marker = Marker()
            skipped_marker.header.frame_id = self.frame_id
            skipped_marker.ns = 'events'
            skipped_marker.id = 12
            skipped_marker.type = Marker.SPHERE_LIST
            skipped_marker.action = Marker.ADD
            skipped_marker.pose.orientation.w = 1.0
            skipped_marker.scale.x = 0.018
            skipped_marker.scale.y = 0.018
            skipped_marker.scale.z = 0.018
            skipped_marker.color = self.make_color(0.80, 0.15, 0.15, 0.95)
            skipped_marker.points = self.skipped_points
            markers.append(skipped_marker)

        return markers

    def build_dynamic_markers(self, current_row):
        markers = []

        trail_marker = Marker()
        trail_marker.header.frame_id = self.frame_id
        trail_marker.ns = 'replay'
        trail_marker.id = 20
        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD
        trail_marker.pose.orientation.w = 1.0
        trail_marker.scale.x = 0.010
        trail_marker.color = self.make_color(0.12, 0.65, 0.55, 0.95)
        trail_marker.points = [
            self.make_point(row['x'], row['y'], self.surface_z + 0.004)
            for row in self.logs[:self.log_index + 1]
        ]
        markers.append(trail_marker)

        tcp_marker = Marker()
        tcp_marker.header.frame_id = self.frame_id
        tcp_marker.ns = 'replay'
        tcp_marker.id = 21
        tcp_marker.type = Marker.SPHERE
        tcp_marker.action = Marker.ADD
        tcp_marker.pose.position.x = float(current_row['x'])
        tcp_marker.pose.position.y = float(current_row['y'])
        tcp_marker.pose.position.z = float(current_row['z'])
        tcp_marker.pose.orientation.w = 1.0
        tcp_marker.scale.x = 0.028
        tcp_marker.scale.y = 0.028
        tcp_marker.scale.z = 0.028
        tcp_marker.color = self.make_color(0.08, 0.20, 0.34, 0.98)
        markers.append(tcp_marker)

        contact_marker = Marker()
        contact_marker.header.frame_id = self.frame_id
        contact_marker.ns = 'replay'
        contact_marker.id = 22
        contact_marker.type = Marker.CYLINDER
        contact_marker.action = Marker.ADD
        contact_marker.pose.position.x = float(current_row['x'])
        contact_marker.pose.position.y = float(current_row['y'])
        contact_marker.pose.position.z = float(self.surface_z + 0.002)
        contact_marker.pose.orientation.w = 1.0
        in_contact = current_row['state'] == 'CONTACT'
        contact_marker.scale.x = 0.065 if in_contact else 0.040
        contact_marker.scale.y = 0.065 if in_contact else 0.040
        contact_marker.scale.z = 0.004
        contact_marker.color = self.make_color(0.82, 0.15, 0.15, 0.75 if in_contact else 0.15)
        markers.append(contact_marker)

        text_marker = Marker()
        text_marker.header.frame_id = self.frame_id
        text_marker.ns = 'replay'
        text_marker.id = 23
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = float(current_row['x'])
        text_marker.pose.position.y = float(current_row['y'])
        text_marker.pose.position.z = float(max(self.surface_z, current_row['z']) + 0.10)
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.04
        text_marker.color = self.make_color(0.08, 0.08, 0.08, 0.98)
        text_marker.text = (
            f"state={current_row['state']} "
            f"t={current_row['time_s']:.1f}s "
            f"Fz={current_row['force_z']:.1f}N "
            f"v={current_row['speed_xy']:.2f}m/s"
        )
        markers.append(text_marker)

        return markers

    def publish_static_markers(self):
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        for marker in self.static_markers:
            marker.header.stamp = now
            marker.lifetime = Duration(seconds=0.0).to_msg()
            marker_array.markers.append(marker)
        self.static_pub.publish(marker_array)

    def on_timer(self):
        if self.static_publish_count < 20:
            self.publish_static_markers()
            self.static_publish_count += 1

        if self.log_index >= len(self.logs):
            if not self.loop:
                return
            self.current_sim_time = 0.0
            self.log_index = 0

        current_row = self.logs[self.log_index]
        while self.log_index + 1 < len(self.logs):
            next_row = self.logs[self.log_index + 1]
            if next_row['time_s'] > self.current_sim_time:
                break
            self.log_index += 1
            current_row = self.logs[self.log_index]

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for marker in self.build_dynamic_markers(current_row):
            marker.header.stamp = now
            marker.lifetime = Duration(seconds=self.timer_period_sec * 2.5).to_msg()
            marker_array.markers.append(marker)
        self.dynamic_pub.publish(marker_array)

        self.current_sim_time += self.timer_period_sec * max(self.playback_rate, 1e-6)
        if self.current_sim_time > self.logs[-1]['time_s'] and self.loop:
            self.current_sim_time = 0.0
            self.log_index = 0


def main(args=None):
    rclpy.init(args=args)
    node = WipingVisualization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
