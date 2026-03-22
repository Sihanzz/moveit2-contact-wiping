import math
import os
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from moveit2_surface_wiping_interfaces.srv import ReachabilityQuery

from .joint_state_helper import JointStateHelper


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ReachabilityIkService(Node):
    def __init__(self):
        super().__init__('ik_service')

        package_share = get_package_share_directory('moveit2_surface_wiping_demo')
        scene_config_path = os.path.join(package_share, 'config', 'scene.yaml')
        reachability_config_path = os.path.join(package_share, 'config', 'reachability.yaml')

        with open(scene_config_path, 'r') as handle:
            self.scene_config = yaml.safe_load(handle)
        with open(reachability_config_path, 'r') as handle:
            self.reachability_config = yaml.safe_load(handle)

        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('base_frame', self.scene_config['world_frame'])
        # TCP frame relative to base:
        #   tcp x -> base x
        #   tcp y -> -base z
        #   tcp z -> base y
        # This is a -90 deg rotation about base x.
        self.declare_parameter('surface_roll', -math.pi / 2.0)
        self.declare_parameter('surface_pitch', 0.0)
        self.declare_parameter('ik_timeout_sec', 0.5)
        self.declare_parameter('service_wait_timeout_sec', 2.0)

        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.surface_roll = self.get_parameter('surface_roll').value
        self.surface_pitch = self.get_parameter('surface_pitch').value
        self.ik_timeout_sec = self.get_parameter('ik_timeout_sec').value
        self.service_wait_timeout_sec = self.get_parameter('service_wait_timeout_sec').value

        surface = self.reachability_config['surface']
        surface_cfg = self.scene_config[surface]
        top_surface = surface_cfg['position'][2] + surface_cfg['size'][2] / 2.0
        self.surface_z = top_surface + self.reachability_config['surface_z_offset']

        patch_center = self.reachability_config['patch_center']
        patch_size = self.reachability_config['patch_size']
        self.x_min = patch_center[0] - patch_size[0] / 2.0
        self.x_max = patch_center[0] + patch_size[0] / 2.0
        self.y_min = patch_center[1] - patch_size[1] / 2.0
        self.y_max = patch_center[1] + patch_size[1] / 2.0

        self.joint_state_helper = JointStateHelper(self)

        self.service_group = ReentrantCallbackGroup()
        self.client_group = ReentrantCallbackGroup()
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.client_group,
        )
        self.service = self.create_service(
            ReachabilityQuery,
            'reachability_query',
            self.handle_query,
            callback_group=self.service_group,
        )

        self.get_logger().info(f'IK service ready for group={self.planning_group}, tip={self.ee_link}')

    def handle_query(self, request, response):
        if not self.ik_client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            response.failure_reason = 'ik_service_unavailable'
            return response

        if not self.joint_state_helper.has_joint_state():
            response.failure_reason = 'joint_state_unavailable'
            return response

        point = request.target_point
        if request.frame_id and request.frame_id != self.base_frame:
            response.failure_reason = 'unsupported_frame'
            return response
        if point.x < self.x_min or point.x > self.x_max or point.y < self.y_min or point.y > self.y_max:
            response.failure_reason = 'out_of_bounds'
            return response
        if abs(point.z - self.surface_z) > 1e-4:
            response.failure_reason = 'surface_height_mismatch'
            return response

        yaw_candidates = [request.yaw] if request.enforce_yaw else [
            request.yaw,
            request.yaw + math.pi / 2.0,
            request.yaw - math.pi / 2.0,
            request.yaw + math.pi,
        ]

        for yaw in yaw_candidates:
            pose = self.make_pose(point.x, point.y, point.z, yaw)
            result = self.call_ik(pose)
            if result is None:
                continue
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                response.reachable = True
                response.failure_reason = ''
                response.solution = result.solution.joint_state
                return response

        response.failure_reason = 'ik_failed'
        return response

    def make_pose(self, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        qx, qy, qz, qw = quaternion_from_euler(
            self.surface_roll,
            self.surface_pitch,
            yaw,
        )
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def call_ik(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_group
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(self.ik_timeout_sec)
        request.ik_request.timeout.nanosec = int((self.ik_timeout_sec % 1.0) * 1e9)
        request.ik_request.robot_state = self.joint_state_helper.get_robot_state()

        future = self.ik_client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=self.service_wait_timeout_sec):
            return None
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityIkService()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
