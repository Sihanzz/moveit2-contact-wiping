import threading

import rclpy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from surface_wiping_interfaces.srv import SolveSurfaceIK

from .joint_state_helper import JointStateHelper


class SurfaceIkServer(Node):
    """
    Thin wrapper around MoveIt /compute_ik for surface wiping tasks.

    This service intentionally stays generic:
    - client code chooses the target pose and surface-specific alignment
    - this node validates the request shape and asks MoveIt for a collision-aware IK solution
    """

    def __init__(self):
        super().__init__('surface_ik_server')

        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ik_timeout_sec', 0.5)
        self.declare_parameter('service_wait_timeout_sec', 2.0)
        self.declare_parameter('service_name', 'solve_surface_ik')
        self.declare_parameter('compute_ik_service_name', '/compute_ik')

        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ik_timeout_sec = float(self.get_parameter('ik_timeout_sec').value)
        self.service_wait_timeout_sec = float(
            self.get_parameter('service_wait_timeout_sec').value
        )
        self.service_name = self.get_parameter('service_name').value
        self.compute_ik_service_name = self.get_parameter(
            'compute_ik_service_name'
        ).value

        self.joint_state_helper = JointStateHelper(self)
        self.service_group = ReentrantCallbackGroup()
        self.client_group = ReentrantCallbackGroup()

        self.ik_client = self.create_client(
            GetPositionIK,
            self.compute_ik_service_name,
            callback_group=self.client_group,
        )
        self.service = self.create_service(
            SolveSurfaceIK,
            self.service_name,
            self.handle_request,
            callback_group=self.service_group,
        )

        self.get_logger().info(
            f'Surface IK server ready: service={self.service_name} '
            f'group={self.planning_group} tip={self.ee_link} '
            f'base_frame={self.base_frame}'
        )

    def handle_request(self, request, response):
        if not self.ik_client.wait_for_service(timeout_sec=self.service_wait_timeout_sec):
            response.success = False
            response.message = 'ik_service_unavailable'
            return response

        target_pose = request.target_pose
        frame_id = target_pose.header.frame_id or self.base_frame
        if frame_id != self.base_frame:
            response.success = False
            response.message = f'unsupported_frame:{frame_id}'
            return response

        if not self._has_valid_quaternion(target_pose):
            response.success = False
            response.message = 'invalid_orientation'
            return response

        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()

        result = self.call_ik(target_pose)
        if result is None:
            response.success = False
            response.message = 'ik_service_timeout'
            return response

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            response.success = True
            response.message = 'success'
            response.joint_state = result.solution.joint_state
            return response

        response.success = False
        response.message = self._error_code_to_string(result.error_code.val)
        return response

    def call_ik(self, pose):
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_group
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped = pose
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = int(self.ik_timeout_sec)
        request.ik_request.timeout.nanosec = int((self.ik_timeout_sec % 1.0) * 1e9)
        request.ik_request.robot_state = self.joint_state_helper.get_robot_state()
        if not self.joint_state_helper.has_joint_state():
            self.get_logger().warn(
                'No /joint_states received yet, calling MoveIt IK with default robot_state seed'
            )

        future = self.ik_client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=self.service_wait_timeout_sec):
            return None
        return future.result()

    def _has_valid_quaternion(self, pose_stamped):
        orientation = pose_stamped.pose.orientation
        norm_sq = (
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w
        )
        return norm_sq > 1e-8

    def _error_code_to_string(self, error_code):
        known_codes = {
            MoveItErrorCodes.SUCCESS: 'success',
            MoveItErrorCodes.FAILURE: 'failure',
            MoveItErrorCodes.PLANNING_FAILED: 'planning_failed',
            MoveItErrorCodes.INVALID_MOTION_PLAN: 'invalid_motion_plan',
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'plan_invalidated',
            MoveItErrorCodes.CONTROL_FAILED: 'control_failed',
            MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'sensor_data_unavailable',
            MoveItErrorCodes.TIMED_OUT: 'timed_out',
            MoveItErrorCodes.PREEMPTED: 'preempted',
            MoveItErrorCodes.START_STATE_IN_COLLISION: 'start_state_in_collision',
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: 'start_state_violates_constraints',
            MoveItErrorCodes.GOAL_IN_COLLISION: 'goal_in_collision',
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'goal_violates_constraints',
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'goal_constraints_violated',
            MoveItErrorCodes.INVALID_GROUP_NAME: 'invalid_group_name',
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'invalid_goal_constraints',
            MoveItErrorCodes.INVALID_ROBOT_STATE: 'invalid_robot_state',
            MoveItErrorCodes.INVALID_LINK_NAME: 'invalid_link_name',
            MoveItErrorCodes.INVALID_OBJECT_NAME: 'invalid_object_name',
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'frame_transform_failure',
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: 'collision_checking_unavailable',
            MoveItErrorCodes.ROBOT_STATE_STALE: 'robot_state_stale',
            MoveItErrorCodes.SENSOR_INFO_STALE: 'sensor_info_stale',
            MoveItErrorCodes.COMMUNICATION_FAILURE: 'communication_failure',
            MoveItErrorCodes.NO_IK_SOLUTION: 'no_ik_solution',
        }
        return known_codes.get(error_code, f'ik_failed:{error_code}')


def main(args=None):
    rclpy.init(args=args)
    node = SurfaceIkServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
