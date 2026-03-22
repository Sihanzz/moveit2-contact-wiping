#!/usr/bin/env python3
"""
Helper module to get current joint states for IK requests.
"""

import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState


class JointStateHelper:
    """Caches the latest joint state for use in IK requests."""

    def __init__(self, node):
        self.node = node
        self.latest_joint_state = None

        # Subscribe to joint states
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.node.get_logger().info('Waiting for joint states...')

    def joint_state_callback(self, msg):
        """Store the latest joint state."""
        self.latest_joint_state = msg

    def get_robot_state(self):
        """Get a RobotState message with current joint positions."""
        robot_state = RobotState()

        if self.latest_joint_state is not None:
            robot_state.joint_state = self.latest_joint_state
            robot_state.is_diff = False

        return robot_state

    def has_joint_state(self):
        """Check if we've received at least one joint state."""
        return self.latest_joint_state is not None
