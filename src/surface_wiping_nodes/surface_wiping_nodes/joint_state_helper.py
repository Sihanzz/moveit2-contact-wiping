from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class JointStateHelper:
    """Caches the latest joint state for seeding IK requests."""

    def __init__(self, node):
        self.node = node
        self.latest_joint_state = None
        self.subscription = self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def has_joint_state(self):
        return self.latest_joint_state is not None

    def get_robot_state(self):
        robot_state = RobotState()
        if self.latest_joint_state is not None:
            robot_state.joint_state = self.latest_joint_state
            robot_state.is_diff = False
        return robot_state
