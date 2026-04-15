import os
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class SceneSetupNode(Node):
    def __init__(self):
        super().__init__('scene_setup')

        self.publisher_ = self.create_publisher(CollisionObject, '/collision_object', 10)

        package_share = get_package_share_directory('surface_wiping_nodes')
        scene_config_path = os.path.join(package_share, 'config', 'scene.yaml')

        self.get_logger().info(f'Loading scene config from: {scene_config_path}')

        with open(scene_config_path, 'r') as f:
            self.scene_config = yaml.safe_load(f)

        self.world_frame = self.scene_config['world_frame']

        self.timer = self.create_timer(1.0, self.publish_scene_once)
        self.published = False

    def make_box_object(self, object_id, size_xyz, position_xyz, orientation_xyzw=None):
        obj = CollisionObject()
        obj.header.frame_id = self.world_frame
        obj.id = object_id
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size_xyz

        pose = Pose()
        pose.position.x = float(position_xyz[0])
        pose.position.y = float(position_xyz[1])
        pose.position.z = float(position_xyz[2])
        if orientation_xyzw:
            pose.orientation.x = float(orientation_xyzw[0])
            pose.orientation.y = float(orientation_xyzw[1])
            pose.orientation.z = float(orientation_xyzw[2])
            pose.orientation.w = float(orientation_xyzw[3])
        else:
            pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)

        return obj

    def publish_scene_once(self):
        if self.published:
            return

        for key in ['countertop', 'faucet', 'mirror']:
            entry = self.scene_config[key]
            obj = self.make_box_object(
                object_id=entry['id'],
                size_xyz=entry['size'],
                position_xyz=entry['position'],
                orientation_xyzw=entry.get('orientation'),
            )
            self.publisher_.publish(obj)
            self.get_logger().info(
                f"Published collision object: {entry['id']} "
                f"size={entry['size']} pos={entry['position']}"
            )

        self.published = True
        self.get_logger().info('Scene objects published.')
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SceneSetupNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
