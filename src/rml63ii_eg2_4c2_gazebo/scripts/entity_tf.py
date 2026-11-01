#!/usr/bin/python3
"""Publish actual model TF and canonical joint feedback for the robot."""
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState


class EntityTransform(Node):
    def __init__(self):
        super().__init__('robot_entity_transform')
        self.child = self.declare_parameter('child_frame', 'arm/world').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.update, 10)
        self.joints = self.create_publisher(JointState, '/arm/joint_states', 10)
        self.create_subscription(JointState, '/arm/joint_state_broadcaster/joint_states',
                                 self.joint_feedback, 10)

    def joint_feedback(self, state):
        # Humble gazebo_ros2_control exports measured mimic interfaces with a
        # _mimic suffix. Keep the public feedback aligned with the URDF names.
        state.name = [name.removesuffix('_mimic') for name in state.name]
        self.joints.publish(state)

    def update(self, state):
        if 'rml63ii_eg2_4c2' not in state.name:
            return
        pose = state.pose[state.name.index('rml63ii_eg2_4c2')]
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = self.child
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = EntityTransform()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
