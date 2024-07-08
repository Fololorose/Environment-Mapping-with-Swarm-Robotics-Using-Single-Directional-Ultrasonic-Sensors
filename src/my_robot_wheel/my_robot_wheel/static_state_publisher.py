#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class StaticStatePublisher(Node):

    def __init__(self):
        super().__init__('static_state_publisher')

        qos_profile = QoSProfile(depth=100)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.static_broadcaster = StaticTransformBroadcaster(self, qos=qos_profile)

        self.get_logger().info("Static State Publisher started")

        self.publish_static_transforms()

        self.timer = self.create_timer(0.1, self.publish_dynamic_transforms)

    def publish_static_transforms(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'
        static_transform.child_frame_id = 'odom'
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(static_transform)

    def publish_dynamic_transforms(self):
        dynamic_transform = TransformStamped()
        dynamic_transform.header.stamp = self.get_clock().now().to_msg()
        dynamic_transform.header.frame_id = 'odom'
        dynamic_transform.child_frame_id = 'base_link'
        dynamic_transform.transform.translation.x = 0.0
        dynamic_transform.transform.translation.y = 0.0
        dynamic_transform.transform.translation.z = 0.0
        dynamic_transform.transform.rotation.x = 0.0
        dynamic_transform.transform.rotation.y = 0.0
        dynamic_transform.transform.rotation.z = 0.0
        dynamic_transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(dynamic_transform)

def main(args=None):
    rclpy.init(args=args) # Initialise the ROS client library
    node = StaticStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
