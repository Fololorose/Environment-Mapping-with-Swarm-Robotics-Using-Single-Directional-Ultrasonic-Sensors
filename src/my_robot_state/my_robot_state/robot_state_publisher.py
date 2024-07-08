#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create a TransformBroadcaster with QoSProfile
        qos_profile = QoSProfile(depth=100)  # Example QoS profile with a depth of 100
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        # Start timer for publishing transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)

        self.get_logger().info('Robot State Publisher started')

    def publish_transforms(self):
        current_time = self.get_clock().now().to_msg()
        
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0

        odom_to_base_footprint = TransformStamped()
        odom_to_base_footprint.header.stamp = current_time
        odom_to_base_footprint.header.frame_id = 'odom'
        odom_to_base_footprint.child_frame_id = 'base_footprint'
        odom_to_base_footprint.transform.translation.x = 0.0
        odom_to_base_footprint.transform.translation.y = 0.0
        odom_to_base_footprint.transform.translation.z = 0.0
        odom_to_base_footprint.transform.rotation.x = 0.0
        odom_to_base_footprint.transform.rotation.y = 0.0
        odom_to_base_footprint.transform.rotation.z = 0.0
        odom_to_base_footprint.transform.rotation.w = 1.0

        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = current_time
        base_footprint_to_base_link.header.frame_id = 'base_footprint'
        base_footprint_to_base_link.child_frame_id = 'base_link'
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.0
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0

        # Publish transforms
        self.broadcaster.sendTransform(map_to_odom)
        self.broadcaster.sendTransform(odom_to_base_footprint)
        self.broadcaster.sendTransform(base_footprint_to_base_link)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
