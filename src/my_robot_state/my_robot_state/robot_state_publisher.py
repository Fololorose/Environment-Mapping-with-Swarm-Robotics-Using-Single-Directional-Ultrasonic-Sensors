#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
import math

class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__("robot_state_publisher")

        # Create a TransformBroadcaster with QoSProfile
        qos_profile = QoSProfile(depth=100)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        # Start timer for publishing transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)

        # Initialize the robot's position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Fixing mode state
        self.fixing_mode = False

        # Create a subscriber to receive the action
        self.action_subscriber = self.create_subscription(
            String, "/cmd", self.action_callback, 10)

        self.get_logger().info("Robot State Publisher started")

        # Start a timer for issuing movement commands
        self.command_timer = self.create_timer(1.0, self.publish_transforms)

    def action_callback(self, msg):
        action = msg.data
        if action == "enter fixing mode":
            self.fixing_mode = True
        elif action == "exit fixing mode":
            self.fixing_mode = False
        elif not self.fixing_mode:
            if action == "forward":
                self.move_forward()
            elif action == "backward":
                self.move_backward()
            elif action == "left":
                self.turn_left()
            elif action == "right":
                self.turn_right()
            elif action == "stop":
                self.stop()

    def move_forward(self):
        self.current_x += 0.05 * math.cos(self.current_theta)
        self.current_y += 0.05 * math.sin(self.current_theta)
        self.get_logger().info("Moving forward")
        
    def move_backward(self):
        self.current_x -= 0.05 * math.cos(self.current_theta)
        self.current_y -= 0.05 * math.sin(self.current_theta)
        self.get_logger().info("Moving backward")

    def turn_left(self):
        self.current_theta += math.pi / 8 # 90-degree turn
        self.get_logger().info("Turning left")
        
    def turn_right(self):
        self.current_theta -= math.pi / 12 # 90-degree turn
        self.get_logger().info("Turning right")
        
    def stop(self):
        self.get_logger().info("Stopping")

    def publish_transforms(self):
        current_time = self.get_clock().now().to_msg()

        # Create transform from "map" to "odom"
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = current_time
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0

        # Create transform from "odom" to "base_footprint"
        odom_to_base_footprint = TransformStamped()
        odom_to_base_footprint.header.stamp = current_time
        odom_to_base_footprint.header.frame_id = "odom"
        odom_to_base_footprint.child_frame_id = "base_footprint"
        odom_to_base_footprint.transform.translation.x = self.current_x
        odom_to_base_footprint.transform.translation.y = self.current_y
        odom_to_base_footprint.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.current_theta)
        odom_to_base_footprint.transform.rotation.x = q[0]
        odom_to_base_footprint.transform.rotation.y = q[1]
        odom_to_base_footprint.transform.rotation.z = q[2]
        odom_to_base_footprint.transform.rotation.w = q[3]

        # Create transform from "base_footprint" to "base_link"
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = current_time
        base_footprint_to_base_link.header.frame_id = "base_footprint"
        base_footprint_to_base_link.child_frame_id = "base_link"
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

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args) # Initialise the ROS client library
    node = RobotStatePublisher() # Create an instance of the WheelNode
    try:
        rclpy.spin(node) # Spin the node to keep it active and listening for messages
    except KeyboardInterrupt:
        pass # Handle the interruption 
    finally:
        node.destroy_node() # Clean up and destroy the node
        rclpy.shutdown() # Shutdown the ROS client library

if __name__ == '__main__':
    main() # Execute the main function if this script is run directly
