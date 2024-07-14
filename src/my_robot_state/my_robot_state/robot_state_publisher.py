# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

# class RobotStatePublisher(Node):

#     def __init__(self):
#         super().__init__('robot_state_publisher')

#         # Create a TransformBroadcaster with QoSProfile
#         qos_profile = QoSProfile(depth=100)  # Example QoS profile with a depth of 100
#         self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

#         # Start timer for publishing transforms
#         self.timer = self.create_timer(0.1, self.publish_transforms)

#         self.get_logger().info('Robot State Publisher started')

#     def publish_transforms(self):
#         current_time = self.get_clock().now().to_msg()
        
#         map_to_odom = TransformStamped()
#         map_to_odom.header.stamp = current_time
#         map_to_odom.header.frame_id = 'map'
#         map_to_odom.child_frame_id = 'odom'
#         map_to_odom.transform.translation.x = 0.0
#         map_to_odom.transform.translation.y = 0.0
#         map_to_odom.transform.translation.z = 0.0
#         map_to_odom.transform.rotation.x = 0.0
#         map_to_odom.transform.rotation.y = 0.0
#         map_to_odom.transform.rotation.z = 0.0
#         map_to_odom.transform.rotation.w = 1.0

#         odom_to_base_footprint = TransformStamped()
#         odom_to_base_footprint.header.stamp = current_time
#         odom_to_base_footprint.header.frame_id = 'odom'
#         odom_to_base_footprint.child_frame_id = 'base_footprint'
#         odom_to_base_footprint.transform.translation.x = 0.0
#         odom_to_base_footprint.transform.translation.y = 0.0
#         odom_to_base_footprint.transform.translation.z = 0.0
#         odom_to_base_footprint.transform.rotation.x = 0.0
#         odom_to_base_footprint.transform.rotation.y = 0.0
#         odom_to_base_footprint.transform.rotation.z = 0.0
#         odom_to_base_footprint.transform.rotation.w = 1.0

#         base_footprint_to_base_link = TransformStamped()
#         base_footprint_to_base_link.header.stamp = current_time
#         base_footprint_to_base_link.header.frame_id = 'base_footprint'
#         base_footprint_to_base_link.child_frame_id = 'base_link'
#         base_footprint_to_base_link.transform.translation.x = 0.0
#         base_footprint_to_base_link.transform.translation.y = 0.0
#         base_footprint_to_base_link.transform.translation.z = 0.0
#         base_footprint_to_base_link.transform.rotation.x = 0.0
#         base_footprint_to_base_link.transform.rotation.y = 0.0
#         base_footprint_to_base_link.transform.rotation.z = 0.0
#         base_footprint_to_base_link.transform.rotation.w = 1.0

#         # Publish transforms
#         self.broadcaster.sendTransform(map_to_odom)
#         self.broadcaster.sendTransform(odom_to_base_footprint)
#         self.broadcaster.sendTransform(base_footprint_to_base_link)

# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotStatePublisher()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class RobotStatePublisher(Node):

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create a TransformBroadcaster with QoSProfile
        qos_profile = QoSProfile(depth=100)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        # Start timer for publishing transforms
        self.timer = self.create_timer(0.1, self.publish_transforms)

        # Initialize the robot's position and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Movement sequence variables
        self.step_count = 0
        self.state = 'forward'
        self.get_logger().info('Robot State Publisher started')

        # Start a timer for issuing movement commands
        self.command_timer = self.create_timer(1.0, self.issue_movement_command)

    def issue_movement_command(self):
        if self.state == 'forward':
            if self.step_count < 5:
                self.move_forward()
                self.step_count += 1
            else:
                self.step_count = 0
                self.state = 'turn_left'
        elif self.state == 'turn_left':
            self.turn_left()
            self.state = 'forward'
            self.step_count = 0

    def move_forward(self):
        self.current_x += 0.1 * math.cos(self.current_theta)
        self.current_y += 0.1 * math.sin(self.current_theta)
        self.get_logger().info('Moving forward')

    def turn_left(self):
        self.current_theta += math.pi / 2  # 90-degree turn
        self.get_logger().info('Turning left')

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
        odom_to_base_footprint.transform.translation.x = self.current_x
        odom_to_base_footprint.transform.translation.y = self.current_y
        odom_to_base_footprint.transform.translation.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.current_theta)
        odom_to_base_footprint.transform.rotation.x = q[0]
        odom_to_base_footprint.transform.rotation.y = q[1]
        odom_to_base_footprint.transform.rotation.z = q[2]
        odom_to_base_footprint.transform.rotation.w = q[3]

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
