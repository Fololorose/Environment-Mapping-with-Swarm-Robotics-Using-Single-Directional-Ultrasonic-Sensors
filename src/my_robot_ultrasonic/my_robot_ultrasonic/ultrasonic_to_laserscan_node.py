#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
import numpy as np

class UltrasonicToLaserScanNode(Node):

    def __init__(self):
        super().__init__("ultrasonic_to_laserscan_node")

        # Create subscribers for each Range topic
        self.left_range_subscriber_ = self.create_subscription(
            Range, "/my_robot/range/left", self.range_callback, 100)
        self.center_range_subscriber_ = self.create_subscription(
            Range, "/my_robot/range/center", self.range_callback, 100)
        self.right_range_subscriber_ = self.create_subscription(
            Range, "/my_robot/range/right", self.range_callback, 100)
        
        # Create publishers for each LaserScan topic
        self.left_laserscan_publisher = self.create_publisher(
            LaserScan, "/my_robot/laser_scan/left", 100)
        self.center_laserscan_publisher = self.create_publisher(
            LaserScan, "/scan", 100)
        self.right_laserscan_publisher = self.create_publisher(
            LaserScan, "/my_robot/laser_scan/right", 100)
        
        # Initialise dictionaries to store the range readings for each sensor
        self.range_dict = {
            "left": [],
            "center": [],
            "right": []
        }

        # Initialise a dictionary to store the latest range readings
        self.latest_ranges = {
            "left": float('inf'),
            "center": float('inf'),
            "right": float('inf')
        }

        # Counter to track number of updates
        self.update_counter = 0

    def range_callback(self, msg: Range):
        frame_id = msg.header.frame_id
        if frame_id == "left_ultrasonic_frame":
            self.process_range(msg, "left", self.left_laserscan_publisher)
        elif frame_id == "center_ultrasonic_frame":
            self.process_range(msg, "center", self.center_laserscan_publisher)
        elif frame_id == "right_ultrasonic_frame":
            self.process_range(msg, "right", self.right_laserscan_publisher)

    def process_range(self, msg: Range, frame_id: str, publisher):
        range_in_m = msg.range  # Range is already in meters

        # Append the new range reading to the appropriate list
        if frame_id in self.range_dict:
            self.range_dict[frame_id].append(range_in_m)
        
            # Ensure the list does not exceed 360 readings
            if len(self.range_dict[frame_id]) == 361:
                self.range_dict[frame_id].pop(0)  # Remove the oldest reading
            
            # Create and publish the LaserScan message
            scan_msg = self.range_to_laserscan(frame_id)
            publisher.publish(scan_msg)

            # Update the latest range reading
            self.latest_ranges[frame_id] = range_in_m

            # Increment update counter
            self.update_counter += 1

            # Check if all three sensors have updated
            if self.update_counter == 3:
                # Print the ranges for all three sensors
                self.get_logger().info(
                    f"Left: {self.latest_ranges['left'] * 100:.2f} cm, " +
                    f"Center: {self.latest_ranges['center'] * 100:.2f} cm, " +
                    f"Right: {self.latest_ranges['right'] * 100:.2f} cm"
                )

                # Reset the update counter
                self.update_counter = 0

    def range_to_laserscan(self, frame_id: str) -> LaserScan:
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()  # Set timestamp
        scan_msg.header.frame_id = "base_link"  # Use a common frame_id for all LaserScan messages
        scan_msg.angle_min = -np.pi  # -180 degrees in radians
        scan_msg.angle_max = np.pi   # 180 degrees in radians
        scan_msg.angle_increment = np.pi / 180  # 1 degree resolution in radians
        scan_msg.time_increment = 0.0  # Time between measurements [s]
        scan_msg.scan_time = 0.1  # Time between scans [s]
        scan_msg.range_min = 0.01  # Minimum range value [m]
        scan_msg.range_max = 4.0  # Maximum range value [m]
        
        # Directly use the ranges from self.range_dict[frame_id]
        ranges = self.range_dict[frame_id]
        scan_msg.ranges = ranges  # Set the ranges
        scan_msg.intensities = []  # Optional: can be used to store intensity values

        return scan_msg

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS client library
    node = UltrasonicToLaserScanNode()  # Create an instance of the UltrasonicToLaserScanNode
    rclpy.spin(node)  # Spin the node to keep it active and listening for messages
    rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == '__main__':
    main()

