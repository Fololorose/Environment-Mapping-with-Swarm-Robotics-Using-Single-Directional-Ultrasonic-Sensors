#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_ultrasonic.ultrasonic import Ultrasonic
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO

class UltrasonicNode(Node):

    def __init__(self):
        super().__init__("ultrasonic_node")

        # Create publishers for the Range data for each sensor
        self.left_range_publisher_ = self.create_publisher(
            Range, "my_robot/range/left", 10)
        self.center_range_publisher_ = self.create_publisher(
            Range, "my_robot/range/center", 10)
        self.right_range_publisher_ = self.create_publisher(
            Range, "my_robot/range/right", 10)
        
        # Create a timer to call the send_distance_command function periodically
        self.timer_ = self.create_timer(1.0, self.send_distance_command)
        
        # Initialise the Ultrasonic sensors
        self.ultrasonics = [
            {"sensor": Ultrasonic(gpio_trigger=4, gpio_echo=17), "frame_id": "left_ultrasonic_frame", "publisher": self.left_range_publisher_},
            {"sensor": Ultrasonic(gpio_trigger=27, gpio_echo=22), "frame_id": "center_ultrasonic_frame", "publisher": self.center_range_publisher_},
            {"sensor": Ultrasonic(gpio_trigger=6, gpio_echo=5), "frame_id": "right_ultrasonic_frame", "publisher": self.right_range_publisher_}
        ]
        
        # Log that the node has been started
        self.get_logger().info("Ultrasonic node has been started")

    def send_distance_command(self):
        distances = {"left": -1, "center": -1, "right": -1}

        # Loop through each sensor, get the distance and publish the range
        for ultrasonic_info in self.ultrasonics:
            distance = ultrasonic_info["sensor"].get_range()

            range_msg = self.create_range_message(distance, ultrasonic_info["frame_id"])
            ultrasonic_info["publisher"].publish(range_msg)
                    
            # Store the distance in the distances dictionary
            if ultrasonic_info["frame_id"] == "left_ultrasonic_frame":
                distances["left"] = distance
            elif ultrasonic_info["frame_id"] == "center_ultrasonic_frame":
                distances["center"] = distance
            elif ultrasonic_info["frame_id"] == "right_ultrasonic_frame":
                distances["right"] = distance

        # Log all distances together
        left_distance = f"{distances['left']:.2f} cm"
        center_distance = f"{distances['center']:.2f} cm"
        right_distance = f"{distances['right']:.2f} cm"

        self.get_logger().info(f"Left: {left_distance}, Center: {center_distance}, Right: {right_distance}")


    def create_range_message(self, distance, frame_id):
        # Create a single Range message
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()  # Set the timestamp
        range_msg.header.frame_id = frame_id  # Frame ID of the sensor
        range_msg.radiation_type = Range.ULTRASOUND  # Specify the type of radiation (ultrasonic)
        range_msg.field_of_view = 0.1  # Example value, adjust as necessary
        range_msg.min_range = self.ultrasonics[0]["sensor"]._range_min / 100.0  # Convert min range to meters
        range_msg.max_range = self.ultrasonics[0]["sensor"]._range_max / 100.0  # Convert max range to meters
        range_msg.range = distance / 100.0  # Convert distance to meters
        return range_msg

def main(args=None):
    rclpy.init(args=args)  # Initialise the ROS client library
    ultrasonic_node = UltrasonicNode()  # Create an instance of the UltrasonicNode

    try:
        rclpy.spin(ultrasonic_node)  # Spin the node to keep it active and listening for messages
    except KeyboardInterrupt:
        pass  # Handle the interruption 
    finally:
        ultrasonic_node.destroy_node()  # Clean up and destroy the node
        rclpy.shutdown()  # Shutdown the ROS client library
        GPIO.cleanup()  # Clean up GPIO pins

if __name__ == '__main__':
    main()  # Execute the main function if this script is run directly
