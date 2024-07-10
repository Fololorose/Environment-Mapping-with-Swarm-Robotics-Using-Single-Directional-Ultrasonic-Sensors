#!/usr/bin/env python3

import time
from .PCA9685 import PCA9685
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Servo configuration
pwm = PCA9685(0x40)
pwm.setPWMFreq(50)

# Initialise pulses
pulse0 = 0
pulse1 = 0

def stop():
    global pulse0, pulse1
    pulse0 = 0
    pulse1 = 0
    rotateWheel()
    print("Stop")

def forward():
    global pulse0, pulse1
    pulse0 = 1370
    pulse1 = 1700
    rotateWheel()
    print("Forward")

def backward():
    global pulse0, pulse1
    pulse0 = 1700
    pulse1 = 1370
    rotateWheel()
    print("Backward")

def turnLeft():
    global pulse0, pulse1
    pulse0 = 1370
    pulse1 = 1370
    rotateWheel()
    print("Turn Left")

def turnRight():
    global pulse0, pulse1
    pulse0 = 1700
    pulse1 = 1700
    rotateWheel()
    print("Turn Right")

def rotateWheel():
    global pulse0, pulse1, pwm
    # Rotate the wheels
    pwm.setServoPulse(0, pulse0)
    pwm.setServoPulse(1, pulse1)
    print(f"Rotating Wheels: pulse0={pulse0}, pulse1={pulse1}")

class WheelNode(Node):

    def __init__(self):
        super().__init__('wheel_node')

        # Create a subscriber for laser scan messages
        self.center_laserscan_subscriber = self.create_subscription(
            LaserScan, '/my_robot/laser_scan/center', self.laserscan_callback, 10)

        # Initialise distance variable
        self.distance = float('inf') 
        
        # Flag to check if the robot is in turning mode 
        self.turning = False  
        
    def laserscan_callback(self, msg):
        # Update the distance variable with the latest laser scan reading
        if msg.ranges: # Check if ranges list is not empty
            self.distance = msg.ranges[-1]  # Get the latest range value
            print(msg.ranges)
        
        # Check if distance is less than 50 cm
        if 0 < self.distance < 0.5 and not self.turning:  # ranges are in meters
            self.turning = True
            stop()
            time.sleep(1)  # Wait for 1 second to provide buffer
            turnLeft()    # Perform turn left action
            time.sleep(1)  # Wait for 1 second to provide buffer
        
        # If the robot is turning, check if the distance is greater than 50 cm to stop turning
        elif self.turning and self.distance > 0.5:
            stop()
            time.sleep(1)  # Wait for 1 second to provide buffer
            self.turning = False

        # Move forward if distance is greater than 50 cm and not in turning mode
        if not self.turning:
            forward()
            time.sleep(1)  # Wait for 1 second to provide buffer

def main(args=None):
    rclpy.init(args=args) # Initialise the ROS client library
    node = WheelNode() # Create an instance of the WheelNode
    try:
        rclpy.spin(node) # Spin the node to keep it active and listening for messages
    except KeyboardInterrupt:
        pass # Handle the interruption 
    finally:
        stop() # Ensure the robot stops moving before shutdown the code
        rotateWheel()
        node.destroy_node() # Clean up and destroy the node
        rclpy.shutdown() # Shutdown the ROS client library

if __name__ == '__main__':
    main() # Execute the main function if this script is run directly
