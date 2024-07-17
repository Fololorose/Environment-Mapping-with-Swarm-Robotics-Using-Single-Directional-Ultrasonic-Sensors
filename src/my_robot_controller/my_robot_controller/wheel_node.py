#!/usr/bin/env python3
import time
from .PCA9685 import PCA9685
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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
    pulse1 = 0
    rotateWheel()
    print("Turn Left")

def turnRight():
    global pulse0, pulse1
    pulse0 = 0
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

        # Create a subscriber for the keyboard control
        self.cmd_subscriber = self.create_subscription(
            String, "/cmd", self.cmd_callback, 10)
        
        # Timer to stop the robot if no command is received for a certain period
        self.timeout = 1.0  # 1 second timeout
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout) 
        
    def cmd_callback(self, msg):
        # Update the last command time when it's not in fixing mode
        self.last_cmd_time = self.get_clock().now()
                
        # Perform the action based on the received command
        command = msg.data.lower()
        
        if command == "forward":
            # forward()
            pass
        elif command == "left":
            # turnLeft()
            pass
        elif command == 'right':
            # turnRight()
            pass
        else:
            stop()

        self.get_logger().info(f'Action: {command}')
    
    def check_timeout(self):
        # Stop the robot if no command is received for the specified timeout
        if (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 > self.timeout:
            stop()

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
