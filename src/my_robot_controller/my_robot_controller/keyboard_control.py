#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import termios, sys, tty

class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(String, "/cmd", 10)
        self.get_logger().info('Keyboard Control started. Use W/S/A/D to move, F to enter fixing mode and Q to exit fixing mode.')
        
        # Save the terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno()) # Set the terminal to raw mode to capture single key presses
        key = sys.stdin.read(1) # Read a single character from the terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings) # Restore the terminal settings
        return key
    
    def run(self):
        try:
            while True:
                key = self.get_key(self.settings)
                if key == "\x03":  # CTRL+C
                    break
                
                msg = String()
                if key.lower() == "w":
                    msg.data = "forward"
                elif key.lower() == "a":
                    msg.data = "left"
                elif key.lower() == "d":
                    msg.data = "right"
                elif key.lower() == "f":
                    msg.data = "enter fixing mode"
                elif key.lower() == "q":
                    msg.data = "exit fixing mode"
                else:
                    msg.data = "stop"
                    
                self.publisher_.publish(msg)
                self.get_logger().info(f'Sent: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Exception: {e}')

        finally:
            # Restore the terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)  # Initialise the ROS client library
    node = KeyboardControl()  # Create an instance of the UltrasonicNode
    node.run()

if __name__ == '__main__':
    main() # Execute the main function if this script is run directly
