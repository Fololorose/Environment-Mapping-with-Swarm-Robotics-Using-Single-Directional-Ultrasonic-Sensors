import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import termios, sys, tty

class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(String, "/cmd", 10)
        self.get_logger().info('Keyboard Control Node has been started')
        self.run()

    def run(self):
        settings = termios.tcgetattr(sys.stdin) # Get the terminal settings to restore them later
        try:
            while True:
                key = self.get_key(settings)
                msg = String()
                if key == 'w':
                    msg.data = 'forward'
                elif key == 's':
                    msg.data = 'backward'
                elif key == 'a':
                    msg.data = 'left'
                elif key == 'd':
                    msg.data = 'right'
                else:
                    msg.data = 'stop'
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(str(e))
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # Restore the terminal settings

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno()) # Set the terminal to raw mode to capture single key presses
        key = sys.stdin.read(1) # Read a single character from the terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) # Restore the terminal settings
        return key

def main(args=None):
    rclpy.init(args=args)  # Initialise the ROS client library
    node = KeyboardControl()  # Create an instance of the UltrasonicNode

    try:
        rclpy.spin(node)  # Spin the node to keep it active and listening for messages
    except KeyboardInterrupt:
        pass  # Handle the interruption 
    finally:
        node.destroy_node()  # Clean up and destroy the node
        rclpy.shutdown()  # Shutdown the ROS client library

if __name__ == '__main__':
    main() # Execute the main function if this script is run directly
