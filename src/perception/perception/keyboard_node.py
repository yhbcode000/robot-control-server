import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(String, 'perception/keyboard_input', 10)
        self.get_logger().info('Keyboard node has been started. Press keys to publish them.')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                msg = String()
                msg.data = key
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: {key}')
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()