import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#!/usr/bin/env python3
"""
This node monitors keyboard input in a non-blocking manner and publishes each character
to the 'keyboard_input' topic. It uses Python's select module to avoid blocking on user input.

Usage:
    1. Ensure ROS environment is correctly sourced.
    2. Run this node (e.g., `ros2 run perception keyboard_node`).
    3. Each received key is published as a String message.
"""


class KeyboardNode(Node):
    """
    ROS2 node that reads keyboard input and publishes it as String messages.
    Clears any queued input to avoid overwhelming the message system.
    """
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher = self.create_publisher(String, 'keyboard_input', 10)
        self.timer = self.create_timer(0.1, self.publish_key)
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)

    def publish_key(self):
        typed_chars = []
        while select.select([sys.stdin], [], [], 0)[0]:
            typed_chars.append(sys.stdin.read(1))
        if typed_chars:
            self.publisher.publish(String(data=typed_chars[-1]))

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
