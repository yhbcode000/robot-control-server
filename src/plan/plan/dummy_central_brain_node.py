import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DummyBrainNode(Node):
    def __init__(self):
        super().__init__('dummy_brain_node')
        self.cmd_pub = self.create_publisher(String, '/cmd_topic', 10)
        self.create_subscription(String, '/language_query', self.dummy_chatbot, 10)
        self.get_logger().info('DummyBrainNode has been started.')

    def dummy_chatbot(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f'Received message: {text}')
        command_map = {
            "forward": "W",
            "back": "S",
            "left": "A",
            "right": "D",
            "stop": "X",
            "q": "Q",
            "e": "E",
        }
        last_cmd = None
        last_index = -1
        for word, cmd in command_map.items():
            idx = text.rfind(word)
            if idx != -1 and idx > last_index:
                last_index = idx
                last_cmd = cmd

        if last_cmd:
            self.publish_cmd(last_cmd)
        else:
            self.get_logger().warn(f'Unknown command: {text}')

    def publish_cmd(self, cmd):
        cmd_msg = String()
        cmd_msg.data = cmd
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Published command: {cmd}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyBrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()