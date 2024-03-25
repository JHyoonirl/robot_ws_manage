import rclpy
import select
import sys
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

if os.name == 'posix':
    import termios
    import tty
    settings = termios.tcgetattr(sys.stdin)


class KeyboardInput(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        qos_profile = QoSProfile(depth=10)

        settings = termios.tcgetattr(sys.stdin)
        self.settings = settings

        self.keyboard_publisher = self.create_publisher(
            String,
            'keyboard_input',
            qos_profile)
        self.timer = self.create_timer(0.01, self.publish_keyboard)

    def publish_keyboard(self):
        msg = String()
        key = self.get_key()
        msg.data = str(key)
        self.keyboard_publisher.publish(msg)
        self.get_logger().info('Published keyboard: {0}'.format(msg.data))

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key



def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()