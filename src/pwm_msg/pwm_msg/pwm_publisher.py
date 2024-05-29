import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class PwmPublisher(Node):

    def __init__(self):
        super().__init__('pwm_publisher')
        qos_profile = QoSProfile(depth=10)
        self.resolution = 16
        self.pwm_range = pow(2,16)
        # pwm initialize
        self.pwm_neut = self.pwm_range * (0.075)
        self.pwm = self.pwm_neut
        self.pwm_key_subscriber = self.create_subscription(
            String,
            'keyboard_input',
            self.pwm_order,
            qos_profile
        )

        self.pwm_publisher = self.create_publisher(
            String,
            'pwm_write',
            qos_profile)
        
        self.timer = self.create_timer(0.01, self.publish_pwm)

    def pwm_order(self, msg):
        if msg.data == 'w':
            self.pwm += 5
        elif msg.data =='s':
            self.pwm -= 5
        elif msg.data == 'q':
            self.pwm = self.pwm_neut


    def publish_pwm(self):
        msg = String()
        msg.data = str(self.pwm)
        self.pwm_publisher.publish(msg)
        self.get_logger().info('Published pwm: {0}'.format(msg.data))
        # if self.count == 100:
        #     self.count = 1
        # else:
        #     self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = PwmPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()