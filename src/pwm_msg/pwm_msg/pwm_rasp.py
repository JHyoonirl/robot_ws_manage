import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from custominterface.srv import Status
from std_msgs.msg import String, Float64


class PwmServer(Node):

    def __init__(self):
        super().__init__('pwm_server')
        qos_profile = QoSProfile(depth=10)
        self.resolution = 16
        # self.pwm_range = pow(2,16)
        # pwm initialize
        self.pwm_neut = int(50)
        self.pwm = self.pwm_neut
        self.srv = self.create_service(
            Status,
            'pwm_server',
            self.pwm_server
        )
        
        self.pwm_publisher = self.create_subscription(
            Float64,
            'pwm_signal',
            self.pwm_subscriber,
            qos_profile)

    def pwm_server(self, request, response):
        if request.pwm_switch == True:
            response.pwm_result = True
        elif request.pwm_switch == False:
            response.pwm_result = False
        self.response = response.pwm_result
        print(self.response)
        return response

    def pwm_subscriber(self, msg):
        self.get_logger().info('Subscribed pwm: {0}'.format(msg.data))
        # pass
        


def main(args=None):
    rclpy.init(args=args)
    node = PwmServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()