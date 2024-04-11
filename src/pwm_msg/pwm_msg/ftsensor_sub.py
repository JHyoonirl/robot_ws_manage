import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import sys



class FTsensor(Node):
    
    def __init__(self):
        super().__init__('ftsensor')
        qos_profile = QoSProfile(depth=10)
        self.force_data = []
        self.torque_data = []
        self.data_receive = []

        self.force_receive = self.create_subscription(
            String,
            'force_data',
            self.force_sub,
            qos_profile
        )
        
        self.torque_receive = self.create_subscription(
            String,
            'torque_data',
            self.torque_sub,
            qos_profile
        )

        self.create_timer(0.01, self.data_sum)

        
        # self.timer = self.create_timer(0.01, self.publish_pwm)

    def force_sub(self, msg):
        # msg is the data which i want to receive
        # print(eval(msg.data))
        self.force_data = eval(msg.data)
        # if msg.data == 'w':
        #     self.pwm += 1
        # elif msg.data =='s':
        #     self.pwm -= 1
        # elif msg.data == 'q':
        #     self.pwm = 77
    def torque_sub(self, msg):
        # msg is the data which i want to receive
        self.torque_data = eval(msg.data)
    def data_sum(self):
        self.data_receive = self.force_data + self.torque_data
        self.get_logger().info('Received FT sensor: {0}'.format(self.data_receive))
        # print(data_receive)


    # def publish_pwm(self):
    #     msg = String()
    #     msg.data = str(self.pwm)
    #     self.pwm_publisher.publish(msg)
    #     self.get_logger().info('Published pwm: {0}'.format(msg.data))
    #     # if self.count == 100:
    #     #     self.count = 1
    #     # else:
    #     #     self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = FTsensor()

    # aw.show()
    try:
        # aw.show()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # sys.exit(qApp.exec_())


if __name__ == '__main__':
    main()
    
    # aw.show()
    # sys.exit(qApp.exec_())