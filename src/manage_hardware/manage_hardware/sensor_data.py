from FT_SENSOR import FTSensor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3
from custominterface.srv import Status

class Sensor(Node):

    def __init__(self):
        super().__init__('FT_data')
        self.sensor = FTSensor(port='/dev/ttyUSB0')

        self.srv = self.create_service(
            Status,
            'sensor_server',
            self.sensor_server
        )
        self.force_publisher = self.create_publisher(
            Vector3,
            'force_data',
            10)
        self.torque_publisher = self.create_publisher(
            Vector3,
            'torque_data',
            10)
        
        self.init_sensor()
        self.timer = self.create_timer(0.001, self.publish_sensor)
        
    def init_sensor(self):
        if not self.sensor.ft_sensor_init():
            self.get_logger().error('Failed to initialize sensor')
            raise Exception("Sensor initialization failed")
        
    def sensor_server(self, request, response):
        if request.sensor_bias_set == True:
            self.bias_set_btn = True
            result = self.sensor.ft_sensor_bias_set()
            if result == True:
                self.get_logger().info('FT sensor bias 성공')
            elif result == False:
                self.get_logger().info('FT sensor bias 실패')
        self.bias_set_btn = False

    def publish_sensor(self):
        force = Vector3()
        torque = Vector3()
        try:
            data = self.sensor.data_read_and_process()
            if data:
                force.x, force.y, force.z = data[:3]
                torque.x, torque.y, torque.z = data[3:6]
                self.force_publisher.publish(force)
                self.torque_publisher.publish(torque)
        except Exception as e:
            self.get_logger().error(f'Error reading sensor data: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Sensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()