from FT_SENSOR import FTSensor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3


class Sensor(Node):

    def __init__(self):
        super().__init__('FTSensor')
        self.sensor = FTSensor(port='/dev/ttyUSB0')
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

    def publish_sensor(self):
        force = Vector3()
        torque = Vector3()
        data = self.sensor.data_read_and_process()
        if self.sensor.new_data_available:
            # Convert the data to a string or the appropriate ROS 2 message format
            self.sensor.new_data_available = False

            force.x = float(data[0])
            force.y = float(data[1])
            force.z = float(data[2])
            torque.x = float(data[3])
            torque.y = float(data[4])
            torque.z = float(data[5])
            self.force_publisher.publish(force)
            self.torque_publisher.publish(torque)
            self.get_logger().info('force: (x:{0}), (y:{1}), (z:{2})'.format(force.x, force.y, force.z))
            self.get_logger().info('torque: (x:{0}), (y:{1}), (z:{2})'.format(torque.x, torque.y, torque.z))

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