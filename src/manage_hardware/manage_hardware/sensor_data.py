import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# from std_msgs.msg import String, Float64
from geometry_msgs.msg import Vector3


class FTSensor(Node):

    def __init__(self):
        super().__init__('FTSensor')
        qos_profile = QoSProfile(depth=10)
        self.sensor_publisher = self.create_publisher(
            Vector3,
            'sensor_data',
            qos_profile)
        
        self.timer = self.create_timer(0.01, self.publish_sensor)

    def publish_sensor(self):
        sensor = Vector3()
        sensor.x = float(1)
        sensor.y = float(1)
        sensor.z = float(1)
        self.sensor_publisher.publish(sensor)
        self.get_logger().info('Published sensor: {0}, {1}, {2}'.format(sensor.x, sensor.y, sensor.z))


def main(args=None):
    rclpy.init(args=args)
    node = FTSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()