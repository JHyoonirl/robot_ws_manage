from FT_SENSOR import FTSensor
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Adjust message type based on your actual data structure

class SensorVisual(Node):
    def __init__(self):
        super().__init__('sensor')
        self.sensor = FTSensor(port='/dev/ttyUSB0')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)  # Adjust topic type and size
        self.timer = self.create_timer(0.5, self.update_labels)  # Adjust timer interval as needed
        self.init_sensor()

    def init_sensor(self):
        if not self.sensor.ft_sensor_init():
            self.get_logger().error('Failed to initialize sensor')
            raise Exception("Sensor initialization failed")

    def update_labels(self):
        data = self.sensor.data_read_and_process()
        if self.sensor.new_data_available:
            # Convert the data to a string or the appropriate ROS 2 message format
            message = String(data[0])  # Adjust the message creation based on your data
            self.publisher_.publish(message)
            self.get_logger().info('Publishing: "%s"' % message.data)
            self.sensor.new_data_available = False

    def close_application(self):
        self.sensor.stop()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    sensor_visual = SensorVisual()
    try:
        rclpy.spin(sensor_visual)
    except KeyboardInterrupt:
        sensor_visual.get_logger().info('Keyboard interrupt, shutting down...')
        sensor_visual.close_application()

if __name__ == '__main__':
    main()