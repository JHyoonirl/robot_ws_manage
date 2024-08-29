import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class ForceTorqueGraph(Node):
    def __init__(self):
        super().__init__('force_torque_graph')
        self.subscription_force = self.create_subscription(
            Vector3,
            'force_data',
            self.listen_force,
            10)
        self.subscription_torque = self.create_subscription(
            Vector3,
            'torque',
            self.listen_torque,
            10)
        self.force_data = []
        self.torque_data = []
        self.fig, self.ax = plt.subplots(2, 1)
        self.ani = animation.FuncAnimation(self.fig, self.update_graph, interval=1000)
        plt.show()

    def listen_force(self, msg):
        self.force_data.append((self.get_clock().now().nanoseconds, msg))

    def listen_torque(self, msg):
        self.torque_data.append((self.get_clock().now().nanoseconds, msg))

    def update_graph(self, frame):
        if self.force_data:
            times, forces = zip(*[(t, (data.x, data.y, data.z)) for t, data in self.force_data])
            self.ax[0].clear()
            self.ax[0].plot(times, [f[0] for f in forces], label='X')
            self.ax[0].plot(times, [f[1] for f in forces], label='Y')
            self.ax[0].plot(times, [f[2] for f in forces], label='Z')
            self.ax[0].legend()
            self.ax[0].set_title('Force Data')

        if self.torque_data:
            times, torques = zip(*[(t, (data.x, data.y, data.z)) for t, data in self.torque_data])
            self.ax[1].clear()
            self.ax[1].plot(times, [t[0] for t in torques], label='X')
            self.ax[1].plot(times, [t[1] for t in torques], label='Y')
            self.ax[1].plot(times, [t[2] for t in torques], label='Z')
            self.ax[1].legend()
            self.ax[1].set_title('Torque Data')

def main(args=None):
    rclpy.init(args=args)
    force_torque_graph = ForceTorqueGraph()
    rclpy.spin(force_torque_graph)
    force_torque_graph.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
