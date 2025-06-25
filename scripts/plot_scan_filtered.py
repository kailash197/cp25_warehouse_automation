#! /usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LaserScanPlotter(Node):
    def __init__(self):
        super().__init__('laser_scan_plotter')

        # Define QoS Profile to match /scan publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # Subscribe to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.scan_callback,
            qos_profile
        )

        self.values = []  # Store intensities or ranges
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], lw=2)
        self.ax.set_title('LaserScan Data')
        self.ax.set_xlabel('Index')
        self.ax.set_ylabel('Value (Intensity or Range)')
        self.ax.grid(True)
        self.ax.set_ylim(0, 9000)  # Adjust for high values
        self.ax.set_xlim(0, 360)  # Default x-axis limit (adjust as needed)

    def scan_callback(self, msg):
        # Use intensities if available, otherwise fallback to ranges
        self.values = list(msg.intensities) if msg.intensities else list(msg.ranges)

    def update_plot(self):
        if self.values:
            indices = range(len(self.values))
            self.line.set_data(indices, self.values)
            self.ax.set_xlim(0, len(self.values))  # Dynamically adjust x-axis
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanPlotter()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            node.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
