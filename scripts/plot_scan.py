#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LaserScanPlotter(Node):
    def __init__(self):
        super().__init__('laser_scan_plotter')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.raw_values = []
        self.filtered_values = []

        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        self.create_subscription(LaserScan, '/filtered_scan', self.filtered_callback, qos)

        # Setup plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.raw_line, = self.ax.plot([], [], 'b-', label='Raw /scan')
        self.filtered_line, = self.ax.plot([], [], 'r-', label='Filtered /filtered_scan')
        self.ax.set_title('LaserScan Comparison')
        self.ax.set_xlabel('Index')
        self.ax.set_ylabel('Range (m)')
        self.ax.set_ylim(0, 10)
        self.ax.set_xlim(0, 360)
        self.ax.legend()
        self.ax.grid(True)

    def scan_callback(self, msg):
        self.raw_values = list(msg.ranges)

    def filtered_callback(self, msg):
        self.filtered_values = list(msg.ranges)

    def update_plot(self):
        updated = False
        if self.raw_values:
            self.raw_line.set_data(range(len(self.raw_values)), self.raw_values)
            updated = True
        if self.filtered_values:
            self.filtered_line.set_data(range(len(self.filtered_values)), self.filtered_values)
            updated = True
        if updated:
            max_len = max(len(self.raw_values), len(self.filtered_values), 360)
            self.ax.set_xlim(0, max_len)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanPlotter()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
