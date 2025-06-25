import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LaserScanPlotter(Node):
    def __init__(self):
        super().__init__('laser_scan_plotter')

        # QoS Profile setup
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # LaserScan subscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

        # Data storage
        self.ranges = []
        self.intensities = []
        self.angles = []
        
        # Plot setup
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        self.fig.subplots_adjust(hspace=0.5)
        
        # Initialize bars with empty data
        self.range_bars = self.ax1.bar([], [], width=0.8)
        self.ax1.set_title('LaserScan Range Values')
        self.ax1.set_xlabel('Measurement Index')
        self.ax1.set_ylabel('Range (m)')
        self.ax1.grid(True)
        self.ax1.set_ylim(0, 10)  # Default range
        
        self.intensity_bars = self.ax2.bar([], [], width=0.8, color='orange')
        self.ax2.set_title('LaserScan Intensity Values')
        self.ax2.set_xlabel('Measurement Index')
        self.ax2.set_ylabel('Intensity')
        self.ax2.grid(True)
        self.ax2.set_ylim(0, 9000)  # Default intensity

    def scan_callback(self, msg):
        # Convert to Python list and filter out infinite values
        self.ranges = [r if not (np.isinf(r) or np.isnan(r)) else 0 for r in msg.ranges]
        self.intensities = [i if not (np.isinf(i) or np.isnan(i)) else 0 
                          for i in (msg.intensities if msg.intensities else [0]*len(msg.ranges))]
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def update_plot(self):
        if self.ranges:
            x = range(len(self.ranges))
            
            # Update range bars
            self.ax1.clear()
            self.range_bars = self.ax1.bar(x, self.ranges, width=0.8)
            self.ax1.set_title('LaserScan Range Values')
            self.ax1.set_xlabel('Measurement Index')
            self.ax1.set_ylabel('Range (m)')
            self.ax1.grid(True)
            
            # Calculate safe y-limits for ranges
            valid_ranges = [r for r in self.ranges if not (np.isinf(r) or np.isnan(r))]
            y_max = max(valid_ranges)*1.1 if valid_ranges else 10
            self.ax1.set_ylim(0, y_max)
            
            # Update intensity bars
            self.ax2.clear()
            self.intensity_bars = self.ax2.bar(x, self.intensities, width=0.8, color='orange')
            self.ax2.set_title('LaserScan Intensity Values')
            self.ax2.set_xlabel('Measurement Index')
            self.ax2.set_ylabel('Intensity')
            self.ax2.grid(True)
            
            # Calculate safe y-limits for intensities
            valid_intensities = [i for i in self.intensities if not (np.isinf(i) or np.isnan(i))]
            y_max = max(valid_intensities)*1.1 if valid_intensities else 9000
            self.ax2.set_ylim(0, y_max)
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanPlotter()
    try:
        plt.ion()  # Interactive mode
        plt.show()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_plot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()

if __name__ == '__main__':
    main()