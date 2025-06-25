import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion
import numpy as np

class YawDialPlotter(Node):
    def __init__(self):
        super().__init__('yaw_dial_plotter')

        # Define QoS Profile to match /odom publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        self.current_yaw_deg = 0  # Store the latest yaw in degrees

        # Initialize polar plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.hand, = self.ax.plot([], [], lw=3, label='Yaw')
        self.ax.set_title('Current Yaw', va='bottom')
        self.ax.set_theta_direction(1)  # Set counter-clockwise direction
        self.ax.set_theta_offset(np.pi / 2)  # Set 0 degrees at the top (north)
        self.ax.set_rticks([])  # Remove radial ticks
        self.ax.grid(True)

    def odom_callback(self, msg):
        # Extract yaw from the quaternion in the Odometry message
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        # Convert yaw to degrees
        self.current_yaw_deg = np.degrees(yaw)

    def update_plot(self):
        # Convert yaw degrees to radians for plotting
        yaw_radians = np.radians(self.current_yaw_deg)

        # Update the "hand" on the polar plot
        self.hand.set_data([yaw_radians, yaw_radians], [0, 1])
        self.ax.text(0, 1.2, f"{self.current_yaw_deg:.2f}°", ha='center', fontsize=12, color='blue')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = YawDialPlotter()
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
