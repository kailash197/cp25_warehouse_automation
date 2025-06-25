#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from sklearn.cluster import DBSCAN
from math import cos, sin
import random

class IntensityClusterNode(Node):
    def __init__(self):
        super().__init__('intensity_cluster_node')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/cluster_markers', 10)

        self.get_logger().info("Intensity Clustering Node with RViz Marker Output started.")

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_inc = msg.angle_increment

        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities)

        angles = np.arange(angle_min, angle_max, angle_inc)
        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]

        mask = (intensities >= 3000.0) & np.isfinite(ranges)
        ranges = ranges[mask]
        angles = angles[mask]

        if len(ranges) == 0:
            self.get_logger().info("No high-intensity points found.")
            self.marker_pub.publish(MarkerArray())  # Clear old markers
            return

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        points = np.vstack((xs, ys)).T

        clustering = DBSCAN(eps=0.25, min_samples=7).fit(points)
        labels = clustering.labels_

        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)

        for cluster_id in range(num_clusters):
            cluster_points = points[labels == cluster_id]

            marker = Marker()
            marker.header.frame_id = "robot_front_laser_base_link"  # or "laser" or "map", depending on TF
            marker.header.stamp = timestamp
            marker.ns = "clusters"
            marker.id = cluster_id
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Random color per cluster
            r, g, b = [random.random() for _ in range(3)]
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            for pt in cluster_points:
                from geometry_msgs.msg import Point
                p = Point()
                p.x = float(pt[0])
                p.y = float(pt[1])
                p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        # self.get_logger().info(f"Published {num_clusters} cluster(s) to RViz.")

def main(args=None):
    rclpy.init(args=args)
    node = IntensityClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
