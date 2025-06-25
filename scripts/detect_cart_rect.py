import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from math import cos, sin, atan2, hypot, pi

class CartDetector(Node):
    def __init__(self):
        super().__init__('cart_detector')
        
        # Subscriber to raw laser scan
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Publisher for modified scan
        self.pub_filtered = self.create_publisher(
            LaserScan,
            '/scan_filtered',
            10)
        
        # Cart dimensions (75mm x 44mm)
        self.cart_length = 0.075  # meters
        self.cart_width = 0.044   # meters
        
        self.get_logger().info("Cart detector node started")

    def scan_callback(self, msg):
        # Convert laser scan to Cartesian coordinates
        angles = np.arange(msg.angle_min, msg.angle_max + msg.angle_increment, 
                          msg.angle_increment)
        ranges = np.array(msg.ranges)
        
        # Filter out infinite values
        valid = np.isfinite(ranges)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        
        # Cluster points (simple distance-based clustering)
        clusters = self.dbscan(np.column_stack((x, y)), eps=0.05, min_samples=3)
        
        # Find clusters that could represent cart legs (looking for 6 clusters)
        if len(clusters) >= 6:
            # Find the 6 largest clusters
            cluster_sizes = [len(c) for c in clusters]
            largest_indices = np.argsort(cluster_sizes)[-6:]
            cart_clusters = [clusters[i] for i in largest_indices]
            
            # Get centroids of these clusters
            centroids = [np.mean(cluster, axis=0) for cluster in cart_clusters]
            
            # Find the rectangle formed by these points
            rect_corners = self.find_rectangle(centroids)
            
            if rect_corners is not None:
                # Create a new scan with the cart area blocked
                filtered_scan = self.block_cart_area(msg, rect_corners)
                self.pub_filtered.publish(filtered_scan)

    def dbscan(self, points, eps, min_samples):
        """Simple DBSCAN implementation for clustering"""
        from sklearn.neighbors import NearestNeighbors
        nbrs = NearestNeighbors(radius=eps).fit(points)
        distances, indices = nbrs.radius_neighbors(points)
        
        labels = np.zeros(len(points)) - 1  # -1 = noise
        cluster_id = 0
        
        for i in range(len(points)):
            if labels[i] != -1:
                continue
                
            if len(indices[i]) >= min_samples:
                # Start new cluster
                labels[i] = cluster_id
                seeds = set(indices[i])
                seeds.discard(i)
                
                while seeds:
                    j = seeds.pop()
                    if labels[j] == -1:
                        labels[j] = cluster_id
                        if len(indices[j]) >= min_samples:
                            seeds.update(indices[j])
                
                cluster_id += 1
                
        # Convert labels to clusters
        clusters = []
        for i in range(cluster_id):
            clusters.append(points[labels == i])
            
        return clusters

    def find_rectangle(self, points):
        """Find rectangle corners from 6 points"""
        # Simple approach: find 4 corners and 2 midpoints
        # This could be enhanced with proper rectangle fitting
        if len(points) != 6:
            return None
            
        # Sort points by x coordinate
        sorted_points = sorted(points, key=lambda p: p[0])
        
        # Assume first 2 are left side, last 2 are right side
        left = sorted_points[:2]
        right = sorted_points[-2:]
        
        # Find top and bottom points
        left_sorted = sorted(left, key=lambda p: p[1])
        right_sorted = sorted(right, key=lambda p: p[1])
        
        # Estimate rectangle corners
        corners = [
            left_sorted[0],    # Bottom-left
            right_sorted[0],    # Bottom-right
            right_sorted[1],    # Top-right
            left_sorted[1]      # Top-left
        ]
        
        return corners

    def block_cart_area(self, scan, corners):
        """Modify scan to block the cart area"""
        filtered = LaserScan()
        filtered.header = scan.header
        filtered.angle_min = scan.angle_min
        filtered.angle_max = scan.angle_max
        filtered.angle_increment = scan.angle_increment
        filtered.time_increment = scan.time_increment
        filtered.scan_time = scan.scan_time
        filtered.range_min = scan.range_min
        filtered.range_max = scan.range_max
        filtered.ranges = list(scan.ranges)  # Make a copy
        
        # Convert corners to polar coordinates
        angles = np.arange(scan.angle_min, scan.angle_max + scan.angle_increment,
                         scan.angle_increment)
        
        # For each angle, check if it intersects the rectangle
        for i, angle in enumerate(angles):
            if i >= len(filtered.ranges):
                continue
                
            r = scan.ranges[i]
            if not np.isfinite(r):
                continue
                
            x = r * cos(angle)
            y = r * sin(angle)
            
            # Check if point is inside rectangle
            if self.point_in_rectangle((x, y), corners):
                # Set range to very small value (block)
                filtered.ranges[i] = 0.01
                
        return filtered

    def point_in_rectangle(self, point, corners):
        """Check if point is inside rectangle using cross product method"""
        if len(corners) != 4:
            return False
            
        # Order corners (assuming they're ordered clockwise or counter-clockwise)
        def cross_product(a, b):
            return a[0]*b[1] - a[1]*b[0]
            
        edges = [
            (corners[1][0] - corners[0][0], corners[1][1] - corners[0][1]),
            (corners[2][0] - corners[1][0], corners[2][1] - corners[1][1]),
            (corners[3][0] - corners[2][0], corners[3][1] - corners[2][1]),
            (corners[0][0] - corners[3][0], corners[0][1] - corners[3][1])
        ]
        
        vectors = [
            (point[0] - corners[0][0], point[1] - corners[0][1]),
            (point[0] - corners[1][0], point[1] - corners[1][1]),
            (point[0] - corners[2][0], point[1] - corners[2][1]),
            (point[0] - corners[3][0], point[1] - corners[3][1])
        ]
        
        # All cross products should have same sign if point is inside
        signs = []
        for edge, vec in zip(edges, vectors):
            cp = cross_product(edge, vec)
            if abs(cp) < 1e-6:  # On edge
                return True
            signs.append(cp > 0)
            
        return all(signs) or not any(signs)

def main(args=None):
    rclpy.init(args=args)
    node = CartDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()