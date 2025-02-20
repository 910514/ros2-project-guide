import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import matplotlib.pyplot as plt
import cv2
import io
from nav_msgs.msg import Odometry

class LiDARClusterVisualizerNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_visualizer')

        # LiDAR data buffer
        self.lidar_data_list = []

        # TurtleBot position
        self.turtlebot_position = (0, 0)  # Initial position (x, y)

        # QoS settings
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscriber to /scan topic for LiDAR data
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile)

        # Subscriber to /odom topic for TurtleBot position
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile)

        # Publisher for visualization as an image
        self.image_publisher = self.create_publisher(Image, '/lidar_clusters', 10)

        # Timer to periodically update visualization
        self.create_timer(1.0, self.update_visualization)  # Update every second

        # CVBridge for converting images to ROS2 format
        self.bridge = CvBridge()

    def lidar_callback(self, msg):
        self.lidar_data_list.append(np.array(msg.ranges))

    def odom_callback(self, msg):
        # Get the position of TurtleBot from /odom topic
        self.turtlebot_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update_visualization(self):
        if not self.lidar_data_list:
            return

        # Get the latest LiDAR data
        lidar_data = self.lidar_data_list[-1]

        # Handle NaN values by filtering them out
        valid_mask = ~np.isnan(lidar_data)
        lidar_data = lidar_data[valid_mask]

        # Generate angles for valid data (shift by 90 degrees to align north as front)
        angles = np.linspace(0, 360, len(self.lidar_data_list[-1]))[valid_mask] - 90
        angles_rad = np.deg2rad(angles)

        # Convert polar coordinates to Cartesian for DBSCAN clustering
        x = lidar_data * np.cos(angles_rad)
        y = lidar_data * np.sin(angles_rad)
        points = np.column_stack((x, y))

        # Apply DBSCAN to cluster the points
        dbscan = DBSCAN(eps=0.3, min_samples=5)  # Adjust parameters as needed
        labels = dbscan.fit_predict(points)

        # Create a Cartesian plot (instead of polar plot)
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title('LiDAR Clustering with TurtleBot Position')

        # Assign colors to each cluster
        unique_labels = set(labels)
        colors = plt.cm.get_cmap('tab20', len(unique_labels))

        for label in unique_labels:
            # Filter points for this cluster
            cluster_mask = labels == label
            cluster_x = x[cluster_mask]
            cluster_y = y[cluster_mask]

            # Assign color (black for noise, different colors for clusters)
            color = 'k' if label == -1 else colors(label)
            ax.scatter(cluster_x, cluster_y, label=f'Cluster {label}', color=color, s=10)

        # Plot TurtleBot's position on the map
        turtlebot_x, turtlebot_y = self.turtlebot_position
        ax.scatter(turtlebot_x, turtlebot_y, color='r', label='TurtleBot Position', marker='x', s=100)

        ax.legend()

        # Save plot to an in-memory buffer
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        plt.close(fig)

        # Convert the plot to a numpy array for OpenCV
        buf_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        img = cv2.imdecode(buf_array, cv2.IMREAD_COLOR)

        # Publish the image as a ROS2 topic
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.image_publisher.publish(image_msg)
        self.get_logger().info('Published clustered LiDAR visualization with TurtleBot position.')

def main(args=None):
    rclpy.init(args=args)
    node = LiDARClusterVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
