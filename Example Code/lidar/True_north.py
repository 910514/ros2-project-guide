import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class DetectObstacle(Node):
    def __init__(self):
        super().__init__('detect_obstacle_node')
        qos = rclpy.qos.QoSProfile(depth=10)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.pose_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)

    def scan_callback(self, scan):
        # Desired angle range in degrees
        lower_bound_deg = 175  # Start angle in degrees
        upper_bound_deg = 185   # End angle in degrees

        # Convert bounds to radians
        lower_bound = lower_bound_deg * math.pi / 180
        upper_bound = upper_bound_deg * math.pi / 180

        angle = scan.angle_min
        total_distance = 0.0
        cnt = 0

        for i in range(len(scan.ranges)):
            if lower_bound < angle < upper_bound:  # Check if angle is within range
                if scan.ranges[i] == scan.ranges[i]:  # Exclude NaN values
                    total_distance += scan.ranges[i]
                    cnt += 1
            angle += scan.angle_increment  # Increment angle for next beam

        if cnt > 0:
            print(total_distance / cnt, "[m]")
        else:
            print("No valid readings in the specified range.")


def main(args=None):
    rclpy.init(args=args)
    node = DetectObstacle()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()