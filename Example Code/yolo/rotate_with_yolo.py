import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class TurtleBot3YOLO(Node):
    def __init__(self):
        super().__init__('turtlebot3_yolo')
        self.bridge = CvBridge()

        # Load YOLO model
        self.model = YOLO('/home/ubuntu/joshhuang/yolo/best_ncnn_model')

        # Set target class ID to 0
        self.target_class_id = 0

        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.latest_image = None

        # Initialize rotation state and timer
        self.is_rotating = False
        self.rotation_timer = None

        self.get_logger().info("TurtleBot3 YOLO node initialized.")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_image()
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image to OpenCV format: {str(e)}")

    def process_image(self):
        if self.latest_image is None or self.is_rotating:
            return

        results = self.model(self.latest_image)

        for result in results[0].boxes:
            class_id = int(result.cls[0])
            if class_id == self.target_class_id:
                x1, y1, x2, y2 = map(int, result.xyxy[0])
                bbox_center_x = (x1 + x2) // 2

                image_center_x = self.latest_image.shape[1] // 2
                offset = bbox_center_x - image_center_x

                self.adjust_turtlebot_rotation(offset)
                break

    def adjust_turtlebot_rotation(self, offset):
        threshold = 50  # Threshold for deciding if the target is centered
        rotation_speed = 0.2

        if abs(offset) <= threshold:
            # Stop rotation if the target is centered
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_publisher.publish(twist)
            self.is_rotating = False  # Ensure the rotation state is reset
            self.get_logger().info("Target centered. Finished rotation.")
            return

        if not self.is_rotating:
            # Rotate towards the target
            twist = Twist()
            twist.angular.z = -rotation_speed if offset < 0 else rotation_speed
            self.cmd_vel_publisher.publish(twist)
            self.is_rotating = True
            self.get_logger().info(f"Rotating TurtleBot3. Offset: {offset}")

            # Start timer to stop rotation after 0.5 seconds
            self.rotation_timer = self.create_timer(0.5, self.stop_rotation)

    def stop_rotation(self):
        # Stop the motor and reset the state
        twist = Twist()
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        # Stop the timer and reset rotation state
        if self.rotation_timer:
            self.rotation_timer.cancel()
            self.rotation_timer = None

        self.is_rotating = False
        self.get_logger().info("Stopped rotating after 0.5 seconds.")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3YOLO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()