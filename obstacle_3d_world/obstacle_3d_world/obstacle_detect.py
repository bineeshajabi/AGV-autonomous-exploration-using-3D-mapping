import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DynamicFilter(Node):
    def __init__(self):
        super().__init__('dynamic_filter')

        self.bridge = CvBridge()
        self.prev_gray = None

        self.sub_rgb = self.create_subscription(
            Image, '/camera/image', self.rgb_callback, 10)

        self.sub_depth = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, 10)

        self.pub_rgb = self.create_publisher(Image, '/filtered/image', 10)
        self.pub_depth = self.create_publisher(Image, '/filtered/depth', 10)

        self.current_depth = None

    def depth_callback(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            return

        # Frame difference
        diff = cv2.absdiff(self.prev_gray, gray)
        _, mask = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

        # Remove moving regions from depth
        if self.current_depth is not None:
            filtered_depth = self.current_depth.copy()
            filtered_depth[mask > 0] = 0  # remove moving parts

            depth_msg = self.bridge.cv2_to_imgmsg(filtered_depth, encoding='passthrough')
            self.pub_depth.publish(depth_msg)

        rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_rgb.publish(rgb_msg)

        self.prev_gray = gray


def main(args=None):
    rclpy.init(args=args)
    node = DynamicFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()