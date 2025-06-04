import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node

from .camera.astra_camera import Camera


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()
        self.camera = Camera()
        self.color_pub = self.create_publisher(Image, 'color_image', 1)
        self.depth_pub = self.create_publisher(Image, 'depth_image', 1)
        self.timer = self.create_timer(1.0 / 28.0, self.timer_callback)

    def timer_callback(self):
        try:
            color = self.camera.get_color()
            depth = self.camera.get_depth()

            color_msg = self.bridge.cv2_to_imgmsg(color, encoding='rgb8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='mono16')

            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing frames: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
