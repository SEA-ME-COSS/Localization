import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_pub_node')
        self.publisher_ = self.create_publisher(Image, 'camera_data', 10)
        self.timer_ = self.create_timer(0.05, self.callback)  # [s]

        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)

        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)

        self.bridge = CvBridge()
        self.msg = Image()


    def callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        self.msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")

        self.publisher_.publish(self.msg)
        # self.get_logger().info('')


    def __del__(self):
        self.pipeline.stop()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
