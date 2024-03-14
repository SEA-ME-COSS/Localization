import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import  cv2
from cv_bridge import CvBridge
import torch


class Detection(Node):
    def __init__(self):
        super().__init__('detection_node')
        self.subscription_ = self.create_subscription(Image, 'camera_data', self.callback, 10)
        self.subscription_  # prevent unused variable warning

        self.bridge = CvBridge()


    def callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        model_path = "../model/model.pt"
        model = torch.hub.load("./yolov5", "custom", path=model_path, source='local')

        image = cv2.resize(image, (640, 640))
        image = image.astype('float32') / 255.0
        img_tensor = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0)
            
        with torch.no_grad():
            output = model(img_tensor)

            print(output)

            # for i in range(output.size(1)):
            #     detection = output[0, i]

                # x_center = detection[0].item()
                # y_center = detection[1].item()
                # width = detection[2].item()
                # height = detection[3].item()
                # conf = detection[4].item()

                # light = detection[5].item()
                # sign = detection[6].item()

        cv2.imshow('Image', image)
        cv2.waitKey(1)

        # self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)

    detection = Detection()

    rclpy.spin(detection)

    detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
