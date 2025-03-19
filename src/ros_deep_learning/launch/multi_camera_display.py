import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiCameraDisplay(Node):
    def __init__(self):
        super().__init__('multi_camera_display')
        self.bridge = CvBridge()
        self.images = [None] * 6
        self.subscribers = [
            self.create_subscription(Image, f'/camera_{i}/raw', lambda msg, i=i: self.image_callback(msg, i), 10)
            for i in range(1, 7)
        ]

    def image_callback(self, msg, index):
        self.images[index - 1] = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.display_images()

    def display_images(self):
        if None not in self.images:
            top_row = np.hstack(self.images[:3])
            bottom_row = np.hstack(self.images[3:])
            full_image = np.vstack([top_row, bottom_row])
            cv2.imshow("Multi Camera Display", full_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
