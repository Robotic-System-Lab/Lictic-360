import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class MultiCameraNode(Node):
    def __init__(self):
        super().__init__('multi_camera_node')

        # Bridge untuk mengkonversi ROS Image message ke OpenCV
        self.bridge = CvBridge()

        # Subscribers untuk setiap kamera
        self.subscription_1 = self.create_subscription(
            Image, '/camera_1/image_raw', self.camera_callback_1, 10)
        self.subscription_2 = self.create_subscription(
            Image, '/camera_2/image_raw', self.camera_callback_2, 10)
        self.subscription_3 = self.create_subscription(
            Image, '/camera_3/image_raw', self.camera_callback_3, 10)
        self.subscription_4 = self.create_subscription(
            Image, '/camera_4/image_raw', self.camera_callback_4, 10)
        self.subscription_5 = self.create_subscription(
            Image, '/camera_5/image_raw', self.camera_callback_5, 10)
        self.subscription_6 = self.create_subscription(
            Image, '/camera_6/image_raw', self.camera_callback_6, 10)

        # Publisher untuk gambar gabungan
        self.publisher_ = self.create_publisher(Image, 'multi_camera_image', 10)

        # Gambar kosong untuk menyimpan gambar-gambar kamera
        self.frames = [None] * 6

    def camera_callback_1(self, msg):
        self.process_frame(msg, 0)

    def camera_callback_2(self, msg):
        self.process_frame(msg, 1)

    def camera_callback_3(self, msg):
        self.process_frame(msg, 2)

    def camera_callback_4(self, msg):
        self.process_frame(msg, 3)

    def camera_callback_5(self, msg):
        self.process_frame(msg, 4)

    def camera_callback_6(self, msg):
        self.process_frame(msg, 5)

    def process_frame(self, msg, camera_index):
        # Mengkonversi ROS Image message ke OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Menyimpan frame sesuai dengan kamera yang menerima gambar
        self.frames[camera_index] = frame

        # Pastikan semua frame tersedia sebelum diproses
        if all(frame is not None for frame in self.frames):
            # Membagi gambar menjadi 2 baris dan 3 kolom
            top_row = np.hstack(self.frames[:3])  # Gabungkan frame 1-3
            bottom_row = np.hstack(self.frames[3:])  # Gabungkan frame 4-6
            combined_frame = np.vstack((top_row, bottom_row))  # Gabungkan menjadi satu frame besar

            # Mengkonversi gambar gabungan kembali ke ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(combined_frame, encoding='bgr8')

            # Publikasikan gambar gabungan
            self.publisher_.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
