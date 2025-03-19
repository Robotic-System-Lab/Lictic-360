from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import jetson_inference
import jetson_utils
import rclpy
import cv2

class MultiCameraObjectDetection(Node):
    def __init__(self):
        super().__init__('jetson_segnet')
        self.bridge = CvBridge()
        self.subscribers = []
        self.images = [None] * 6

        # Inisialisasi segNet untuk segmentasi
        self.net = jetson_inference.segNet("fcn-resnet18-voc")

        # Set the alpha blending value
        self.net.SetOverlayAlpha(150.0)

        # Subscribe ke setiap topik kamera
        for i in range(6):
            topic_name = f'/camera_{i + 1}/image_raw'
            self.subscribers.append(
                self.create_subscription(
                    Image,
                    topic_name,
                    lambda msg, idx=i: self.image_callback(msg, idx),
                    10
                )
            )
        self.timer = self.create_timer(0.1, self.display_images)

    def crop_center_width(self, image, target_width):
        """Memotong gambar pada bagian tengah dengan width yang ditentukan."""
        height, width, _ = image.shape
        if target_width >= width:
            return image  # Tidak ada crop jika target_width lebih besar atau sama dengan width asli
        x_start = (width - target_width) // 2
        return image[:, x_start:x_start + target_width]

    def image_callback(self, msg, index):
        """Callback untuk menerima data gambar dari kamera."""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Terapkan crop hanya pada width (default height 240, width dikurangi ke 230)
            cropped_image = self.crop_center_width(image, 230)

            self.images[index] = cropped_image
        except Exception as e:
            self.get_logger().error(f"Error processing image from camera {index + 1}: {e}")

    def segment_image(self, image):
        """Gunakan segNet untuk segmentasi gambar dan log hasil deteksi dengan bounding box persegi."""
        # Konversi dari OpenCV ke CUDA Image
        cuda_image = jetson_utils.cudaFromNumpy(image)
        self.net.Process(cuda_image)
        self.net.Overlay(cuda_image)

        # Konversi kembali dari CUDA Image ke OpenCV
        segmented_image = jetson_utils.cudaToNumpy(cuda_image)
        
        # Deteksi objek berdasarkan kontur pada gambar overlay
        import cv2
        import numpy as np
        # Asumsi: Latar belakang adalah hitam (nilai 0) sehingga kita threshold pixel > 0
        gray = cv2.cvtColor(segmented_image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            side = max(w, h)
            x2 = x + side
            y2 = y + side
            self.get_logger().info(
                f"Detected object at [{x}, {y}, {x2}, {y2}]"
            )
        return segmented_image

    def display_images(self):
        """Gabungkan dan tampilkan gambar dari semua kamera dengan hasil segmentasi."""
        if all(image is not None for image in self.images):
            try:
                # Proses setiap gambar untuk segmentasi
                processed_images = [self.segment_image(image) for image in self.images]

                # Gabungkan semua gambar dalam satu baris (6x1)
                combined_image = cv2.hconcat(processed_images)

                # Tampilkan hasil segmentasi
                cv2.imshow('Multi Camera Display (6x1)', combined_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Error during image concatenation: {e}")
        else:
            self.get_logger().warning("Not all camera feeds are available.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()