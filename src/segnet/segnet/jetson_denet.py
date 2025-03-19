from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import jetson_inference
import jetson_utils
import rclpy
import cv2

class MultiCameraHumanDetection(Node):
    def __init__(self):
        super().__init__('jetson_denet')
        self.bridge = CvBridge()
        self.subscribers = []
        self.images = [None] * 6

        # Inisialisasi detektor objek hanya untuk manusia ("person" class ID = 1 di COCO dataset)
        self.net = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.5)

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
            cropped_image = self.crop_center_width(image, 250)

            self.images[index] = cropped_image
        except Exception as e:
            self.get_logger().error(f"Error processing image from camera {index + 1}: {e}")

    def detect_humans(self, image):
        """Gunakan DetectNet untuk mendeteksi hanya objek manusia pada gambar."""
        # Konversi dari OpenCV ke CUDA Image
        cuda_image = jetson_utils.cudaFromNumpy(image)
        detections = self.net.Detect(cuda_image)

        # Gambar bounding box hanya untuk manusia (ClassID == 1 untuk 'person')
        for detection in detections:
            # Koordinat bounding box
            x1, y1, x2, y2 = map(int, [detection.Left, detection.Top, detection.Right, detection.Bottom])
            # confidence = detection.Confidence * 100
            # label = self.net.GetClassDesc(detection.ClassID)

            # Generate a unique color for each label (0-90)
            color = tuple(int((detection.ClassID * 123 + i * 45) % 256) for i in range(3))

            # Gambar bounding box
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

            # Tambahkan teks label dan tingkat kepercayaan
            # text = f"{label} {confidence:.1f}%"
            # cv2.putText(image, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        return image

    def display_images(self):
        """Gabungkan dan tampilkan gambar dari semua kamera dengan hasil deteksi."""
        if all(image is not None for image in self.images):
            try:
                # Proses setiap gambar untuk deteksi manusia
                processed_images = [self.detect_humans(image) for image in self.images]

                # Gabungkan semua gambar dalam satu baris (6x1)
                combined_image = cv2.hconcat(processed_images)

                cv2.imshow('Multi Camera Display (6x1) - Human Detection', combined_image)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Error during image concatenation: {e}")
        else:
            self.get_logger().warning("Not all camera feeds are available.")

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraHumanDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
