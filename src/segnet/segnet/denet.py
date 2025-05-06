import time
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import colorsys
import jetson_inference     # type: ignore # <-- Added for DetectNet
import jetson_utils         # type: ignore #  <-- Added for image conversion

import numpy as np
from math import atan2, degrees
from .hazard import hazard_lookup

class DetectionNode(Node):
  def __init__(self):
    super().__init__('denet')
    self.get_logger().info('Detection node has been started.')
    self.bridge = CvBridge()
    
    self.declare_parameter('cam_center', 150)
    self.declare_parameter('detection_model', "ssd-mobilenet-v2")
    self.cam_center = self.get_parameter('cam_center').value
    self.detection_model = self.get_parameter('detection_model').value
    self.detection_threshold = self.get_parameter('detection_threshold').value
    
    self.get_logger().info('Loading DetectNet Model...')
    self.model = jetson_inference.detectNet(self.detection_model, threshold=self.detection_threshold)
    self.get_logger().info('DetectNet model loaded, ready to perform detection.')
    
    self.label_colors = self.create_color_map(91)
    self.timestamp = 0
    self.images = [None] * 6
    self.deg360 = [{
                'label': None,
                'conf': 0,
              }] * 360
    
    self.robot_yaw = 0
    self.subscription = self.create_subscription(
        Odometry,
        '/odom',
        self.odom_callback,
        10)
    
    self.subscribers = []
    self.detection_publisher = self.create_publisher(String, '/segnet', 10)
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
    self.timer = self.create_timer(0.5, self.display_images)
  
  
  def odom_callback(self, msg: Odometry):
    # Mengambil quaternion orientasi dari pesan Odometry
    q = msg.pose.pose.orientation
    # Menghitung yaw dari quaternion:
    # yaw = arctan2(2*(w*z + x*y), 1 - 2*(y² + z²))
    yaw_rad = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    yaw_deg = degrees(yaw_rad)
    # Normalisasi sehingga berada pada rentang 0-359 derajat:
    self.robot_yaw = int(yaw_deg % 360)

  # --- New: Helper function to create a color map ---
  def create_color_map(self, n):
    colors = {}
    for i in range(n):
      hue = i / n
      r, g, b = colorsys.hsv_to_rgb(hue, 1, 1)
      # Convert to BGR integer values for OpenCV
      colors[i] = (int(b * 255), int(g * 255), int(r * 255))
    return colors
  # --- End New ---

  def image_callback(self, msg, index):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    self.get_logger().debug(f"Received image from camera_{index}")
    self.images[index] = cv_image

  def collect_results(self, results, cv_image, index):
    detection_data = []
    # For each detection returned by DetectNet
    for detection in results:
      class_id = detection.ClassID
      label_name = self.model.GetClassDesc(class_id)
      hazard_score = hazard_lookup.get(label_name, 1)
      
      conf = detection.Confidence         # Retrieve detection confidence
      x1 = int(detection.Left)              # Left coordinate (x1)
      x2 = int(detection.Right)             # Right coordinate (x2)
      detection_data.append({
          'label': hazard_score,
          'conf': conf,
          'x1': x1,
          'x2': x2,
      })

    width = cv_image.shape[1]*6
    pixels_per_degree = width // 360

    for data in detection_data:
      for degree in range(index*60, (index+1)*60):
        if data['x1'] <= (degree-(60*index))*pixels_per_degree < data['x2']:
          if (self.deg360[degree]['conf'] == 0 or self.deg360[degree]['conf'] < data['conf']):
            self.deg360[degree] = {
              'label': data['label'],
              'conf': data['conf'],
              'x1': data['x1'],
              'x2': data['x2']
            }

  def detect_image(self, image, index):
    """Gunakan DetectNet untuk image detection"""
    cuda_image_det = jetson_utils.cudaFromNumpy(image)
    results = self.model.Detect(cuda_image_det)

    # Gambar bounding boxes pada gambar segmented_image
    for detection in results:
      x1 = int(detection.Left)
      y1 = int(detection.Top)
      x2 = int(detection.Right)
      y2 = int(detection.Bottom)
      label_id = int(detection.ClassID)
      color = self.label_colors.get(label_id, (0, 255, 0))
      cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

    self.collect_results(results, image, index)
    return image

  def display_images(self):
    """Gabungkan dan tampilkan gambar dari semua kamera dengan hasil segmentasi."""
    if all(image is not None for image in self.images):
      try:
        self.deg360 = [{'label': None, 'conf': 0}] * 360
        self.timestamp = time.time()
        
        collected_images = [self.detect_image(image, idx) for idx, image in enumerate(self.images)]
        resized_images = []
        target_height = 160
        for image in collected_images:
            h, w = image.shape[:2]
            scale = target_height / h if h > target_height else 1
            new_w = int(w * scale)
            resized_image = cv2.resize(image, (new_w, target_height))
            resized_images.append(resized_image)
        
        border_thickness = 5
        bordered_images = []
        for idx, img in enumerate(resized_images):
            bordered_images.append(img)
            if idx < len(resized_images) - 1:
                # Membuat kolom border dengan warna hitam (0, 0, 0)
                border = np.full((target_height, border_thickness, 3), 0, dtype=np.uint8)
                bordered_images.append(border)
                
        # Gabungkan semua gambar dan border secara horizontal
        combined_image = cv2.hconcat(bordered_images)
        cv2.imshow("Segmented Images", combined_image)
        cv2.waitKey(1)

        detected = [
          (self.deg360[(self.cam_center + int(self.robot_yaw) + i) % 360]['label'] + 1)
          if self.deg360[(self.cam_center + int(self.robot_yaw) + i) % 360]['label'] is not None else -1
          for i in range(360)
        ]
        payload = {
          'timestamp': self.timestamp,
          'detected': detected
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self.segmentation_publisher.publish(msg_out)
        
      except Exception as e:
        self.get_logger().error(f"Error: {e}")
      self.images = [None] * 6
    else:
      # available_cameras = sum(1 for image in self.images if image is not None)
      # self.get_logger().error(f"Available camera feeds: {available_cameras}/6")
      self.get_logger().warn(f"Waiting..")

    

def main(args=None):
  rclpy.init(args=args)
  node = DetectionNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()