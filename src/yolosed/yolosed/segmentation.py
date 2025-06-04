import time
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

import numpy as np
from math import atan2, degrees
from .hazard import hazard_lookup

class YOLOSegnetNode(Node):
  def __init__(self):
    super().__init__('segmentation')
    self.get_logger().info('Segmentation node has been started.')
    self.bridge = CvBridge()
    
    self.declare_parameter('cam_center', 150)
    self.declare_parameter('segmentation_model', "yolo11m-seg")
    self.cam_center = self.get_parameter('cam_center').value
    self.segmentation_model = self.get_parameter('segmentation_model').value
    
    self.get_logger().info('Loading Model...')
    self.model = YOLO(f'./src/yolosed/model/{self.segmentation_model}.pt')
    self.get_logger().info(f'Model loaded on: {self.model.device}, ready to perform segmentation.')
    
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
    self.segmentation_publisher = self.create_publisher(String, '/segnet', 10)
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
    self.timer = self.create_timer(0.4, self.display_images)
    
  def odom_callback(self, msg: Odometry):
      # Mengambil quaternion orientasi dari pesan Odometry
      q = msg.pose.pose.orientation
      # Menghitung yaw dari quaternion:
      # yaw = arctan2(2*(w*z + x*y), 1 - 2*(y² + z²))
      yaw_rad = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
      yaw_deg = degrees(yaw_rad)
      # Normalisasi sehingga berada pada rentang 0-359 derajat:
      self.robot_yaw = int(yaw_deg % 360)

  def image_callback(self, msg, index):
    # self.get_logger().info(f'Received image data, performing segmentation...')
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    self.images[index] = cv_image

  def collect_results(self, results, cv_image, index):
    segmentation_data = []
    for result in results:
      for box in result.boxes:
        class_id = box.cls.item()
        label_name = self.model.names[int(class_id)]
        hazard_score = hazard_lookup.get(label_name, 4)
        
        conf = box.cls.item()
        xyxy = box.xyxy[0].tolist()
        x1, y1, x2, y2 = xyxy
        segmentation_data.append({
          'label': hazard_score,
          'conf': conf,
          'x1': int(x1),
          'x2': int(x2),
        })

    width = cv_image.shape[1]
    pixels_per_degree = width//60
    
    for data in segmentation_data:
      for degree in range(index*60, (index+1)*60):
        if data['x1'] <= (degree-(60*index))*pixels_per_degree < data['x2']:
          if (self.deg360[degree]['conf'] == 0 or self.deg360[degree]['conf'] < data['conf']):
            self.deg360[degree] = {
              'label': data['label'],
              'conf': data['conf'],
              'x1': data['x1'],
              'x2': data['x2']
            }

  def segment_image(self, image, index):
    """Gunakan segNet untuk segmentasi gambar."""
    results = self.model(image, verbose=False)
    segmented_image = results[0].plot()
    self.collect_results(results, image, index)
    return segmented_image

  def display_images(self):
    """Gabungkan dan tampilkan gambar dari semua kamera dengan hasil segmentasi."""
    if all(image is not None for image in self.images):
      try:
        self.deg360 = [{'label': None, 'conf': 0}] * 360
        self.timestamp = time.time()

        collected_images = [self.segment_image(image, idx) for idx, image in enumerate(self.images)]
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
          self.deg360[i]['label']
          for i in range(360)
        ]
        reversed_detected = detected[::-1]
        translate_detected = [
          (reversed_detected[((self.cam_center - self.robot_yaw) + i) % 360])
          if reversed_detected[((self.cam_center - self.robot_yaw) + i) % 360] is not None else -1
          for i in range(360)
        ]
        payload = {
          'timestamp': self.timestamp,
          'detected': translate_detected
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self.segmentation_publisher.publish(msg_out)
        self.get_logger().info("Success processing inference!")

      except Exception as e:
        self.get_logger().error(f"Error: {e}")
    else:
      self.get_logger().warning("Not all camera feeds are available.")
    self.images = [None] * 6
    

def main(args=None):
  rclpy.init(args=args)
  node = YOLOSegnetNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()