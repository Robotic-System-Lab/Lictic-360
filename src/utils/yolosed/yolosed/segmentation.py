import time
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os

import numpy as np
from math import atan2, degrees
from .hazard import hazard_lookup
from sensor_msgs.msg import LaserScan

class YOLOSegnetNode(Node):
  def __init__(self):
    super().__init__('segmentation')
    self.get_logger().info('Segmentation node has been started.')
    self.bridge = CvBridge()
    
    self.declare_parameter('view_p', 0.5)
    self.declare_parameter('view_h', 0.2)
    self.view_p = self.get_parameter('view_p').value
    self.view_h = self.get_parameter('view_h').value
    
    self.declare_parameter('cam_center', 150)
    self.cam_center = self.get_parameter('cam_center').value
    
    self.declare_parameter('segmentation_model', "yolo11m-seg")
    self.segmentation_model = self.get_parameter('segmentation_model').value
    
    self.get_logger().info('Loading Model...')
    model_path = os.path.join(os.path.dirname(__file__), 'model', f"{self.segmentation_model}")
    self.model = YOLO(model_path)
    self.get_logger().info(f'Model loaded on: {self.model.device}, ready to perform segmentation.')
    
    self.timestamp = 0
    self.images = [None] * 6
    self.deg360 = [{
                'label': None,
                'conf': 0,
              }] * 360
    
    self.subscribers = []
    self.label_start_publisher = self.create_publisher(Float64, '/label_start', 10)
    self.label_end_publisher = self.create_publisher(String, '/label_end', 10)
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
  
  def image_callback(self, msg, index):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    self.images[index] = cv_image

  def collect_results(self, results, cv_image, index):
    label_data = []
    segmentation_data = []
    height = cv_image.shape[0]
    y_min_valid = int(self.view_p * height)
    y_max_valid = int(self.view_h * height + y_min_valid)
    
    for result in results:
      if result.boxes is not None:
        for box in result.boxes:
          xyxy = box.xyxy[0].tolist()
          x1, _, x2, _ = xyxy
          class_id = box.cls.item()
          label_name = self.model.names[int(class_id)]
          conf = box.conf.item()

          label_data.append({
            'label': hazard_lookup.get(label_name, 1),
            'conf': conf,
            'name': label_name,
          })
      if result.masks is not None:
        idx = 0
        for mask in result.masks.xy:
          valid_points = mask[(mask[:, 1] >= y_min_valid) & (mask[:, 1] <= y_max_valid)]
          if valid_points.size > 0:
            x1 = int(valid_points[:, 0].min())
            x2 = int(valid_points[:, 0].max())
            segmentation_data.append({
              'x1': x1,
              'x2': x2,
              'conf': label_data[idx]['conf'],
              'label': label_data[idx]['label'],
              'name': label_data[idx]['name'],
            })
          else:
            segmentation_data.append({'name': f"INVALID-{label_data[idx]['name']}"})
          idx += 1

    width = cv_image.shape[1]
    pixels_per_degree = width//60
    
    for data in segmentation_data:
      # self.get_logger().info(f"Collected data: {data['name']}")
      if 'label' not in data:
        continue
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
        # self.get_logger().info("Performing segmentation on collected images...")
        self.timestamp = time.time()
        label_start_msg = Float64()
        label_start_msg.data = self.timestamp
        self.label_start_publisher.publish(label_start_msg)
        
        self.deg360 = [{'label': None, 'conf': 0}] * 360
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

        # Tambahkan overlay hitam semi transparan (opacity 50%)
        overlay = combined_image.copy()
        h, w = combined_image.shape[:2]
        alpha = 0.5
        top_end = int(self.view_p * h)
        bottom_start = int((self.view_h + self.view_p) * h)
        cv2.rectangle(overlay, (0, 0), (w, top_end), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, bottom_start), (w, h), (0, 0, 0), -1)

        # Gabungkan overlay dengan combined_image
        combined_image = cv2.addWeighted(overlay, alpha, combined_image, 1 - alpha, 0)

        cv2.imshow("Segmented Images", combined_image)
        cv2.waitKey(1)

        detected = [
          self.deg360[i]['label']
          for i in range(360)
        ]
        reversed_detected = detected[::-1]
        translate_detected = [
          (reversed_detected[((self.cam_center) + i) % 360])
          if reversed_detected[((self.cam_center) + i) % 360] is not None else 99
          for i in range(360)
        ]
        payload = {
          'timestamp': self.timestamp,
          'detected': translate_detected
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self.label_end_publisher.publish(msg_out)
        self.get_logger().info("Successfully performed segmentation!")

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