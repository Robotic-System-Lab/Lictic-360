import os
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.executors import MultiThreadedExecutor

class SegmentationNode(Node):
  def __init__(self):
    super().__init__('segnet')
    model = '11'
    
    self.get_logger().info('Segmentation node has been started.')
    self.bridge = CvBridge()
    self.get_logger().info('Loading Model...')
    self.model = YOLO(f'./src/segnet/model/yolo{model}m-seg.pt')
    self.get_logger().info('Model loaded, ready to perform segmentation.')
    
    self.timestamp = 0
    self.subscribers = []
    self.images = [None] * 6
    self.deg360 = [{
                'label': None,
                'conf': 0,
              }] * 360
    
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
    self.timer = self.create_timer(0.1, self.display_images)

  def image_callback(self, msg, index):
    self.get_logger().info(f'Received image data, performing segmentation...')
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    self.images[index] = cv_image

  def collect_segnet(self, results, cv_image, index):
    segmentation_data = []
    for result in results:
      for box in result.boxes:
        label = box.cls.item()
        conf = box.cls.item()
        xyxy = box.xyxy[0].tolist()
        x1, y1, x2, y2 = xyxy
        segmentation_data.append({
          'label': int(label),
          'conf': conf,
          'x1': int(x1),
          'x2': int(x2),
        })

    width = cv_image.shape[1]*6
    pixels_per_degree = width//360
    
    for data in segmentation_data:
      for degree in range(index*60, (index+1)*60):
        if data['x1'] <= degree * pixels_per_degree < data['x2']:
          if (self.deg360[degree]['conf'] == 0 or self.deg360[degree]['conf'] < data['conf']):
            self.deg360[degree] = {
              'label': data['label'],
              'conf': data['conf'],
              'x1': data['x1'],
              'x2': data['x2']
            }

  def crop_center_width(self, image, target_width):
    """Memotong gambar pada bagian tengah dengan width yang ditentukan."""
    _, width, _ = image.shape
    if target_width >= width:
      return image
    x_start = (width - target_width) // 2
    return image[:, x_start:x_start + target_width]

  def segment_image(self, image, index):
    """Gunakan segNet untuk segmentasi gambar."""
    results = self.model(image)
    segmented_image = results[0].plot()
    self.collect_segnet(results, image, index)
    return segmented_image

  def display_images(self):
    """Gabungkan dan tampilkan gambar dari semua kamera dengan hasil segmentasi."""
    if all(image is not None for image in self.images):
      try:
        self.timestamp = time.time()
        processed_images = [self.segment_image(image, idx) for idx, image in enumerate(self.images)]
        combined_image = cv2.hconcat(processed_images)
        cv2.imshow('Multi Camera Display (6x1)', combined_image)
        cv2.waitKey(1)

        detected = [x['label']+1 if x['label'] is not None else -1 for x in self.deg360]
        detected = [detected[(i - 309) % 360] for i in range(360)]
        payload = {
          'timestamp': self.timestamp,
          'detected': detected
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        
        self.get_logger().info(f'Segmentation completed.')
      except Exception as e:
        self.get_logger().error(f"Error during image concatenation: {e}")
    else:
      self.get_logger().warning("Not all camera feeds are available.")
    

def main(args=None):
  rclpy.init(args=args)
  node = SegmentationNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()