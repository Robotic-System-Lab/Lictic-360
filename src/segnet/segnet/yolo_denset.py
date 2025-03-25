import os
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import colorsys
import numpy as np
from ultralytics import YOLO
from rclpy.executors import MultiThreadedExecutor

class YOLODensetNode(Node):
  def __init__(self):
    super().__init__('yolo_denset')
    self.get_logger().info('Segmentation node has been started.')
    self.bridge = CvBridge()
    model = '11'
    
    self.get_logger().info('Loading YOLO Detection Model...')
    self.model = YOLO(f'./src/segnet/model/yolo{model}m.pt')
    self.get_logger().info(f'Detection model loaded on: {self.model.device}, ready to perform detection.')
    
    # --- New: Load YOLO Segmentation model ---
    self.get_logger().info('Loading YOLO Segmentation Model...')
    self.segnet = YOLO(f'./src/segnet/model/yolo{model}m-seg.pt')
    self.get_logger().info(f'Segmentation model loaded on: {self.segnet.device}, ready to perform segmentation.')
    
    # --- New: Create a color map for 91 labels ---
    self.label_colors = self.create_color_map(91)
    self.segcounter = 0

    self.cam_req = 1
    self.timestamp = 0
    self.subscribers = []
    self.images = [None] * self.cam_req
    self.deg360 = [{'label': None, 'conf': 0}] * 360
    
    self.segmentation_publisher = self.create_publisher(String, '/segnet', 10)
    for i in range(1):
      topic_name = f'/camera/image_raw'
      self.subscribers.append(
        self.create_subscription(
          Image,
          topic_name,
          lambda msg, idx=i: self.image_callback(msg, idx),
          10
        )
      )
    self.timer = self.create_timer(.4, self.display_images)

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

  def segment_image(self, image, index):
    """Gunakan YOLO untuk segmentasi dan deteksi pada gambar."""
    # Pertama: Dapatkan overlay segmentation dengan YOLO segmentation model
    results_seg = self.segnet(image)
    segmented_image = results_seg[0].plot()
    
    # Kedua: Lakukan deteksi dengan YOLO detection model
    results_det = self.model(image)
    for box in results_det[0].boxes:
      xyxy = box.xyxy[0].tolist()
      x1, y1, x2, y2 = map(int, xyxy)
      label_id = int(box.cls.item())
      color = self.label_colors.get(label_id, (0, 255, 0))
      cv2.rectangle(segmented_image, (x1, y1), (x2, y2), color, 2)

    # Update data segmen untuk sensor
    self.collect_segnet(results_det, segmented_image, index)
    return segmented_image

  def collect_segnet(self, results, cv_image, index):
    segmentation_data = []
    # For each detection returned by DetectNet
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
    pixels_per_degree = width // 360

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

  def display_images(self):
    """Gabungkan dan tampilkan gambar dari semua kamera dengan hasil segmentasi."""
    if all(image is not None for image in self.images):
      try:
        self.deg360 = [{'label': None, 'conf': 0}] * 360
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
        self.segmentation_publisher.publish(msg_out)
        

        self.segcounter +=1
        unique_labels = len(set(x for x in detected if x != -1))
        self.get_logger().info(f'Segmentation {self.segcounter} completed. Unique labels detected: {unique_labels}')
      except Exception as e:
        self.get_logger().error(f"Error during image concatenation: {e}")
      self.images = [None] * self.cam_req
    else:
      available_cameras = sum(1 for image in self.images if image is not None)
      self.get_logger().info(f"Available camera feeds: {available_cameras}/{self.cam_req}")

    

def main(args=None):
  rclpy.init(args=args)
  node = YOLODensetNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()