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
import jetson_inference     # <-- Added for DetectNet
import jetson_utils         # <-- Added for image conversion
from rclpy.executors import MultiThreadedExecutor
from .openstitch.stitcher import Stitcher

class SegmentationNode(Node):
  def __init__(self):
    super().__init__('denset')
    self.get_logger().info('Segmentation node has been started.')
    self.bridge = CvBridge()
    self.stitcher = Stitcher(detector="sift", confidence_threshold=.3)
    
    self.get_logger().info('Loading DetectNet Model...')
    self.model = jetson_inference.detectNet("ssd-mobilenet-v2", threshold=0.5)
    self.get_logger().info('DetectNet model loaded, ready to perform detection.')
    
    # --- New: Initialize segNet for segmentation overlay ---
    self.get_logger().info('Loading SegNet Model for segmentation overlay...')
    self.segnet = jetson_inference.segNet("fcn-resnet18-voc")
    self.segnet.SetOverlayAlpha(150.0)
    # --- End New ---
    
    # --- New: Create a color map for 91 labels ---
    self.label_colors = self.create_color_map(91)
    self.segcounter = 0
    # --- End New ---

    self.timestamp = 0
    self.subscribers = []
    self.images = [None] * 6
    self.deg360 = [{'label': None, 'conf': 0}] * 360
    
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

  def detect_image(self, image):
    """Gunakan DetectNet untuk image detection dan tambahkan overlay dari segNet segmentation."""
    # Lakukan detection dengan DetectNet
    cuda_image_det = jetson_utils.cudaFromNumpy(image)
    detections = self.model.Detect(cuda_image_det)

    # Gambar bounding boxes pada gambar segmented_image
    for detection in detections:
      x1 = int(detection.Left)
      y1 = int(detection.Top)
      x2 = int(detection.Right)
      y2 = int(detection.Bottom)
      label_id = int(detection.ClassID)
      # Gunakan warna dari color map berdasarkan id label (0-90)
      color = self.label_colors.get(label_id, (0, 255, 0))
      cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)

    # Update data segmen untuk sensor (tetap ada proses collect_segnet)
    self.collect_segnet(detections, image)
    return image

  def collect_segnet(self, results, cv_image):
    segmentation_data = []
    # For each detection returned by DetectNet
    for detection in results:
      label = detection.ClassID           # Retrieve class ID
      conf = detection.Confidence         # Retrieve detection confidence
      x1 = int(detection.Left)              # Left coordinate (x1)
      x2 = int(detection.Right)             # Right coordinate (x2)
      segmentation_data.append({
          'label': int(label),
          'conf': conf,
          'x1': x1,
          'x2': x2,
      })

    width = cv_image.shape[1]*6
    pixels_per_degree = width // 360

    for data in segmentation_data:
      for degree in range(0, 360):
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

        collected_images = [image for image in self.images]
        each_size = self.images[0].shape[1]
        min_size = each_size-(30*5)

        combined_image = self.stitcher.stitch(collected_images)
        combined_size = combined_image.shape[1]

        if (combined_size//6 < min_size):
          raise RuntimeError(f"Combined size doesn't meet the minimum size requirement ({min_size} < {combined_size})")
        
        segmented_images = []
        for i in range(6):
            start_x = i * (combined_image.shape[1] // 6)
            end_x = (i + 1) * (combined_image.shape[1] // 6)
            camera_image = combined_image[:, start_x:end_x]
            segmented_image = self.detect_image(camera_image)
            segmented_images.append(segmented_image)
        processed_images = cv2.hconcat(segmented_images)
        
        target_width, target_height = 2160, 240
        if processed_images.shape[1] < target_width or processed_images.shape[0] < target_height:
            reshaped_image = cv2.resize(processed_images, (target_width, target_height))
        cv2.imshow('Multi Camera Display (6x1)', reshaped_image)

        cv2.waitKey(1)
        detected = [x['label']+1 if x['label'] is not None else -1 for x in self.deg360]
        # detected = [detected[(i - 309) % 360] for i in range(360)]
        payload = {
          'timestamp': self.timestamp,
          'detected': detected
        }
        msg_out = String()
        msg_out.data = json.dumps(payload)
        self.segmentation_publisher.publish(msg_out)

        self.segcounter +=1
        self.get_logger().info(f'Segmentation {self.segcounter} completed')
      except Exception as e:
        self.get_logger().error(f"Error: {e}")
      self.images = [None] * 6
    else:
      # available_cameras = sum(1 for image in self.images if image is not None)
      # self.get_logger().error(f"Available camera feeds: {available_cameras}/6")
      self.get_logger().warn(f"Waiting..")

    

def main(args=None):
  rclpy.init(args=args)
  node = SegmentationNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()