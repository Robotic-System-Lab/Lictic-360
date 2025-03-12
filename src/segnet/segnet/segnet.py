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

class SegmentationNode(Node):
  def __init__(self):
    super().__init__('segnet')
    self.get_logger().info('Segmentation node has been started.')
    self.bridge = CvBridge()
    self.get_logger().info('Loading Model...')
    self.model = YOLO('./src/segnet/model/yolo11m-seg.pt')
    self.get_logger().info('Model loaded, ready to perform segmentation.')
    
    self.image_count = 0
    self.processing = False
    self.image_subscriber = self.create_subscription(
      Image,
      '/camera/image_raw',
      self.image_callback,
      10)
    self.segmentation_publisher = self.create_publisher(String, '/segnet', 10)

  def image_callback(self, msg):
    if self.processing:
      return

    self.processing = True
    self.get_logger().info(f'({self.image_count}) Received image data, performing segmentation...')
    self.image_count += 1

    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    results = self.model(cv_image)
    
    pred_img = results[0].plot()
    cv2.imshow('Predicted Image', pred_img)
    cv2.waitKey(1)

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

    deg360 = [{
              'label': None,
              'conf': 0,
              'x1': 0,
              'x2': 0
            }] * 360
    width = cv_image.shape[1]*6
    pixels_per_degree = width//360
    
    for data in segmentation_data:
      for degree in range(360):
        if data['x1'] <= degree * pixels_per_degree < data['x2']:
          if (deg360[degree]['conf'] == 0 or deg360[degree]['conf'] < data['conf']):
            deg360[degree] = {
              'label': data['label'],
              'conf': data['conf'],
              'x1': data['x1'],
              'x2': data['x2']
            }
    
    msg = String()
    msg.data = json.dumps(deg360)
    self.segmentation_publisher.publish(msg)

    self.processing = False
    self.get_logger().info(f'({self.image_count}) Successfully performed segmentation. Waiting for next image...')

def main(args=None):
  rclpy.init(args=args)
  node = SegmentationNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()