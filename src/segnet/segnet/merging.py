import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2

class MergerNode(Node):
  def __init__(self):
    super().__init__('merger')
    self.get_logger().info('Merger node has been started.')
    self.map_subscriber = self.create_subscription(
      OccupancyGrid,
      '/map',
      self.map_callback,
      10)
    self.scan_subscriber = self.create_subscription(
      LaserScan,
      '/scan',
      self.scan_callback,
      10)
    self.image_subscriber = self.create_subscription(
      Image,
      '/camera/image_raw',
      self.image_callback,
      10)
    
    self.map_count = 0
    self.scan_count = 0
    self.image_count = 0
    self.temp_dir = '/home/lamp/workspaces/gmapping/src/segnet/segnet/temp'
    os.makedirs(self.temp_dir, exist_ok=True)
    self.bridge = CvBridge()

  def map_callback(self, msg):
    self.get_logger().info(f'Received map data: {msg}')
    self.map_count += 1
    if self.map_count <= 2:
      file_path = os.path.join(self.temp_dir, f'map_{self.map_count}.txt')
      with open(file_path, 'w') as file:
        file.write(str(msg))
        
  def scan_callback(self, msg):
    self.get_logger().info(f'Received scan data: {msg}')
    self.scan_count += 1
    if self.scan_count <= 2:
      file_path = os.path.join(self.temp_dir, f'scan_{self.scan_count}.txt')
      with open(file_path, 'w') as file:
        file.write(str(msg))

  def image_callback(self, msg):
    self.get_logger().info(f'Received image data')
    self.image_count += 1
    if self.image_count % 60 == 0:
      cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
      file_path = os.path.join(self.temp_dir, 'dataset', f'image_{self.image_count/30}.png')
      cv2.imwrite(file_path, cv_image)
      cv2.imshow('Image', cv_image)
      cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  node = MergerNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()