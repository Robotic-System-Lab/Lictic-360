import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
import json

class Watcher(Node):
	def __init__(self):
		super().__init__('watcher')
		self.create_subscription(
			LaserScan,
			'/scan',
			self.scan_callback,
			10)
		self.create_subscription(
			String,
			'/label',
			self.label_callback,
			10)
		self.create_subscription(
			Odometry,
			'/odom_merged',
			self.odom_callback,
			10)
		self.create_subscription(
			OccupancyGrid,
			'/map',
			self.map_callback,
			10)
		self.odom_data = None
		self.scan_data = None
		self.label_data = None
		self.map_data = None

	def odom_callback(self, msg: Odometry):
		self.odom_data = msg
		timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
		# self.get_logger().info(f'/odom: -{timestamp}-')
 
	def scan_callback(self, msg: LaserScan):
		self.scan_data = msg
		timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
		# self.get_logger().info(f'/scan: [{timestamp}]')

	def label_callback(self, msg: String):
		data = json.loads(msg.data)
		self.label_data = data
		if 'timestamp' in data:
			timestamp = data['timestamp']
			# self.get_logger().info(f'/label: {timestamp}')
 
	def map_callback(self, msg: OccupancyGrid):
		self.map_data = msg
		timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
  
		print(f'/label: {timestamp}')
		# self.get_logger().info(f'/scan: [{timestamp}]')

def main(args=None):
	rclpy.init(args=args)
	watcher = Watcher()
	rclpy.spin(watcher)
	watcher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()