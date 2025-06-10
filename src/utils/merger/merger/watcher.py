import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import Odometry
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

	def odom_callback(self, msg: Odometry):
		timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
		self.get_logger().info(f'/odom_..: {timestamp}')
 
	def scan_callback(self, msg: LaserScan):
		timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
		self.get_logger().info(f'/scan: {timestamp}')

	def label_callback(self, msg: String):
		try:
			data = json.loads(msg.data)
			if 'timestamp' in data:
				timestamp = data['timestamp']
				self.get_logger().info(f'/label: {timestamp}')
			else:
				self.get_logger().warn('Key "timestamp" not found in JSON data.')
		except json.JSONDecodeError:
			self.get_logger().error('Invalid JSON received on /label.')

def main(args=None):
	rclpy.init(args=args)
	watcher = Watcher()
	rclpy.spin(watcher)
	watcher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()