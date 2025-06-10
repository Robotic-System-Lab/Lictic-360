import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
import tf2_ros

class MapRebroadcasterNode(Node):
	def __init__(self):
		super().__init__('map_rebroadcaster')
		self.br = tf2_ros.TransformBroadcaster(self)
		self.latest_header_stamp = Float64()
  
		self.declare_parameter('maxrange', 5.0)
  
		self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
		self.odom_publisher = self.create_publisher(Odometry, '/odom_merged', 10)
		self.label_publisher = self.create_publisher(String, '/label', 10)
  
		self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
		self.create_subscription(LaserScan, '/velodyne_scan', self.callback_scan, 10)
		self.create_subscription(Float64, '/label_start', self.callback_start, 10)
		self.create_subscription(String, '/label_end', self.callback_all, 10)

		self.base_foot_print = TransformStamped()
		self.base_link = TransformStamped()
  
		self.odom_tf = TransformStamped()
		self.temp_odom = Odometry()
		self.save_odom = Odometry()
  
		self.scan_tf = TransformStamped()
		self.temp_scan = LaserScan()
		self.save_scan = LaserScan()
		self.limited_scan = LaserScan()
  
	def callback_odom(self, msg: Odometry):
		self.temp_odom = msg
	def callback_scan(self, msg: LaserScan):
		self.temp_scan = msg
	def callback_start(self, msg: Float64):
  	#############################################
  	#### Segmentation Timestamp saved
		timestamp = Time()
		timestamp.sec = int(msg.data)
		timestamp.nanosec = int((msg.data - int(msg.data)) * 1e9)
		self.latest_header_stamp = timestamp
  	#############################################
  	#### Temp Data saved
		self.odom_tf = TransformStamped()
		self.save_odom = self.temp_odom
  
		self.scan_tf = TransformStamped()
		self.save_scan = self.temp_scan
		self.limited_scan = LaserScan()
  	#############################################
  
  	#############################################
  	#### Odom saved
		self.save_odom.header.stamp = self.latest_header_stamp
		self.save_odom.header.frame_id = 'odom_merged'
		self.save_odom.child_frame_id = 'base_footprint'
		self.odom_tf.header.stamp = self.latest_header_stamp
		self.odom_tf.header.frame_id = 'odom_merged'
		self.odom_tf.child_frame_id = 'base_footprint'
		self.odom_tf.transform.translation.x = self.save_odom.pose.pose.position.x
		self.odom_tf.transform.translation.y = self.save_odom.pose.pose.position.y
		self.odom_tf.transform.translation.z = self.save_odom.pose.pose.position.z
		self.odom_tf.transform.rotation = self.save_odom.pose.pose.orientation
		# self.br.sendTransform(self.odom_tf)
  	#############################################
   
  	#############################################
  	#### base_foot_print saved
		self.base_foot_print = TransformStamped()
		self.base_foot_print.header.stamp = self.latest_header_stamp
		self.base_foot_print.header.frame_id = 'base_footprint'
		self.base_foot_print.child_frame_id = 'base_link'
		self.base_foot_print.transform.translation.x = self.save_odom.pose.pose.position.x
		self.base_foot_print.transform.translation.y = self.save_odom.pose.pose.position.y
		self.base_foot_print.transform.translation.z = self.save_odom.pose.pose.position.z
		self.base_foot_print.transform.rotation = self.save_odom.pose.pose.orientation
  	#############################################
   
  	#############################################
  	#### base_link saved
		self.base_link = TransformStamped()
		self.base_link.header.stamp = self.latest_header_stamp
		self.base_link.header.frame_id = 'base_footprint'
		self.base_link.child_frame_id = 'base_link'
		self.base_link.transform.translation.x = self.save_odom.pose.pose.position.x
		self.base_link.transform.translation.y = self.save_odom.pose.pose.position.y
		self.base_link.transform.translation.z = self.save_odom.pose.pose.position.z
		self.base_link.transform.rotation = self.save_odom.pose.pose.orientation
  	#############################################
  
  	#############################################
  	#### Scan saved
		# self.scan_tf.header.stamp = self.latest_header_stamp
		# self.scan_tf.header.frame_id = 'trash_scan'
		# self.scan_tf.child_frame_id = 'trash_footprint'
		# # self.br.sendTransform(scan_tf)
		maxrange = self.get_parameter('maxrange').value
		self.limited_scan.header.stamp = self.latest_header_stamp
		self.limited_scan.header.frame_id = 'base_scan'
		self.limited_scan.angle_min = self.save_scan.angle_min
		self.limited_scan.angle_max = self.save_scan.angle_max
		self.limited_scan.angle_increment = self.save_scan.angle_increment
		self.limited_scan.time_increment = self.save_scan.time_increment
		self.limited_scan.scan_time = self.save_scan.scan_time
		self.limited_scan.range_min = self.save_scan.range_min
		self.limited_scan.range_max = maxrange
		self.limited_scan.ranges = [
			distance if distance <= maxrange else float('inf')
			for distance in self.save_scan.ranges
		]
		# self.scan_publisher.publish(limited_scan)
  	#############################################
   
   
	def callback_all(self, msg: String):
		#############################################
		#### Whole republish
		label_msg = String()
		label_msg.data = msg.data
		self.label_publisher.publish(label_msg)
  
		self.odom_publisher.publish(self.save_odom)
		self.br.sendTransform(self.odom_tf)
  
		self.br.sendTransform(self.base_foot_print)
		self.br.sendTransform(self.base_link)
  
		self.scan_publisher.publish(self.limited_scan)
		# self.br.sendTransform(self.scan_tf)

def main(args=None):
	rclpy.init(args=args)
	node = MapRebroadcasterNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()