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
		self.static_br = tf2_ros.StaticTransformBroadcaster(self)
		self.initialized_transform = False
  
		self.declare_parameter('maxrange', 5.0)
  
		self.odom_publisher = self.create_publisher(Odometry, '/odom_merged', 10)
		self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
		self.label_publisher = self.create_publisher(String, '/label', 10)
  
		self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
		self.create_subscription(LaserScan, '/velodyne_scan', self.callback_scan, 10)
		self.create_subscription(Float64, '/label_start', self.callback_start, 10)
		self.create_subscription(String, '/label_end', self.callback_restart, 10)
  
		self.latest_stamp = None

		self.base_foot_print = TransformStamped()
		self.base_link = TransformStamped()
  
		self.odom_tf = TransformStamped()
		self.temp_odom = Odometry()
		self.save_odom = Odometry()
  
		self.scan_tf = TransformStamped()
		self.temp_scan = LaserScan()
		self.save_scan = LaserScan()
		self.limited_scan = LaserScan()
	
	def init_merged_frames(self, msg: Odometry):
		if not self.initialized_transform:
			static_fp_to_merged_tf = TransformStamped()
			static_fp_to_merged_tf.header.stamp = msg.header.stamp
			static_fp_to_merged_tf.header.frame_id = 'lictic_footprint'
			static_fp_to_merged_tf.child_frame_id = 'lictic_link'
			static_fp_to_merged_tf.transform.translation.x = 0.0
			static_fp_to_merged_tf.transform.translation.y = 0.0
			static_fp_to_merged_tf.transform.translation.z = 0.0
			static_fp_to_merged_tf.transform.rotation.x = 0.0
			static_fp_to_merged_tf.transform.rotation.y = 0.0
			static_fp_to_merged_tf.transform.rotation.z = 0.0
			static_fp_to_merged_tf.transform.rotation.w = 1.0
   
			static_merged_to_scan_tf = TransformStamped()
			static_merged_to_scan_tf.header.stamp = msg.header.stamp
			static_merged_to_scan_tf.header.frame_id = 'lictic_link'
			static_merged_to_scan_tf.child_frame_id = 'lictic_scan'
			static_merged_to_scan_tf.transform.translation.x = 0.0
			static_merged_to_scan_tf.transform.translation.y = 0.0
			static_merged_to_scan_tf.transform.translation.z = 0.0
			static_merged_to_scan_tf.transform.rotation.x = 0.0
			static_merged_to_scan_tf.transform.rotation.y = 0.0
			static_merged_to_scan_tf.transform.rotation.z = 0.0
			static_merged_to_scan_tf.transform.rotation.w = 1.0

			self.static_br.sendTransform([static_fp_to_merged_tf, static_merged_to_scan_tf])
			self.initialized_transform = True
			self.get_logger().warn('Static merged transform initialized.')
  
	def callback_odom(self, msg: Odometry):
		self.latest_stamp = self.get_clock().now().to_msg()
  
		self.temp_odom = msg
		self.init_merged_frames(msg=msg)
  
		self.save_odom.header.stamp = self.latest_stamp
		self.odom_publisher.publish(self.save_odom)
  
  	#############################################
  	#### odom_manipulation
		tempdata = self.save_odom
  	#############################################
  	#### odom_tf inherited from save_odom
		self.odom_tf = TransformStamped()
		self.odom_tf.header.stamp = self.latest_stamp
		self.odom_tf.header.frame_id = 'lictic_odom'
		self.odom_tf.child_frame_id = 'lictic_footprint'
		self.odom_tf.transform.translation.x = tempdata.pose.pose.position.x
		self.odom_tf.transform.translation.y = tempdata.pose.pose.position.y
		self.odom_tf.transform.translation.z = tempdata.pose.pose.position.z
		self.odom_tf.transform.rotation = tempdata.pose.pose.orientation
  
  	#############################################
  	#### all transform broadcasted
		self.br.sendTransform(self.odom_tf)
  
	def callback_scan(self, msg: LaserScan):
		self.temp_scan = msg

		if (self.latest_stamp is not None):
			self.limited_scan.header.stamp = self.latest_stamp
			self.scan_publisher.publish(self.limited_scan)
  
	def callback_start(self, msg: Float64):
  	#############################################
  	#### Segmentation Timestamp saved
		timestamp = Time()
		timestamp.sec = int(msg.data)
		timestamp.nanosec = int((msg.data - int(msg.data)) * 1e9)
  	#############################################
  	#### Temp Data saved
		self.save_odom = self.temp_odom
  
		self.scan_tf = TransformStamped()
		self.save_scan = self.temp_scan
		self.limited_scan = LaserScan()
  	#############################################
  
  	#############################################
  	#### Odom saved
		self.save_odom.header.frame_id = 'odom_merged'
		self.save_odom.child_frame_id = 'base_footprint'
  
  	#############################################
  	#### Scan saved
		maxrange = self.get_parameter('maxrange').value
		self.limited_scan.header.frame_id = 'lictic_scan'
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
   
	def callback_restart(self, msg: String):
		#############################################
		#### Whole republish
		label_msg = String()
		label_msg.data = msg.data
		self.label_publisher.publish(label_msg)

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