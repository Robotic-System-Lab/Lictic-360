import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros

class LidarLimiterNode(Node):
    def __init__(self):
        super().__init__('lidar_limiter')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.declare_parameter('maxrange', 1.0)
        self.create_subscription(LaserScan, '/velodyne_scan', self.base_scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

    def base_scan_callback(self, msg: LaserScan):
        # Update timestamp dari data base_scan
        self.latest_base_scan_stamp = msg.header.stamp
        t = TransformStamped()
        t.header.frame_id = 'trash_scan'
        t.child_frame_id = 'trash_footprint'
        self.br.sendTransform(t)
        
        maxrange = self.get_parameter('maxrange').value
        limited_msg = LaserScan()
        limited_msg.header = msg.header
        limited_msg.angle_min = msg.angle_min
        limited_msg.angle_max = msg.angle_max
        limited_msg.angle_increment = msg.angle_increment
        limited_msg.time_increment = msg.time_increment
        limited_msg.scan_time = msg.scan_time
        limited_msg.range_min = msg.range_min
        limited_msg.range_max = maxrange
        limited_msg.ranges = [
            distance if distance <= 1 else float('inf')
        		for distance in msg.ranges
        ]
        limited_msg.header.frame_id = 'base_scan'
        self.scan_pub.publish(limited_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarLimiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()