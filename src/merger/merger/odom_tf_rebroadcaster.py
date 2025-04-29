import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTfRebroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_rebroadcaster')
        # Membuat broadcaster untuk mengirim transform
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/velodyne_scan', self.base_scan_callback, 10)
        # Publisher untuk menerbitkan LaserScan dengan frame id yang telah diubah
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.latest_base_scan_stamp = None
        self.timer = self.create_timer(0.1, self.br_footprint_to_baselink)  # 10 Hz
        self.timer = self.create_timer(0.1, self.br_baselink_to_basescan)  # 10 Hz

    def br_footprint_to_baselink(self):
        t = TransformStamped()
        if self.latest_base_scan_stamp is not None:
            t.header.stamp = self.latest_base_scan_stamp
            t.header.frame_id = 'base_footprint'
            t.child_frame_id = 'base_link'

            # Static offset, adjust if needed
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.2  # Adjust to your robot config

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.br.sendTransform(t)
            
    def br_baselink_to_basescan(self):
        t = TransformStamped()
        if self.latest_base_scan_stamp is not None:
            t.header.stamp = self.latest_base_scan_stamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'base_scan'

            # Static offset, adjust if needed
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.2  # Adjust to your robot config

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.br.sendTransform(t)

    def base_scan_callback(self, msg: LaserScan):
        # Update timestamp dari data base_scan
        self.latest_base_scan_stamp = msg.header.stamp
        # Ubah frame id dari 'velodyne' menjadi 'base_scan'
        msg.header.frame_id = 'base_scan'
        # Terbitkan pesan LaserScan yang telah diperbarui
        self.scan_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # Jika timestamp dari base_scan tersedia, gunakan itu, jika tidak gunakan timestamp odometry
        if self.latest_base_scan_stamp is not None:
            t.header.stamp = self.latest_base_scan_stamp
        else:
            t.header.stamp = msg.header.stamp
        # Pastikan frame induk adalah 'odom'
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        # Salin informasi posisi
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        # Salin informasi orientasi
        t.transform.rotation = msg.pose.pose.orientation
        
        # Broadcast transform yang telah disiapkan
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTfRebroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()