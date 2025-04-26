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
        # Berlangganan pada topik /odom
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # Berlangganan pada topik /base_scan untuk mengambil timestamp
        self.create_subscription(LaserScan, '/scan', self.base_scan_callback, 10)
        # Menyimpan timestamp terbaru dari base_scan
        self.latest_base_scan_stamp = None

    def base_scan_callback(self, msg: LaserScan):
        # Update timestamp dari data base_scan
        self.latest_base_scan_stamp = msg.header.stamp

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # Jika timestamp dari base_scan tersedia, gunakan itu, jika tidak gunakan timestamp odometry
        if self.latest_base_scan_stamp is not None:
            t.header.stamp = self.latest_base_scan_stamp
        else:
            t.header.stamp = msg.header.stamp
        # Pastikan frame induk adalah 'odom'
        t.header.frame_id = 'odom'
        # Tetapkan child_frame_id secara eksplisit ke 'base_footprint'
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