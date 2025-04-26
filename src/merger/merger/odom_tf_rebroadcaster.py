import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomTfRebroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_rebroadcaster')
        # Membuat broadcaster untuk mengirim transform
        self.br = tf2_ros.TransformBroadcaster(self)
        # Berlangganan pada topik /odom
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
    
    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # Gunakan waktu dari pesan odometry
        t.header.stamp = self.get_clock().now().to_msg()
        # Pastikan frame induk adalah 'odom' untuk tf tree yang lengkap
        t.header.frame_id = 'odom'
        # Gunakan child_frame_id dari pesan odom (biasanya 'base_link')
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