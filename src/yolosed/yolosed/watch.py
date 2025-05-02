import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import atan2
from math import atan2, degrees

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('watch')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.get_logger().info("Odom subscriber node started.")

    def listener_callback(self, msg: Odometry):
        # Mengambil quaternion orientasi dari pesan Odometry
        q = msg.pose.pose.orientation
        # Menghitung yaw dari quaternion:
        # yaw = arctan2(2*(w*z + x*y), 1 - 2*(y² + z²))
        yaw_rad = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        yaw_deg = degrees(yaw_rad)
        # Normalisasi sehingga berada pada rentang 0-359 derajat:
        yaw_normalized = int((-yaw_deg) % 360)
        self.get_logger().info(f'Robot yaw: {yaw_normalized:.3f} degrees')

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()