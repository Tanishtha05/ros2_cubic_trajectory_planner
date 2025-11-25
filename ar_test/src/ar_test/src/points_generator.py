import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ar_interface.msg import CubicTrajParams
import random

class PointGenerator(Node):
    def __init__(self):
        super().__init__('points_generator')
        self.publisher_ = self.create_publisher(CubicTrajParams, 'cubic_traj_params', 10)
        self.timer = self.create_timer(10.0, self.publish_params)

    def publish_params(self):
        msg = CubicTrajParams()
        msg.x0 = random.uniform(-10, 10)
        msg.y0 = random.uniform(-10, 10)
        msg.z0 = random.uniform(-10, 10)
        msg.xf = random.uniform(-10, 10)
        msg.yf = random.uniform(-10, 10)
        msg.zf = random.uniform(-10, 10)
        msg.tf = random.uniform(4, 8)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = PointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
