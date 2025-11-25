import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ar_interface.msg import CubicTrajCoeffs
import numpy as np

class PlotCubicTraj(Node):
    def __init__(self):
        super().__init__('plot_cubic_traj')

        # Subscribe to the trajectory coeffs topic
        self.subscription = self.create_subscription(
            CubicTrajCoeffs,
            'cubic_traj_coeffs',
            self.trajectory_callback,
            10)

        # Publishers for position, velocity, and acceleration trajectories
        self.position_publisher = self.create_publisher(Float64, 'position_trajectory', 10)
        self.velocity_publisher = self.create_publisher(Float64, 'velocity_trajectory', 10)
        self.acceleration_publisher = self.create_publisher(Float64, 'acceleration_trajectory', 10)

        self.get_logger().info('PlotCubicTraj Node Initialized')

    def trajectory_callback(self, msg):
        self.get_logger().info(f'Received coefficients: {msg}')

        a0 = msg.a0
        a1 = msg.a1
        a2 = msg.a2
        a3 = msg.a3
        tf = msg.tf

        # Create a time array from 0 to tf with 100 points
        time_array = np.linspace(0, tf, num=100)

        for t in time_array:
            # Compute position, velocity, and acceleration
            position = a0 + a1 * t + a2 * t**2 + a3 * t**3
            velocity = a1 + 2 * a2 * t + 3 * a3 * t**2
            acceleration = 2 * a2 + 6 * a3 * t

            # Publish the values to their respective topics
            position_msg = Float64()
            position_msg.data = position
            self.position_publisher.publish(position_msg)

            velocity_msg = Float64()
            velocity_msg.data = velocity
            self.velocity_publisher.publish(velocity_msg)

            acceleration_msg = Float64()
            acceleration_msg.data = acceleration
            self.acceleration_publisher.publish(acceleration_msg)

        self.get_logger().info('Published position, velocity, and acceleration trajectories')

def main(args=None):
    rclpy.init(args=args)
    node = PlotCubicTraj()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
