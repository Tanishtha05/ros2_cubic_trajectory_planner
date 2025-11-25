import rclpy
from rclpy.node import Node
from rclpy.task import Future
from ar_interface.msg import CubicTrajParams, CubicTrajCoeffs
from ar_interface.srv import ComputeCubicTraj

class CubicTrajPlanner(Node):
    def __init__(self):
        super().__init__('cubic_traj_planner')

        # Create a subscription to the 'trajectory_params' topic
        self.subscription = self.create_subscription(
            CubicTrajParams, 'cubic_traj_params', self.listener_callback, 10)

        # Create a publisher for the 'trajectory_coeffs' topic
        self.publisher = self.create_publisher(CubicTrajCoeffs, 'cubic_traj_coeffs', 10)

        # Create a client for the compute_cubic_traj service
        self.client = self.create_client(ComputeCubicTraj, 'compute_cubic_traj')

    def listener_callback(self, msg):
        if self.client.wait_for_service(timeout_sec=1.0):

            # Create a request for the service
            request = ComputeCubicTraj.Request()
            request.p0 = msg.p0
            request.pf = msg.pf
            request.v0 = msg.v0
            request.vf = msg.vf
            request.tf = msg.tf

            # Call the service and handle the response
            future = self.client.call_async(request)
            future.add_done_callback(lambda future: self.service_response_callback(future, request))

        else:
            self.get_logger().warn('Service not available')

    def service_response_callback(self, future, request):
        try:
            response = future.result()

            # Publish the coefficients and time parameters
            msg = CubicTrajCoeffs()
            msg.a0 = response.a0
            msg.a1 = response.a1
            msg.a2 = response.a2
            msg.a3 = response.a3
            msg.t0 = request.t0
            msg.tf = request.tf
            self.publisher.publish(msg)
            self.get_logger().info(f'Published trajectory coefficients: {msg}')

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CubicTrajPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
