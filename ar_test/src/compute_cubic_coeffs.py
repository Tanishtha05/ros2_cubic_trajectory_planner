import rclpy
from rclpy.node import Node
from ar_interface.srv import ComputeCubicTraj

class ComputeCubicCoeffs(Node):
    def __init__(self):
        super().__init__('compute_cubic_coeffs')
        self.srv = self.create_service(ComputeCubicTraj, 'compute_cubic_traj', self.compute_coeffs_callback)

    def compute_coeffs_callback(self, request, response):
        t0 = request.t0
        tf = request.tf
        dt = tf - t0

        # Compute coefficients
        response.a0 = request.p0
        response.a1 = request.v0
        response.a2 = (3 * (request.pf - request.p0) - (2 * request.v0 + request.vf) * dt) / (dt ** 2)
        response.a3 = (2 * (request.p0 - request.pf) + (request.v0 + request.vf) * dt) / (dt ** 3)

        self.get_logger().info(f'Computed coefficients: a0={response.a0}, a1={response.a1}, a2={response.a2}, a3={response.a3}')
        return response

def main(args=None):
    rclpy.init(args=args)
    compute_cubic_coeffs = ComputeCubicCoeffs()
    rclpy.spin(compute_cubic_coeffs)
    compute_cubic_coeffs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
