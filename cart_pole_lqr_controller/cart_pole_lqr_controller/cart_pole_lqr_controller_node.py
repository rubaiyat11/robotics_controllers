#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from scipy.linalg import solve_continuous_are


class cart_pole_lqr_controller_node(Node):
    def __init__(self):
        super().__init__("cart_pole_lqr_controller_node")

        self.x = 0.0
        self.angle = 0.0
        self.state = np.array([self.x, self.angle])
        self.target_state = np.array([10.0, 30.0])

        self.m = 0.5
        self.M = 2.0

        self.length = 1.0

        self.force = np.array([0.0, 0.0])
        self.max_force = 0.0

        self.g = 9.8

        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.g*self.m)/self.M, 0],
            [0, 0, 0, 1],
            [0, 0, (self.M + self.m) * self.g / self.M * self.length, 0]
        ])

        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/self.M * self.length]
        ])

        self.Q = np.array([
            [10, 0, 0, 0],
            [0, 10, 0, 0],
            [0, 0, 100, 0],
            [0, 0, 0, 10]
        ])

        self.R = np.array([
            [1],
            [10]
        ])

        self.P = solve_continuous_are(
            self.A,
            self.B,
            self.Q,
            self.R
        )

        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

        self.dt = 0.01

        self.timer = self.create_timer(0.01, self.timer_callback)

    
    def timer_callback(self):

        self.error_state = self.state - self.target_state

        self.force = -(self.K * self.error_state)

        self.fx = self.force[1]
        self.ftheta = self.force[3]

        if abs(self.fx) > self.max_force:
            self.fx = self.max_force
        elif abs(self.ftheta) > self.max_force:
            self.ftheta = self.max_force

        self.force[1] = self.fx
        self.force[3] = self.ftheta

        self.acceleration += self.force/(self.m + self.M)

        self.velocity += self.acceleration * self.dt

        self.position += self.velocity * self.dt

        self.get_logger().info(f"Position: {self.position} | Error: {self.error_state}")








def main(args=None):
    rclpy.init(args=args)
    node = cart_pole_lqr_controller_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    