#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from scipy.linalg import solve_continuous_are


class cart_pole_lqr_controller_node(Node):
    def __init__(self):
        super().__init__("cart_pole_lqr_controller_node")

        self.x = 0.0
        self.x_dot = 0.0
        self.angle = 0.0
        self.angle_dot = 0.0
        self.state = np.array([
            self.x,
            self.x_dot,
            self.angle,
            self.angle_dot
            ])
        self.target_state = np.array([
            10.0,
            0.0,
            0.0,
            0.0
            ])

        self.m = 0.5
        self.M = 2.0

        self.length = 1.0

        self.force = np.array([0.0, 0.0])
        self.max_force = 100.0

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
            [1]
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

        self.force = float(-(self.K @ self.error_state))

        self.force = np.clip(
            self.force,
            -self.max_force,
            self.max_force
        )

        self.state_dot = self.A @ self.state + self.B.flatten() * self.force

        self.state += self.state_dot * self.dt

        self.get_logger().info(
            f"State = {self.state}"
        )








def main(args=None):
    rclpy.init(args=args)
    node = cart_pole_lqr_controller_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    