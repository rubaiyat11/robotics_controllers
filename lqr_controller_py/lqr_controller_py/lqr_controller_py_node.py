#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from scipy.linalg import solve_continuous_are
import numpy as np


class helix_pid_controller_py_node(Node):
    def __init__(self):
        super().__init__("helix_pid_controller_py_node")

        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0

        self.mass = 1.5

        self.force = 0.0
        self.max_force = 100.0

        self.target_position = 10.0
        self.target_velocity = 0.0

        self.state = np.array([0.0, 0.0])
        self.target_state = np.array([10.0, 0.0])

        self.A = np.array([
            [0.0, 1],
            [0.0 , 0.0]
        ])

        self.B = np.array([
            [0.0],
            [1.0/self.mass]
        ])

        self.Q = np.array([
            [10.0, 0],
            [0, 3.0]
        ])

        self.R = np.array([
            [1.0]
        ])

        self.P = solve_continuous_are(
            self.A,
            self.B,
            self.Q,
            self.R
        )

        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

        self.damping = 0.5
        
        self.dt = 0.01

        self.timer = self.create_timer(0.01, self.timer_callback)

    
    def timer_callback(self):

        self.state = np.array([
            self.position,
            self.velocity
        ])

        self.error_state = self.state - self.target_state

        self.force = -(self.K @ self.error_state).item()

        if abs(self.force) > self.max_force:
            self.force = np.sign(self.force) * self.max_force


        self.acceleration = self.force / self.mass
        
        self.velocity += self.acceleration * self.dt
        
        self.position += self.velocity * self.dt

        
        self.get_logger().info(f"State: {self.state} | Error: {self.error_state}")
        

        
def main(args=None):
    rclpy.init(args=args)
    node = helix_pid_controller_py_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
