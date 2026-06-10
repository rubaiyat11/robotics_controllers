#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from scipy.linalg import solve_continuous_are
import numpy as np


class lqr_controller_py_node(Node):
    def __init__(self):
        super().__init__("lqr_controller_py_node")

        self.position = np.array([0.0, 0.0])
        self.velocity = np.array([0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0])

        self.mass = 1.5

        self.force = np.array([0.0, 0.0])
        self.max_force = 100.0

        self.target_position = np.array([5.0, 5.0])
        self.target_velocity = np.array([0.0, 0.0])

        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.target_state = np.array([5.0, 5.0, 0.0, 0.0])

        self.A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])

        self.B = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [1.0/self.mass, 0.0],
            [0.0, 1.0/self.mass]
        ])

        self.Q = np.array([
            [10.0, 0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])

        self.R = np.array([
            [1.0, 0.0],
            [0.0, 1.0]
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
            self.position[0],
            self.position[1],
            self.velocity[0],
            self.velocity[1]
        ])

        self.target_state = np.array([
            self.target_position[0],
            self.target_position[1],
            self.target_velocity[0],
            self.target_velocity[1]
        ])

        self.error_state = self.state - self.target_state

        self.force = -(self.K @ self.error_state)

        self.Fx = self.force[0]
        self.Fy = self.force[1]

        if(abs(self.Fx) > self.max_force):
            self.unit_vector = self.Fx / abs(self.Fx)
            self.Fx = self.unit_vector * self.max_force
        elif(abs(self.Fx) > self.max_force):
            self.unit_vector = self.Fy / abs(self.Fy)
            self.Fy = self.unit_vector / abs(self.Fy)

        self.force[0] = self.Fx
        self.force[1] = self.Fy

        self.acceleration = self.force / self.mass
        
        self.velocity += self.acceleration * self.dt
        
        self.position += self.velocity * self.dt

        
        self.get_logger().info(f"State: {self.state} | Error: {self.error_state}")
        

        
def main(args=None):
    rclpy.init(args=args)
    node = lqr_controller_py_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
