#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from scipy.optimize import minimize


class mpc_controller_py_node(Node):
    def __init__(self):
        super().__init__("mpc_controller_py_node")

        self.position = 0.0
        self.velocity = 0.0

        self.state = np.array([
            self.position,
            self.velocity
        ])

        self.target_state = np.array([
            5.0,
            0.0
        ])

        self.A = np.array([
            [0.0, 1.0],
            [0.0, 0.0]
        ])

        self.B = np.array([
            [0.0],
            [1.0]
        ])

        self.dt = 0.1

        self.N = 10

        self.u = np.zeros(10)

        self.Q = np.array([
            [10.0, 0.0],
            [0.0, 1.0]
        ])

        self.R = 0.1


        timer = self.create_timer(
            self.dt,
            self.timer_callback
        )

    
    def cost_function(self, u_sequence):
        
        predicted_state = self.state.copy()

        total_cost = 0.0

        for force in u_sequence:
            state_dot = (
                self.A @ predicted_state
                + self.B.flatten() * force
            )

            predicted_state += state_dot * self.dt

            error = predicted_state - self.target_state

            state_cost = error.T @ self.Q @ error

            control_cost = self.R * force**2

            total_cost += state_cost + control_cost

        return total_cost
    
    def timer_callback(self):
        initial_guess = np.zeros(self.N)


        result = minimize(
            self.cost_function,
            initial_guess
        )

        optimal_sequence = result.x

        force = optimal_sequence[0]

        state_dot = (
            self.A @ self.state
            + self.B.flatten() * force
        )

        self.state += state_dot * self.dt

        self.get_logger().info(
            f"State={self.state}, Force={force:.3f}"
        )

        
def main(args=None):
    rclpy.init(args=args)
    node = mpc_controller_py_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()