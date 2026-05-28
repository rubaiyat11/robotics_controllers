#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np


class PidControllerPyNode(Node):
    def __init__(self):
        super().__init__("pid_controller_py_node")

        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])

        self.force = np.array([0.0, 0.0, 0.0])
        self.omega = 0.5

        self.mass = 1.0
        self.dt = 0.01

        self.x = 0.0
        self.y = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.radius = 5.0
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.target_velocity = np.array([0.0, 0.0, 0.0])

        self.kp = np.array([2.0, 2.0, 2.0])
        self.ki = np.array([0.5, 0.5, 1.0])
        self.kd = np.array([0.2, 0.2, 0.6])

        self.error = np.array([0.0, 0.0, 0.0])
        self.velocity_error = np.array([0.0, 0.0, 0.0])
        self.prev_error = np.array([0.0, 0.0, 0.0])
        self.prev_velocity = np.array([0.0, 0.0, 0.0])
        self.integral = np.array([0.0, 0.0, 0.0])
        self.derivative = np.array([0.0, 0.0, 0.0])

        self.damping = 0.5
        self.gravity_accel = np.array([0.0, 0.0, -9.81])

        self.max_integral = 20.0
        self.max_force = 30.0

        self.time = 0.0


        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )


    def timer_callback(self):
        self.time += self.dt

        self.x = np.cos(self.time * self.omega) * self.radius
        self.y = np.sin(self.time * self.omega) * self.radius

        self.v_x = -self.radius * self.omega * np.sin(self.time * self.omega)
        self.v_y = self.radius * self.omega * np.cos(self.time * self.omega)

        self.target_position = np.array([self.x, self.y, 0.0])
        self.target_velocity = np.array([self.v_x, self.v_y, 0.0])

        self.error = self.target_position - self.position
        self.velocity_error = self.target_velocity - self.velocity


        self.integral += self.error * self.dt

        if np.linalg.norm(self.integral) > self.max_integral:
            self.unit_vector = self.integral / np.linalg.norm(self.integral)

            self.integral = self.unit_vector * self.max_integral


        self.derivative = self.velocity_error

        self.force = self.error * self.kp + self.integral * self.ki + self.derivative * self.kd

        if np.linalg.norm(self.force) > self.max_force:
            self.unit_vector = self.force / np.linalg.norm(self.force)

            self.force = self.unit_vector * self.max_force

        self.force -= self.damping * self.velocity

        self.gravity = self.mass * self.gravity_accel

        self.force += self.gravity

        self.acceleration = self.force / self.mass

        self.velocity += self.acceleration * self.dt

        self.position += self.velocity * self.dt


        self.prev_error = self.error
        self.prev_velocity = self.velocity
        self.get_logger().info(
            f"Pos: {self.position} | "
            f"Tar: {self.target_position} |"
            f"Vel: {self.velocity} | "
            f"Acc: {self.acceleration} | "
            f"Force: {self.force}"
        )


def main(args=None):
    rclpy.init(args=args)
    node= PidControllerPyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
