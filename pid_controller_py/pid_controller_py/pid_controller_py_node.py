#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class PidControllerPyNode(Node):
    def __init__(self):
        super().__init__("pid_controller_py_node")

        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0

        self.force = 0.0

        self.mass = 1.0
        self.dt = 0.01

        self.target_position = 10.0

        self.kp = 2.0
        self.ki = 0.5
        self.kd = 1.0

        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

        self.damping = 0.5
        self.max_integral = 20.0

        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )


    def timer_callback(self):
        self.error = self.target_position - self.position

        self.integral += self.error * self.dt

        if self.integral > self.max_integral:
            self.integral = self.max_integral

        if self.integral < -self.max_integral:
            self.integral = -self.max_integral


        self.derivative = (self.error - self.prev_error) / self.dt

        self.force = self.error * self.kp + self.integral * self.ki + self.derivative * self.kd

        self.force -= self.damping * self.velocity

        self.acceleration = self.force / self.mass

        self.velocity += self.acceleration * self.dt

        self.position += self.velocity * self.dt


        self.prev_error = self.error
        self.get_logger().info(
            f"Pos: {self.position:.2f} | "
            f"Vel: {self.velocity:.2f} | "
            f"Acc: {self.acceleration:.2f} | "
            f"Force: {self.force:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node= PidControllerPyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
