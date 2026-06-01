#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np


class helix_pid_controller_py_node(Node):
    def __init__(self):
        super().__init__("helix_pid_controller_py_node")

        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.acceleration = np.array([0.0, 0.0, 0.0])

        self.force = np.array([0.0, 0.0, 0.0])

        self.integral = np.array([0.0, 0.0, 0.0])
        self.derivative = np.array([0.0, 0.0, 0.0])

        self.error = np.array([0.0, 0.0, 0.0])
        self.velocity_error = np.array([0.0, 0.0, 0.0])

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.target_velocity = np.array([0.0, 0.0, 0.0])

        self.p_x = 0.0
        self.p_y = 0.0
        self.p_z = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.v_z = 0.0


        self.kp = np.array([2.0, 2.0, 3.0])
        self.ki = np.array([0.8, 0.8, 2.5])
        self.kd = np.array([0.5, 0.5, 2.0])

        self.damping = 0.5
        self.gravity_acceleration = np.array([0.0, 0.0, -9.81])

        self.max_integral = 50.0
        self.min_integral = -50.0
        self.max_force = 50.0

        self.omega = 0.5

        self.radius = 3

        self.mass = 1.5
        
        self.dt = 0.01
        self.time = 0.0

        self.timer = self.create_timer(0.01, self.timer_callback)

    
    def timer_callback(self):
        self.time += self.dt

        self.p_x = self.radius * np.cos(self.omega * self.time)
        self.p_y = self.radius * np.sin(self.omega * self.time)
        self.p_z = self.time

        self.target_position = np.array([self.p_x, self.p_y, self.p_z])

        self.v_x = - self.radius * self.omega * np.sin(self.omega * self.time)
        self.v_y = self.radius * self.omega * np.cos(self.omega * self.time)
        self.v_z = 1.0

        self.target_velocity = np.array([self.v_x, self.v_y, self.v_z])

        self.error = self.target_position - self.position

        self.integral += self.error * self.dt

        if np.linalg.norm(self.integral) > self.max_integral:
            self.unit_vector_integral = self.integral / np.linalg.norm(self.integral)

            self.integral = self.unit_vector_integral * self.max_integral


        self.velocity_error = self.target_velocity - self.velocity

        self.derivative = self.velocity_error

        self.force = self.error * self.kp + self.integral * self.ki + self.derivative * self.kd

        if np.linalg.norm(self.force) > self.max_force:
           self.unit_vector_force = self.force / np.linalg.norm(self.force)

           self.force = self.unit_vector_force * self.max_force


        self.gravity = self.gravity_acceleration * self.mass

        self.force += self.gravity

        self.force -= self.damping * self.velocity

        self.acceleration = self.force / self.mass
        
        self.velocity += self.acceleration * self.dt
        
        self.position += self.velocity * self.dt

        
        self.get_logger().info(f"Pos: {self.position}  | Target Pos: {self.target_position} | Vel: {self.velocity} | Accel: {self.acceleration} | Force: {self.force}")
        

        
def main(args=None):
    rclpy.init(args=args)
    node = helix_pid_controller_py_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
