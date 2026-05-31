#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np


class PidControllerPyNode(Node):
    def __init__(self):
        super().__init__("pid_controller_py_node")

        self.position = np.array([0.0, 0.0, 0.0])   #Plant current position
        self.velocity = np.array([0.0, 0.0, 0.0])   #Plant current velocity
        self.acceleration = np.array([0.0, 0.0, 0.0]) #Plant current acceleration

        self.force = np.array([0.0, 0.0, 0.0])      #The control input of PID
        self.omega = 0.5                        #Angular velocity for circling target

        self.mass = 1.0     #Mass of plant
        self.dt = 0.01      #Time interval of the loop

        self.x = 0.0        #x axis position of target
        self.y = 0.0        #y axis position of target
        self.v_x = 0.0      #x axis velocity of target
        self.v_y = 0.0      #y axis velocity of target
        self.radius = 5.0   #radius of the circle created by the movement of plant
        self.target_position = np.array([0.0, 0.0, 0.0])    #Target position (trajectory)
        self.target_velocity = np.array([0.0, 0.0, 0.0])    #Target velocity (changing)

        self.kp = np.array([2.0, 2.0, 2.0])     #Proportional constant or weight
        self.ki = np.array([0.5, 0.5, 1.0])     #Integral constant or weight
        self.kd = np.array([0.2, 0.2, 0.6])     #Derivative constant or weight

        self.error = np.array([0.0, 0.0, 0.0])  #Distance from target to plant current position
        self.velocity_error = np.array([0.0, 0.0, 0.0])         #Desired Velocity 
        self.prev_error = np.array([0.0, 0.0, 0.0])     #For storing previous loop's error value
        self.prev_velocity = np.array([0.0, 0.0, 0.0])  #For storing previous loop's velocity value
        self.integral = np.array([0.0, 0.0, 0.0])       #Integral value storing variable
        self.derivative = np.array([0.0, 0.0, 0.0])     #Derivative value storing variable

        self.damping = 0.5      #Faking the force felt by plant moving through air with changing velocity
        self.gravity_accel = np.array([0.0, 0.0, -9.81])    #Gravity constraint

        self.max_integral = 20.0    #Limiting integral value for anti-windup
        self.max_force = 30.0       #Limiting actuator force for actuator saturation

        self.time = 0.0      #total time


        self.timer = self.create_timer(
            self.dt,
            self.timer_callback
        )


    def timer_callback(self):
        self.time += self.dt        #Tracking total time passed

        #X & Y axis positions of target
        self.x = np.cos(self.time * self.omega) * self.radius
        self.y = np.sin(self.time * self.omega) * self.radius

        #Velocity of the target trajectory
        self.v_x = -self.radius * self.omega * np.sin(self.time * self.omega)
        self.v_y = self.radius * self.omega * np.cos(self.time * self.omega)

        #Initializing the target
        self.target_position = np.array([self.x, self.y, 0.0])
        self.target_velocity = np.array([self.v_x, self.v_y, 0.0])      #Target trajectory

        #Calculating errors
        self.error = self.target_position - self.position
        self.velocity_error = self.target_velocity - self.velocity


        self.integral += self.error * self.dt       #Calculates steady state error of velocity

        if np.linalg.norm(self.integral) > self.max_integral:       #Antiwindup logic
            self.unit_vector = self.integral / np.linalg.norm(self.integral)

            self.integral = self.unit_vector * self.max_integral


        self.derivative = self.velocity_error       #Uses desired velocity as the parameter

        self.force = self.error * self.kp + self.integral * self.ki + self.derivative * self.kd     #Control ouput

        if np.linalg.norm(self.force) > self.max_force:             #Clamping autuator force
            self.unit_vector = self.force / np.linalg.norm(self.force)

            self.force = self.unit_vector * self.max_force

        self.force -= self.damping * self.velocity      #Dampings

        self.gravity = self.mass * self.gravity_accel   #Gravity

        self.force += self.gravity          #Resultant

        self.acceleration = self.force / self.mass

        self.velocity += self.acceleration * self.dt    #Applied velocity

        self.position += self.velocity * self.dt        #Applied position change


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
