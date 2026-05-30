#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <cmath>


class pid_controller_cpp_node : public rclcpp::Node{
public:
    pid_controller_cpp_node() : Node("plant_node"){
        controller_update_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&pid_controller_cpp_node::timer_callback, this)
        );

        position = Eigen::Vector3d::Zero();
        velocity = Eigen::Vector3d::Zero();
        acceleration = Eigen::Vector3d::Zero();

        force = Eigen::Vector3d::Zero();
        omega = 0.5;

        mass = 1.0;
        dt = 0.01;

        x = 0.0;
        y = 0.0;
        v_x = 0.0;
        v_y = 0.0;
        radius = 5.0;
        target_position = Eigen::Vector3d::Zero();
        target_velocity = Eigen::Vector3d::Zero();

        kp << 2.0, 2.0, 2.0;
        ki << 0.5, 0.5, 1.0;
        kd << 0.2, 0.2, 0.6;

        error = Eigen::Vector3d::Zero();
        velocity_error = Eigen::Vector3d::Zero();
        prev_error = Eigen::Vector3d::Zero();
        integral = Eigen::Vector3d::Zero();
        derivative = Eigen::Vector3d::Zero();

        damping = 0.5;
        gravity_accel << 0.0, 0.0, -9.81;

        max_integral = 20.0;
        max_force = 30.0;

        time = 0.0;
    }

private:
    void timer_callback(){
        time += dt;

        x = std::cos(time * omega) * radius;
        y = std::sin(time * omega) * radius;

        v_x = - omega * y;
        v_y = omega * x;

        target_position << x, y, 0.0;
        target_velocity << v_x, v_y, 0.0;

        error = target_position - position;
        velocity_error = target_velocity - velocity;

        integral += error * dt;

        if(integral.norm() > max_integral){
            Eigen::Vector3d unit_vector = integral.normalized();

            integral = unit_vector * max_integral;
        }

        derivative = velocity_error;

        force = error.cwiseProduct(kp) + integral.cwiseProduct(ki) + derivative.cwiseProduct(kd);

        if(force.norm() > max_force){
            Eigen::Vector3d unit_vector = force.normalized();
            
            force = unit_vector * max_force;
        }

        force -= damping * velocity;

        Eigen::Vector3d gravity = mass * gravity_accel;
        
        force += gravity;

        acceleration = force / mass;

        velocity += acceleration * dt;

        position += velocity * dt;
    

        prev_error = error;

        RCLCPP_INFO(this->get_logger(), "Pos: x=%.2f, y=%.2f, z=%.2f", position.x(), position.y(), position.z());

    }

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Eigen::Vector3d force;

    Eigen::Vector3d target_position;
    Eigen::Vector3d target_velocity;

    Eigen::Vector3d kp;
    Eigen::Vector3d ki;
    Eigen::Vector3d kd;
    
    Eigen::Vector3d error;
    Eigen::Vector3d velocity_error;
    Eigen::Vector3d prev_error;

    Eigen::Vector3d integral;
    Eigen::Vector3d derivative;

    Eigen::Vector3d gravity_accel;

    double omega;
    double mass;

    double dt;

    double x;
    double y;
    double v_x;
    double v_y;

    double radius;

    double damping;
    double max_integral;
    double max_force;

    double time;

    rclcpp::TimerBase::SharedPtr controller_update_timer;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pid_controller_cpp_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}