#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <chrono>


class cart_pole_lqr_controller_cpp_node : public rclcpp::Node{
public:
    cart_pole_lqr_controller_cpp_node() : Node("cart_pole_lqr_controller_cpp_node"){
        controller_update_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&cart_pole_lqr_controller_cpp_node::timer_callback, this)
        );
    

    m = 0.5;
    M = 2.0;
    length = 1.0;
    force = 0.0;
    max_force = 100.0;
    dt = 0.01;
    g = 9.8;

    A.setZero();
    B.setZero();
    K.setZero();
    Q.setZero();

    state = Eigen::Vector4d::Zero();
    target_state << 10.0,
                    0.0,
                    0.0,
                    0.0;

    A << 0, 1, 0, 0,
         0, 0, (g*m)/M, 0,
         0, 0, 0, 1,
         0, 0, (M + m)*g/M*length, 0;

    B << 0,
         1/M,
         0,
         -1/M*length;
    
    Q << 10, 0, 0, 0,
         0, 10,  0, 0,
         0, 0, 100, 0,
         0, 0, 0, 10;
    
    R << 1.0;

    K << -3.16227766,  -6.3360993,  -94.56792517, -25.97337483;

    }

private:
    void timer_callback(){
        Eigen::Vector4d error_state = state - target_state;

        force = -(K * error_state)(0);

        force = std::clamp(force, -max_force, max_force);

        Eigen::Vector4d state_dot = A * state + B * force;

        state += state_dot * dt;

        RCLCPP_INFO(this->get_logger(),
            "x=%.2f xdot=%.2f theta=%.2f thetadot=%.2f u=%.2f",
            state(0), state(1), state(2), state(3), force
        );

    }

    Eigen::Vector4d state;
    Eigen::Vector4d target_state;
    Eigen::Matrix4d A;
    Eigen::Vector4d B;
    Eigen::Matrix4d Q;
    Eigen::Matrix<double, 1, 1> R;
    Eigen::RowVector4d K;
    Eigen::Matrix4d P;

    double m;
    double M;
    double length;
    double force;
    double max_force;
    double dt;
    double g;

    rclcpp::TimerBase::SharedPtr controller_update_timer;
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cart_pole_lqr_controller_cpp_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
