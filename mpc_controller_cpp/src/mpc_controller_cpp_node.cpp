#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>


class mpc_controller_cpp_node : public rclcpp::Node{
public:
    mpc_controller_cpp_node() : Node("mpc_controller_cpp_node"){
        state <<
            0.0,
            0,0;

        target_state <<
            5.0,
            0.0;

        A <<
            0.0, 1.0,
            0.0, 0.0;

        B <<
            0.0,
            1.0;

        Q <<
            10.0, 0.0,
            0.0, 1.0;
        
        R = 0.1;

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &mpc_controller_cpp_node::timerCallback,
                this
            )
        );
    }

private:

    double compute_cost(const std::vector<double>& u_sequence){
        
    }

    Eigen::Vector2d state;
    Eigen::Vector2d target_state;
    Eigen::Matrix2d A;
    Eigen::Vector2d B;
    Eigen::Matrix2d Q;

    double R;
    double dt;
    int horizon;

    rclcpp::TimerBase::SharedPtr timer;
}