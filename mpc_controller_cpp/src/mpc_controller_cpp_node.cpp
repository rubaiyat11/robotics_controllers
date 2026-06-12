#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>


class mpc_controller_cpp_node : public rclcpp::Node{
public:
    mpc_controller_cpp_node() : Node("mpc_controller_cpp_node"){
        state <<
            0.0,
            0.0;

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

        dt = 0.1;
        horizon = 10;

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(
                &mpc_controller_cpp_node::timerCallback,
                this
            )
        );
    }

private:

    void timerCallback(){
        double force = find_best_force();

        Eigen::Vector2d state_dot = 
            A * state
            + B * force;

        state += state_dot * dt;

        RCLCPP_INFO(
            this->get_logger(),
            "Position=%.3f Velocity=%.3f Force=%.3f",
            state(0),
            state(1),
            force
        );
    }

    double compute_cost(const std::vector<double>& u_sequence){
        Eigen::Vector2d predicted_state = state;

        double total_cost = 0.0;

        for(double force : u_sequence){
            Eigen::Vector2d state_dot =
                A * predicted_state
                + B * force;
                
            predicted_state += state_dot * dt;

            Eigen::Vector2d error =
                predicted_state - target_state;

            double state_cost =
                (error.transpose() * Q * error)(0,0);
            
            double control_cost =
                R * force * force;

            total_cost +=
                state_cost + control_cost;
            
        }

        return total_cost;
    }

    double find_best_force(){
        double best_force = 0.0;
        double best_cost =
            std::numeric_limits<double>::max();

        for(double test_force = -20.0;
            test_force <= 20.0;
            test_force += 0.5){

                std::vector<double> u_sequence(
                    horizon,
                    test_force
                );

                double cost = compute_cost(u_sequence);

                if(cost < best_cost){
                    best_cost = cost;
                    best_force = test_force;
                }
                
        }

        return best_force;
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
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<mpc_controller_cpp_node>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}