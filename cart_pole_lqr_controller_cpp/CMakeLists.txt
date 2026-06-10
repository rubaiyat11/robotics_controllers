cmake_minimum_required(VERSION 3.8)
project(cart_pole_lqr_controller_cpp)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(cart_pole_lqr_controller_cpp_node
  src/cart_pole_lqr_controller_cpp_node.cpp
)


target_include_directories(cart_pole_lqr_controller_cpp_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(cart_pole_lqr_controller_cpp_node
  rclcpp
)

install(TARGETS
  cart_pole_lqr_controller_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()