cmake_minimum_required(VERSION 3.8)
project(pid_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pid_controller_cpp_node src/pid_controller_cpp_node.cpp)

ament_target_dependencies(
  pid_controller_cpp_node
  rclcpp
)

target_link_libraries(
    pid_controller_cpp_node
    Eigen3::Eigen
)

install(TARGETS
  pid_controller_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
