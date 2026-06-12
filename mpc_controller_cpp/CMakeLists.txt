cmake_minimum_required(VERSION 3.8)
project(mpc_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(
  mpc_controller_cpp_node
  src/mpc_controller_cpp_node.cpp
)

target_include_directories(
  mpc_controller_cpp_node
  PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(
  mpc_controller_cpp_node
  rclcpp
)

install(
  TARGETS
  mpc_controller_cpp_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
