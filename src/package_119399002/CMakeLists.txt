cmake_minimum_required(VERSION 3.8)
project(package_119399002)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(moveit_core)
find_package(moveit_ros_move_group)
find_package(moveit_ros_planning)
find_package(moveit_ros_planning_interface)
find_package(tf2_geometry_msgs)
find_package(ament_cmake REQUIRED)

add_executable(picknplace src/picknplace_node.cpp)
ament_target_dependencies(picknplace
  rclcpp
  moveit_core
  moveit_ros_move_group
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2_geometry_msgs)

target_compile_features(picknplace PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS picknplace
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
