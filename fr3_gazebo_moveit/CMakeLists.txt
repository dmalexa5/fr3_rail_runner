cmake_minimum_required(VERSION 3.8)
project(fr3_gazebo_moveit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# add_executable(joint_publisher src/joint_publisher.cpp)
# ament_target_dependencies(joint_publisher rclcpp trajectory_msgs)

# install(
#   TARGETS
#   joint_publisher
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
