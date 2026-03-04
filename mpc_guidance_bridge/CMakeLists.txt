cmake_minimum_required(VERSION 3.10)
project(mpc_guidance_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(svea_vision_msgs REQUIRED)

include_directories(
  include
)

add_executable(mpc_guidance_bridge
  src/main.cpp
  src/ros2_receiver.cpp
)

ament_target_dependencies(
  mpc_guidance_bridge
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  svea_vision_msgs
)

install(
  TARGETS mpc_guidance_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
