cmake_minimum_required(VERSION 3.8)
project(opencv_masking)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(masking_node src/masking_node.cpp)
ament_target_dependencies(masking_node 
  rclcpp sensor_msgs std_msgs cv_bridge image_transport OpenCV)

install(TARGETS masking_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
