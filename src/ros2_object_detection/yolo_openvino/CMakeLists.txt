cmake_minimum_required(VERSION 3.8)
project(yolo_openvino)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)  # ← TAMBAHKAN INI

# Find OpenVINO
find_package(OpenVINO REQUIRED)

# Build YOLO node
add_executable(yolo_node src/yolo_node.cpp)
ament_target_dependencies(yolo_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  cv_bridge 
  image_transport
  OpenCV
  ament_index_cpp  # ← TAMBAHKAN INI
)
target_link_libraries(yolo_node openvino::runtime)

# Install
install(TARGETS yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
