cmake_minimum_required(VERSION 3.8)
project(serial_to_cmdvel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller REQUIRED)

add_executable(sercom_to_cmdvel src/sercom_to_cmdvel.cpp)
ament_target_dependencies(sercom_to_cmdvel rclcpp controller)
install(TARGETS
  sercom_to_cmdvel
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
