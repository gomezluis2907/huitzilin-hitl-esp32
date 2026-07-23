cmake_minimum_required(VERSION 3.8)
project(huitzilin_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs)
find_package(actuator_msgs)

add_executable(huitzilin_serial src/huitzilin_serial.cpp)
ament_target_dependencies(huitzilin_serial rclcpp sensor_msgs tf2 tf2_geometry_msgs geometry_msgs)

install(TARGETS
  huitzilin_serial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
