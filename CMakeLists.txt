cmake_minimum_required(VERSION 3.8)
project(trimble_drive)

# Compiler flags for GCC and Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}_node
    src/trimble_drive.cpp
)

# Link against dependencies
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs sensor_msgs)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if any)
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()

