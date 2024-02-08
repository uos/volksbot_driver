cmake_minimum_required(VERSION 3.8)
project(volksbot_driver)

# Configure C++ compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(epos2_motor_controller REQUIRED)

# Add executable and link external dependencies
add_executable(volksbot src/volksbot_node.cpp)
ament_target_dependencies(volksbot geometry_msgs nav_msgs sensor_msgs tf2_ros epos2_motor_controller)
 
target_include_directories(volksbot
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Install the launch directory and urdfs
install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Install binaries
install(TARGETS
  volksbot
  DESTINATION lib/${PROJECT_NAME})

ament_package()
