cmake_minimum_required(VERSION 2.8.3)

project(volksbot_driver)

find_package(
             catkin REQUIRED 
             COMPONENTS
                 roscpp
                 sensor_msgs
                 geometry_msgs
                 nav_msgs
                 tf)

find_package(
            LibEPOS2 NO_MODULE REQUIRED)

catkin_package(
    CATKIN_DEPENDS
             roscpp
             sensor_msgs
             geometry_msgs
             nav_msgs
             tf
             libepos2-dev
    INCLUDE_DIRS include)
             
include(${LIBEPOS2_USE_FILE})

include_directories(${catkin_INCLUDE_DIRS} include/)

add_executable(volksbot src/volksbot_node.cpp src/volksbot.cpp)
target_link_libraries(volksbot ${LIBEPOS2_LIBRARIES} ${catkin_LIBRARIES})
