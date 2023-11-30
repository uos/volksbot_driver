cmake_minimum_required(VERSION 3.2)

project(volksbot_driver VERSION 1.0.2)

find_package(catkin REQUIRED 
    COMPONENTS
        roscpp
        sensor_msgs
        geometry_msgs
        nav_msgs
        tf
        epos2_motor_controller)

# find_package(
#             LibEPOS2 NO_MODULE REQUIRED)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES volksbot_driver
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs tf epos2_motor_controller
    #  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} include/)

add_executable(volksbot 
    src/volksbot_node.cpp 
    src/volksbot.cpp)
target_link_libraries(volksbot 
    ${catkin_LIBRARIES})
add_dependencies(volksbot 
    ${catkin_EXPORTED_TARGETS})

install(TARGETS volksbot RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
