cmake_minimum_required(VERSION 2.8.3)

project(volksbot_driver)

find_package(
             catkin REQUIRED
             COMPONENTS
                 roscpp
                 sensor_msgs
                 geometry_msgs
                 nav_msgs
                 tf
                 libepos2)

# find_package(
#             LibEPOS2 NO_MODULE REQUIRED)

catkin_package()

find_package(LibFTDI1 NO_MODULE REQUIRED)
include(${LIBFTDI_USE_FILE})

include_directories(${catkin_INCLUDE_DIRS} include/)

add_executable(volksbot src/volksbot_node.cpp src/volksbot.cpp)
target_link_libraries(volksbot 
    ${LIBEPOS2_LIBRARIES} 
    ${catkin_LIBRARIES}
    ${LIBFTDI_LIBARIES})
add_dependencies(volksbot ${catkin_EXPORTED_TARGETS})

install(TARGETS volksbot RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY meshes/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)
install(DIRECTORY urdf/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
