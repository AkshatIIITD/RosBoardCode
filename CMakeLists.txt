cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  roscpp
)

LINK_DIRECTORIES(
  /usr/local/lib
)

catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  /usr/local/lib
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(tick_node src/tick_pub.cpp)
target_link_libraries(tick_node libpigpiod_if2.so ${catkin_LIBRARIES})

add_executable(odom_node src/odom_publisher.cpp)
target_link_libraries(odom_node ${catkin_LIBRARIES})

add_executable(simple_control_node src/simple_control.cpp)
target_link_libraries(simple_control_node libpigpiod_if2.so ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  src/tickpy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
