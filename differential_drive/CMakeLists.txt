cmake_minimum_required(VERSION 3.0.2)
project(differential_drive)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -li2c)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  diagnostic_msgs
  tf
)


catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bno055_i2c_node src/bno055_i2c_driver.cpp src/bno055_i2c_node.cpp src/watchdog.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(bno055_i2c_node ${catkin_LIBRARIES})

install(TARGETS bno055_i2c_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

