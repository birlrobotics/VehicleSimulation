cmake_minimum_required(VERSION 2.8.3)
project(ground_car)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)
find_package(ignition-msgs1 REQUIRED)

catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES gazebo_radar_plugin
 CATKIN_DEPENDS gazebo_ros
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})

add_library(gazebo_ros_block_laser SHARED plugins/gazebo_ros_block_laser.cpp)
target_link_libraries(gazebo_ros_block_laser ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})


install(TARGETS PriusHybridPlugin gazebo_ros_block_laser
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

