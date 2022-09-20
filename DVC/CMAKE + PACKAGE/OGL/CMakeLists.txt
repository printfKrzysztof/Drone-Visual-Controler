cmake_minimum_required(VERSION 3.0.2)
project(object_global_localizator)


find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  std_msgs
  darknet_ros_msgs
  object_global_localizator_msgs
)


catkin_package(
  INCLUDE_DIRS include

)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

add_executable(object_global_localizator src/object_global_localizator.cpp include/object_global_localizator_functions.cpp)
target_link_libraries(object_global_localizator ${catkin_LIBRARIES})