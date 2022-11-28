cmake_minimum_required(VERSION 3.0.2)
project(dvc)

set(Eigen3_DIR /lib/cmake/eigen3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  std_msgs
  darknet_ros_msgs
  iq_sim
  mavros_msgs
  cv_bridge
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  message_generation
  dvc_msgs
  Eigen3 REQUIRED
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dvc
  CATKIN_DEPENDS roscpp mavros std_msgs geometry_msgs message_runtime
  #DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
         scripts/MC/image_saver_node.py
         scripts/MC/test.py
         scripts/MC/insert_data_to_db_trees.py
         scripts/MC/insert_data_to_db_colors.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY /${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_tree_seeker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(prey_control src/prey_control.cpp)
target_link_libraries(prey_control ${catkin_LIBRARIES})

add_executable(predator_main_controller src/predator_main_controller.cpp)
target_link_libraries(predator_main_controller ${catkin_LIBRARIES})

add_executable(read_yolo_data src/read_yolo_data.cpp)
target_link_libraries(read_yolo_data ${catkin_LIBRARIES})

add_executable(learning_session src/learning_session.cpp)
target_link_libraries(learning_session ${catkin_LIBRARIES})

add_executable(pilot src/pilot.cpp)
target_link_libraries(pilot ${catkin_LIBRARIES})

add_executable(kalman src/kalman.cpp src/kalman-test.cpp)
target_link_libraries(kalman ${catkin_LIBRARIES})