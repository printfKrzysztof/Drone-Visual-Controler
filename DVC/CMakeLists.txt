cmake_minimum_required(VERSION 3.0.2)
project(dvc)

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
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

## Generate services in the 'srv' folder


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include/DR include/MC  include/OGL include/TP  include
  LIBRARIES dvc
  CATKIN_DEPENDS roscpp mavros std_msgs geometry_msgs  message_runtime
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tree_seeker.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/tree_seeker_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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

add_executable(square src/DR/square.cpp)
target_link_libraries(square ${catkin_LIBRARIES})

add_executable(circle src/DR/circle.cpp)
target_link_libraries(circle ${catkin_LIBRARIES})

add_executable(randomly src/DR/randomly.cpp)
target_link_libraries(randomly ${catkin_LIBRARIES})

add_executable(subscriber src/DR/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})

add_executable(sub2 src/DR/sub2.cpp)
target_link_libraries(sub2 ${catkin_LIBRARIES})

add_executable(drone_ridder src/DR/drone_ridder.cpp include/DR/drone_ridder_functions.cpp)
target_link_libraries(drone_ridder ${catkin_LIBRARIES})

add_executable(test_DR_square src/DR/test_DR_square.cpp)
target_link_libraries(test_DR_square ${catkin_LIBRARIES})

add_executable(test_set_global_position_square src/DR/test_set_global_position_square.cpp)
target_link_libraries(test_set_global_position_square ${catkin_LIBRARIES})

add_executable(test_set_global_position_still src/DR/test_set_global_position_still.cpp)
target_link_libraries(test_set_global_position_still ${catkin_LIBRARIES})

add_executable(test_set_local_position src/DR/test_set_local_position.cpp)
target_link_libraries(test_set_local_position ${catkin_LIBRARIES})

add_executable(test_set_position_offset src/DR/test_set_position_offset.cpp)
target_link_libraries(test_set_position_offset ${catkin_LIBRARIES})

add_executable(test_set_heading src/DR/test_set_heading.cpp)
target_link_libraries(test_set_heading ${catkin_LIBRARIES})

add_executable(ball_droper src/BD/ball_droper.cpp)
target_link_libraries(ball_droper ${catkin_LIBRARIES})

add_executable(mission_commander_trees src/MC/mission_commander_trees.cpp include/MC/mission_commander_trees_functions.cpp)
target_link_libraries(mission_commander_trees ${catkin_LIBRARIES})

add_executable(mission_commander_3color src/MC/mission_commander_3colors.cpp include/MC/mission_commander_3colors_functions.cpp)
target_link_libraries(mission_commander_3color ${catkin_LIBRARIES})

add_executable(object_global_localizator src/OGL/object_global_localizator.cpp include/OGL/object_global_localizator_functions.cpp)
target_link_libraries(object_global_localizator ${catkin_LIBRARIES})

add_executable(trajectory_planer src/TP/trajectory_planer.cpp include/TP/trajectory_planer_functions.cpp include/TP/TreeObejctPosition.cpp)
target_link_libraries(trajectory_planer ${catkin_LIBRARIES})