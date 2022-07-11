cmake_minimum_required(VERSION 3.0.2)
project(hyrecro_pcl)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  gazebo_ros
)
find_package(gazebo REQUIRED)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES hyrecro_pcl
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hyrecro_pcl.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pcl_hyrecro_node.cpp)


add_executable(offline_mapping          src/offline_mapping.cpp)
# add_executable(online_mapping           src/online_mapping.cpp)
add_executable(show_planes          src/show_planes.cpp)


# add_executable(test_pcl               src/test_pcl.cpp)
# add_executable(extract_indices        src/extract_indices.cpp)
# add_executable(save_PCD               src/save_PCD.cpp)
# add_executable(filterPCD              src/filterPCD.cpp)
# add_executable(getIndx                src/getIndx.cpp)

# add_executable(mapping                src/mapping.cpp)
# add_executable(mappingrealsense       src/mappingrealsense.cpp)


# add_executable(topic_cloud_viewer     src/topic_cloud_viewer.cpp)
# add_executable(my_rosbag_to_pcd       src/my_rosbag_to_pcd.cpp)
# add_executable(view_cloud             src/view_cloud.cpp)



##############
# DEPRECATED #
############## 
# add_executable(plane2planeReg src/plane2planeReg.cpp)
# add_executable(segFromNormals src/segFromNormals.cpp)
# add_executable(save_PCD_each_sec src/save_PCD_each_sec.cpp)
# add_executable(pairwise_incremental src/pairwise_incremental.cpp)
# add_executable(regFromPCD src/regFromPCD.cpp)
# add_executable(registration_from_plane_indices src/registration_from_plane_indices.cpp)
# add_executable(extract_normals src/extract_normals.cpp)
# add_executable(registration src/registration.cpp)
# add_executable(plane_segmentation src/plane_segmentation.cpp)


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

target_link_libraries(offline_mapping         pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
target_link_libraries(show_planes             pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})



#target_link_libraries(save_PCD                    ${GAZEBO_LIBRARIES} ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(save_PCD                                  ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(test_pcl              pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(extract_indices       pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(filterPCD             pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(topic_cloud_viewer    pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(getIndx               pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(mapping               pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(my_rosbag_to_pcd                          ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(mappingrealsense      pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})
# target_link_libraries(view_cloud            pcl_visualization   ${PCL_LIBRARIES}  ${catkin_LIBRARIES})



##############
# DEPRECATED #
##############

# target_link_libraries(pairwise_incremental pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(plane_segmentation pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(plane2planeReg pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(regFromPCD pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(registration_from_plane_indices pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(registration pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(segFromNormals pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(save_PCD_each_sec pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(extract_normals pcl_visualization  ${PCL_LIBRARIES} ${catkin_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
# install(DIRECTORY include/${PROJECT_NAME}/
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl_hyrecro.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
