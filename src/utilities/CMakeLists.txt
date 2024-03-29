cmake_minimum_required(VERSION 2.8.3)
project(utilities)

add_compile_options(-std=c++17)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp 
  std_msgs 
  cv_bridge
  rosbag
  rosbag_storage
  topic_tools
  genmsg
  ros_type_introspection
  message_generation
  )

set(CV_MAJOR_VERSION=4.8.0)
set(OpenCV_DIR=/usr/local/lib/cmake/opencv4)
find_package(OpenCV 4.8.0  COMPONENTS
core
imgproc
imgcodecs
CONFIG
REQUIRED)



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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
file(GLOB MSG_FULL_FILENAMES ${PROJECT_SOURCE_DIR}/msg/*.msg)
set(MSG_FILES "")
foreach(f ${MSG_FULL_FILENAMES})
    GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
    list(APPEND MSG_FILES "${fname}.msg")
endforeach()

add_message_files(
  FILES
  ${MSG_FILES}
  # omnirobot.msg
  # marker.msg
  # stamped_markers.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  stop_execution.srv
  # Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES perception
    CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include 
	${catkin_INCLUDE_DIRS}		
	)
## Declare a C++ library
# add_library(perception
#   src/${PROJECT_NAME}/perception.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(perception ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable

file(GLOB EXECUTABLE_FILES ${PROJECT_SOURCE_DIR}/executables/*.cpp)
    # ${PROJECT_SOURCE_DIR}/src/prx/experiments/*.cpp)

foreach(f ${EXECUTABLE_FILES})
    MESSAGE("file: ${f}")
    GET_FILENAME_COMPONENT(fname ${f} NAME_WE )
    add_executable(${fname} ${f})
    add_dependencies(${fname}
                     ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                     ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${fname}
                          ${catkin_LIBRARIES}
                          ${OpenCV_LIBS}
                        )
endforeach()
# add_executable(video_gatherer_cpp src/vid_gather_cpp.cpp)
# add_executable(display_video_cpp src/display_video.cpp)
# add_executable(offline_rosbag_analysis_cpp src/offline_rosbag_analysis.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rosbag_record ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(video_gatherer_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(display_video_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(offline_rosbag_analysis_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# target_link_libraries(rosbag_record
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

# target_link_libraries(video_gatherer_cpp
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

# target_link_libraries(display_video_cpp
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

# target_link_libraries(offline_rosbag_analysis_cpp
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/test_video_resource.py scripts/vid_gather.py
#   scripts/test_video_resource.py scripts/rosbag_to_imgs.py
#   scripts/test_video_resource.py scripts/show_imgs.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS perception perception_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
