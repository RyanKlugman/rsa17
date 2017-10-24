cmake_minimum_required(VERSION 2.8.3)
project(rsa17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs 
    nav_msgs
    dynamic_reconfigure
    message_generation
    genmsg 
    actionlib_msgs
    actionlib
    crosbot 
    crosbot_map
    crosbot_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
     ExplorerFeedback.msg
)

## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES
    GetPath.srv
    SetExplorerMode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs  # Or other packages containing msgs
    geometry_msgs
    nav_msgs
    actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/CrosbotExploreReconfig.cfg
    cfg/CrosbotAStarReconfig.cfg
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rsa17
   CATKIN_DEPENDS
     roscpp 
     geometry_msgs 
     nav_msgs
     actionlib_msgs
     crosbot 
     crosbot_map
     crosbot_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(rsa17
  src/basicClient/actionParameters.cpp
  src/basicClient/basicClient.cpp
  src/astar.cpp
  src/explorer.cpp
  src/explorerROS.cpp
)

## Declare a cpp executable
add_executable(astar_explorer src/nodes/astar_explorer_node.cpp)
add_executable(astar_planner src/nodes/astar_planner_node.cpp)
add_executable(basic_client src/nodes/basicActionClient_node.cpp)
add_executable(explorer src/nodes/explorer_node.cpp)

## Add dependencies to the executable
add_dependencies(rsa17
    ${PROJECT_NAME}_gencpp
    ${PROJECT_NAME}_gencfg
    ${PROJECT_NAME}_generate_messages
)
add_dependencies(astar_explorer 
    ${PROJECT_NAME} 
    rsa17
)
add_dependencies(astar_planner
    ${PROJECT_NAME}
    rsa17
)
add_dependencies(explorer 
    ${PROJECT_NAME}
    rsa17
)
add_dependencies(basic_client 
    ${PROJECT_NAME} 
    rsa17
)

## Specify libraries to link a library or executable target against
target_link_libraries(rsa17
  ${catkin_LIBRARIES} crosbot crosbot_map
)

target_link_libraries(astar_explorer
  ${catkin_LIBRARIES} rsa17
)
target_link_libraries(astar_planner
  ${catkin_LIBRARIES} rsa17
)
target_link_libraries(explorer
  ${catkin_LIBRARIES} rsa17
)
target_link_libraries(basic_client
  ${catkin_LIBRARIES} rsa17
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS rsa17 astar_planner astar_explorer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rsa17.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
