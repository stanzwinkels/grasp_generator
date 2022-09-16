cmake_minimum_required(VERSION 2.8.3)
project(grasp_generator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules eigen_conversions message_generation roscpp rospy sensor_msgs std_msgs rosprolog roslib actionlib actionlib_msgs aruco_msgs geometry_msgs rospy)

catkin_python_setup()

# PCL
#find_package(PCL 1.9 REQUIRED)
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIB)
  message(STATUS "Library GPD found in ${GPD_LIB}")
else()
  message(FATAL_ERROR "Library GPD not found")
endif()
include_directories(${GPD_LIB_INCLUDE_DIR})
message(STATUS "gpd_include_dir: ${GPD_LIB_INCLUDE_DIR}")

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++14")



## Generate messages in the 'msg' folder
add_message_files(FILES  GraspConfig.msg GraspConfigList.msg)

## Generate services in the 'srv' folder
add_service_files(FILES 
                  DetectGrasps.srv
                  PoseAruco.srv
                  QuadricDetect.srv
                  )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Grasp.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs actionlib actionlib_msgs aruco_msgs)

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
INCLUDE_DIRS 
#CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs #std_msgs actionlib_msgs aruco_msgs
#DEPENDS PCL
)

###########
## Build ##
###########
find_package(PkgConfig)
pkg_check_modules(SWIPL REQUIRED swipl)
include_directories(${SWIPL_INCLUDE_DIRS})
link_directories(${SWIPL_LIBRARY_DIRS})



catkin_install_python(PROGRAMS scripts/grasp_generator/server_clients/detect_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}_grasp_messages src/grasp_generator/grasp_messages.cpp)
add_dependencies(${PROJECT_NAME}_grasp_messages ${catkin_EXPORTED_TARGETS})
add_library(${PROJECT_NAME}_grasp_plotter src/grasp_generator/grasp_plotter.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_detect_server src/grasp_generator/detect_server.cpp)


## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_detect_server PROPERTIES OUTPUT_NAME detect_server PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_detect_server
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}                      
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_messages
                      ${GPD_LIB}
                      ${catkin_LIBRARIES})
