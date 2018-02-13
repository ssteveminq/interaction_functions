cmake_minimum_required(VERSION 2.8.3)
project(gaze_service)	

SET(SOURCES src/gaze_service_node.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
)

#find_package(Eigen3 REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(PCL 1.7 REQUIRED)

add_service_files(
   FILES
   gaze_target.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   std_srvs
)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES humantracking
#  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
  	roscpp
  	geometry_msgs
  	sensor_msgs
    std_msgs
  	message_runtime
    tf
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(gaze_service_test src/gaze_test.cpp ${SOURCES})
target_link_libraries(gaze_service_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gaze_service_test ${catkin_EXPORTED_TARGETS})
