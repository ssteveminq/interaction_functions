cmake_minimum_required(VERSION 2.8.3)
project(gaze_service)	

SET(SOURCES src/gaze_service_node.cpp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  pcl_ros
  pcl_conversions
  actionlib
  actionlib_msgs
  message_generation
)

#find_package(Eigen3 REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_service_files(
   FILES
   gaze_target.srv
)
add_action_files(
   DIRECTORY action 
   FILES Gazing.action
   FindPerson.action
   GetHeadAngle.action
    )

generate_messages(
   DEPENDENCIES
   std_msgs
   std_srvs
   geometry_msgs
   actionlib_msgs
)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)
include_directories(src)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  DEPENDS Boost
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  message_runtime
  tf
  actionlib_msgs
  geometry_msgs
  actionlib
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(gaze_service_test src/gaze_service_test.cpp ${SOURCES})
target_link_libraries(gaze_service_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gaze_service_test ${catkin_EXPORTED_TARGETS})

add_executable(gaze_action_server src/gaze_action_server.cpp ${SOURCES})
target_link_libraries(gaze_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gaze_action_server ${catkin_EXPORTED_TARGETS})

add_executable(gaze_action_client src/gaze_action_client.cpp ${SOURCES})
target_link_libraries(gaze_action_client ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(gaze_action_client ${catkin_EXPORTED_TARGETS})

add_executable(clicked_gaze_test src/clicked_gaze_test.cpp ${SOURCES})
target_link_libraries(clicked_gaze_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(clicked_gaze_test ${catkin_EXPORTED_TARGETS})

add_executable(ar_gaze_test src/ar_gaze_test.cpp ${SOURCES})
target_link_libraries(ar_gaze_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ar_gaze_test ${catkin_EXPORTED_TARGETS})

add_executable(closest_human_gaze src/closest_human_gaze.cpp ${SOURCES})
target_link_libraries(closest_human_gaze ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(closest_human_gaze ${catkin_EXPORTED_TARGETS})

add_executable(find_people_server src/find_people_server.cpp ${SOURCES})
target_link_libraries(find_people_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(find_people_server ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(getheadangle_server src/getheadangle_server.cpp ${SOURCES})
target_link_libraries(getheadangle_server ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(getheadangle_server ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(person_tracking src/person_tracking_gaze.cpp ${SOURCES})
target_link_libraries(person_tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(person_tracking ${catkin_EXPORTED_TARGETS})

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

