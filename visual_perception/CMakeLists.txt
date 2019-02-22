cmake_minimum_required(VERSION 2.8.3)
project(visual_perception)

# Look for bfl (Bayesian Filtering Library)
find_package(PkgConfig REQUIRED)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  tf
  message_filters
  sensor_msgs
  nav_msgs
  actionlib_msgs
  trajectory_msgs
  control_msgs
  actionlib
  visualization_msgs
  pcl_ros
  pcl_conversions
  rviz_tools_py
  ar_track_alvar_msgs
  openpose_ros_wrapper_msgs
  octomap_ros
  octomap_msgs
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PCL 1.7 REQUIRED)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
   std_srvs
   actionlib_msgs
)


## Declare things to be passed to dependent projects
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS
    actionlib
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    actionlib_msgs
    visualization_msgs
    ar_track_alvar_msgs
    octomap_msgs 
LIBRARIES visual_perception
)

## Specify additional locations of header files
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(visual_perception 
            src/visible_region.cpp

)

## Declare a cpp executable
add_executable(visible_manager src/visible_region.cpp)
add_executable(people_marker src/people_marker.cpp)
add_executable(particle_artrack src/particle_artrack.cpp)
add_executable(octomap_checker src/octomap_checker.cpp)



## Add cmake target dependencies of the executable/library
add_dependencies(visible_manager visual_perception ${catkin_EXPORTED_TARGETS})
add_dependencies(people_marker ${catkin_EXPORTED_TARGETS})

add_dependencies(particle_artrack ${catkin_EXPORTED_TARGETS})
add_dependencies(octomap_checker ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(visible_manager visual_perception ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})
target_link_libraries(people_marker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${BFL_LIBRARIES})
target_link_libraries(particle_artrack ${catkin_LIBRARIES} ${Boost_LIBRARIES}
                                        ${BFL_LIBRARIES} ${PCL_LIBRARIES})

target_link_libraries(octomap_checker ${catkin_LIBRARIES} ${Boost_LIBRARIES}
                                        ${BFL_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS visible_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

