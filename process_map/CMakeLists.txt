cmake_minimum_required(VERSION 2.8.3)
project(process_map)
set (CMAKE_CXX_STANDARD 11)
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set (fcl_DIR "/usr/local/lib/cmake/fcl")
set (octomap_DIR "/usr/local/share/octomap")
find_package(octomap 1.7 REQUIRED)
find_package(fcl REQUIRED)
find_package(ompl REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  pathwithflag_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES re_planning
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp std_msgs pathwithflag_msgs
) 

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES} ${fcl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

#add_definitions(${PCL_DEFINITIONS})


#add_executable(${PROJECT_NAME}_node src/path_planning.cpp)
#target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
#   ${fcl_LIBRARIES}
#   ${OMPL_LIBRARIES}
#   fcl
#)
add_executable(pub_octomap src/pub_octomap.cpp)

target_link_libraries(pub_octomap
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
)

add_executable(mark src/mark.cpp)

target_link_libraries(mark
   ${catkin_LIBRARIES}
)

add_executable(realtime_octomap src/realtime_octomap.cpp)

target_link_libraries(realtime_octomap
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${PCL_LIBRARIES}
   orocos-kdl
)

#add_executable(collision_detection src/collision_detection.cpp)

#target_link_libraries(collision_detection
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
#   ${fcl_LIBRARIES}
#   ${OMPL_LIBRARIES}
#   fcl
#)

add_executable(re_planning src/re_planning.cpp)

target_link_libraries(re_planning
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${fcl_LIBRARIES}
   ${OMPL_LIBRARIES}
   fcl
   orocos-kdl
)

