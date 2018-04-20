cmake_minimum_required(VERSION 2.8.3)

set (fcl_DIR "/usr/local/lib/cmake/fcl")
set (octomap_DIR "/home/nrsl/MyApps/octomap-1.7.2/lib/cmake/octomap")

project(path_planning)
set (CMAKE_CXX_STANDARD 11)

find_package(octomap 1.7 REQUIRED)
find_package(fcl REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp std_msgs
#  DEPENDS octomap PCL
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${fcl_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES} ${fcl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/path_planning.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${fcl_LIBRARIES}
   ${OMPL_LIBRARIES}
   fcl
)
