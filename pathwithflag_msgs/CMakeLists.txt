cmake_minimum_required(VERSION 2.8.3)
project(pathwithflag_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs nav_msgs)


 add_message_files(
   FILES
   PathWithFlag.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   nav_msgs
 )

catkin_package(

  CATKIN_DEPENDS message_runtime

)

