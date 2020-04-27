cmake_minimum_required(VERSION 2.8.3)

project(rbx1_vision)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

add_message_files(
  FILES
  roi_time.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
    CATKIN_DEPENDS rospy
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)