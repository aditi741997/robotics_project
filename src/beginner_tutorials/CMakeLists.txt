cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge sensor_msgs genmsg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher beginner_tutorials_generate_messages_cpp)

add_executable(old_publisher src/old_publisher.cpp)
target_link_libraries(old_publisher ${catkin_LIBRARIES})
add_dependencies(old_publisher beginner_tutorials_generate_messages_cpp)

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber beginner_tutorials_generate_messages_cpp)

add_executable(objdetector src/obj_detector.cpp)
target_link_libraries(objdetector ${catkin_LIBRARIES})
add_dependencies(objdetector beginner_tutorials_generate_messages_cpp)

add_executable(objtracker src/obj_tracker.cpp)
target_link_libraries(objtracker ${catkin_LIBRARIES})
add_dependencies(objtracker beginner_tutorials_generate_messages_cpp)
