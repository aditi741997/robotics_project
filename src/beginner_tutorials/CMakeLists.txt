cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-std=c++11")
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge sensor_msgs genmsg dynamic_reconfigure)

find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

generate_dynamic_reconfigure_options(
cfg/BinSize.cfg)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS})

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
target_link_libraries(objdetector ${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${PYTHON_LIBRARIES})
add_dependencies(objdetector beginner_tutorials_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_executable(objtracker src/obj_tracker.cpp)
target_link_libraries(objtracker ${catkin_LIBRARIES})
add_dependencies(objtracker beginner_tutorials_generate_messages_cpp)

add_executable(controller src/controller_node.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller beginner_tutorials_generate_messages_cpp)

add_executable(newcontroller src/new_controller_node.cpp)
target_link_libraries(newcontroller ${catkin_LIBRARIES})
add_dependencies(newcontroller beginner_tutorials_generate_messages_cpp)

add_executable(dsubscriber src/dummy_subscriber.cpp)
target_link_libraries(dsubscriber ${catkin_LIBRARIES})
add_dependencies(dsubscriber beginner_tutorials_generate_messages_cpp)

add_executable(localplanner src/local_planner.cpp)
target_link_libraries(localplanner ${catkin_LIBRARIES})
add_dependencies(localplanner beginner_tutorials_generate_messages_cpp)

add_executable(globalcmptd src/globalcmp_td.cpp)
target_link_libraries(globalcmptd ${catkin_LIBRARIES})
add_dependencies(globalcmptd beginner_tutorials_generate_messages_cpp)
