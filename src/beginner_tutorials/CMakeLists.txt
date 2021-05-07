cmake_minimum_required(VERSION 2.8.3)
SET(CMAKE_CXX_FLAGS "-O2 -std=c++11")
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge sensor_msgs genmsg dynamic_reconfigure)

find_package(OpenCV REQUIRED)
find_package(PythonLibs REQUIRED)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

##generate_dynamic_reconfigure_options(
  ##cfg/cc_freq.cfg
##)


## Declare a catkin package
catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
${PYTHON_INCLUDE_DIRS})

find_library(MOSEK_LIBRARY mosek64 /home/ubuntu/mosek/9.2/tools/platform/linux64x86/bin)

set(MOSEK_LIB_SRC
        src/BaseModel.cc
        src/BaseModel.os
        src/Debug.cc
        src/fusion.cc
        src/IntMap.cc
        src/mosektask.cc
        src/SolverInfo.cc
        src/StringBuffer.cc)

add_library(${PROJECT_NAME}
        src/dag.cpp
	src/dag_controller_be.cpp
	src/dag_multi_core_approx.cpp
	src/dag_mc.cpp
	${MOSEK_LIB_SRC})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher beginner_tutorials_generate_messages_cpp)

add_executable(old_publisher src/old_publisher.cpp)
target_link_libraries(old_publisher ${catkin_LIBRARIES})
add_dependencies(old_publisher beginner_tutorials_generate_messages_cpp)

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber beginner_tutorials_generate_messages_cpp)

add_executable(campreprocess src/cam_preprocess_node.cpp)
target_link_libraries(campreprocess ${catkin_LIBRARIES})
add_dependencies(campreprocess beginner_tutorials_generate_messages_cpp)

add_executable(objdetector src/obj_detector.cpp)
target_link_libraries(objdetector ${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${PYTHON_LIBRARIES})
add_dependencies(objdetector beginner_tutorials_generate_messages_cpp)

add_executable(objtracker src/obj_tracker.cpp)
target_link_libraries(objtracker ${catkin_LIBRARIES})
add_dependencies(objtracker beginner_tutorials_generate_messages_cpp)

add_executable(controller src/controller_node.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller beginner_tutorials_generate_messages_cpp)

add_executable(shimfreqnode src/shim_freq_node.cpp)
target_link_libraries(shimfreqnode ${catkin_LIBRARIES})
add_dependencies(shimfreqnode beginner_tutorials_generate_messages_cpp)

add_executable(shimstreamnode src/shim_stream_node.cpp)
target_link_libraries(shimstreamnode ${catkin_LIBRARIES})
add_dependencies(shimstreamnode beginner_tutorials_generate_messages_cpp)


add_executable(dag_controller src/dag_controller_node.cpp src/dag_controller_be.cpp src/dag_mc.cpp src/dag_multi_core_approx.cpp src/dag.cpp ${MOSEK_LIB_SRC})
target_link_libraries(dag_controller ${catkin_LIBRARIES} ${MOSEK_LIBRARY})
add_dependencies(dag_controller beginner_tutorials_generate_messages_cpp)
