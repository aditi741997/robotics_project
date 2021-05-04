/*
 * yolo_object_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>

#include <unistd.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");

  ros::Publisher yolo_tid_pub = nodeHandle.advertise<std_msgs::Header> ("/robot_0/exec_start_yolo", 1, true);
  std_msgs::Header hdr;
  std::stringstream ss_e;
  ss_e << ::getpid() << " yolo " << ::gettid();
  hdr.frame_id = ss_e.str();
  ROS_INFO("YOLO b4 init: sending tid: %i",  ::gettid());
  yolo_tid_pub.publish(hdr);

  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);
  ros::spin();
  return 0;
}
