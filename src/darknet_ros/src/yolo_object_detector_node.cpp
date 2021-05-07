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

void pub_extra_tids(ros::Publisher* yolo_pub)
{
	ros::NodeHandle nh;
	for (int i = 0; i < 3; i++)
	{
		// PMT:
		std::stringstream ss_e, ss_e1;
		ss_e << ::getpid() << " yolo_extra " << nh.getPMTId();
		std_msgs::Header hdre, hdre1;
		hdre.frame_id = ss_e.str();
		yolo_pub->publish(hdre);
		ros::Duration(0.2).sleep();
		// InternalCBQT:
		ss_e1 << ::getpid() << " yolo_extra " << nh.getInternalCBQTId();
		hdre1.frame_id = ss_e1.str();
		yolo_pub->publish(hdre1);
		ros::Duration(0.2).sleep();
		printf("Publishing yolo extra tids: PMTId: %i, InternalCBQTId: %i", nh.getPMTId(), nh.getInternalCBQTId());
	}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");

  ros::Publisher yolo_tid_pub = nodeHandle.advertise<std_msgs::Header> ("/robot_0/exec_start_yolo", 9, true);
  
  std_msgs::Header hdr;
  std::stringstream ss_e;
  ss_e << ::getpid() << " yolo " << ::gettid();
  hdr.frame_id = ss_e.str();
  ROS_INFO("YOLO b4 init: sending tid: %i",  ::gettid());
  yolo_tid_pub.publish(hdr);

  std::thread pub_tid_thr(pub_extra_tids,&yolo_tid_pub);

  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);
  ros::spin();
  return 0;
}
