#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <sys/sysinfo.h>
#include <vector>
#include <algorithm>
#include <time.h>
#include <bits/stdc++.h>
#include <sys/time.h>
#include <stdlib.h>

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>

#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

// This node is either TD or subscribes to triggers from Scheduler.
// Each time it runs, it does preprocessing [~25ms sieve] then picks an img from a dataset and publishes it. [to be used by the yolo - objDetection node]

void calc_primes(int64_t limit, bool duh)
{
	// ROS_ERROR("calc_primes was CALLED with %i", limit);
	int i, num = 1, primes = 0;
	while (num <= limit) {
		i = 2;
		while (i <= num) {
			if(num % i == 0)
				break;
			i++;
		}
		if (i == num)
			primes++;
		num++;
	}
	if (duh)
		ROS_INFO("Found %i primes", primes);
};

double get_time_now()
{
	struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double time_now = ts.tv_sec + 1e-9*ts.tv_nsec;

	return time_now;
}

class CamPreProcess
{
	public:
		ros::NodeHandle nh;

		std::string data_path, fname_pre;
		
		// if TD: publish at pub_freq, else listen to socket for triggers.
		std::string use_td_;
		std::string pub_topic_;
		double pub_freq_;

		ros::Publisher pub_cam, pp_exec_end_pub, pp_thr_info_pub;
		int total_pub_count;

		// for publishing based on pub_freq_:
		ros::WallTimer timer_thread;

		// for listening to triggers:
		int client_sock_fd;
	        boost::thread sock_recv_thread; // to indefinitely listen on the socket fd.
		void socket_recv();

		int img_ct = 0;
		int total_imgs = 0;

		void recvTimerEvent(const ros::WallTimerEvent& event);
		void do_work();

		CamPreProcess(std::string use_td, double freq, std::string pub_topic, int total_img, std::string folder, std::string fname_prefix);
};

CamPreProcess::CamPreProcess(std::string use_td, double freq, std::string pub_topic, int total_img, std::string folder, std::string fname_prefix)
{
	ROS_ERROR("CamPreProcess: Recvd all params %s, %f, %s, %i, %s, %s", use_td.c_str(), freq, pub_topic.c_str(), total_img, folder.c_str(), fname_prefix.c_str());
	use_td_ = use_td;
	pub_freq_ = freq;

	pub_topic_ = pub_topic;

	total_imgs = total_img;

	data_path = folder;
	fname_pre = fname_prefix;

	pub_cam = nh.advertise<sensor_msgs::Image> (pub_topic_, 1, true);

	pp_exec_end_pub = nh.advertise<std_msgs::Header>("/robot_0/exec_end_ppcam", 1, false);
	pp_thr_info_pub = nh.advertise<std_msgs::Header>("/robot_0/exec_start_ppcam", 1, false);

	total_pub_count = 0;

	if (use_td.find("yes") != std::string::npos)
	{
		timer_thread = nh.createWallTimer(ros::WallDuration(1.0/pub_freq_), &CamPreProcess::recvTimerEvent, this);
		ROS_ERROR("USE TD was TRUE!!!! Made a timer thread! for time %f", (1.0/pub_freq_) );
	}
	else
	{
		errno = 0;
		client_sock_fd = socket(AF_INET, SOCK_STREAM, 0);
		ROS_ERROR("CamPreProcessNode:: SOCKET: client_sock_fd is %i !", client_sock_fd);
		struct sockaddr_in serv_addr;
	        serv_addr.sin_family = AF_INET;
		serv_addr.sin_port = htons(5327);
		
		int pton_ret = inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
		ROS_ERROR("CamPreProcessNode:: SOCKET: in inet_pton %i", pton_ret);

		int con_ret = connect(client_sock_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
		ROS_ERROR("CamPreProcessNode:: SOCKET: Done with connecting, retval: %i, errno: %i, err str: %s !!!", con_ret, errno, std::strerror(errno) );

		sock_recv_thread = boost::thread(&CamPreProcess::socket_recv, this);
	}
}

void CamPreProcess::socket_recv()
{
	std::string s = "ppcam\n";
	char smsg[1+s.length()];
	strcpy(smsg, s.c_str());
	send(client_sock_fd, smsg, strlen(smsg), 0);

	int ret = 7;
        struct sched_param sp = { .sched_priority = 3,};
	ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);
	ROS_ERROR("CamPreProcessNode Socket_recv thread set priority to 3, retval: %i, tid: %i", ret, ::gettid());

	int read_size;
        char msg [2048];
	
	while ( (read_size = recv(client_sock_fd, msg, 2048, 0) ) > 0 )
	{
		std::string s_msg = msg;
		memset(msg, 0, 2048);
		do_work();
	}
	if(read_size == 0)
		ROS_ERROR("CamPreProcessNode::socket_recv THREAD: Client disconnected!!");
	else if(read_size == -1)
		ROS_ERROR("CamPreProcessNode::socket_recv THREAD: Client read error!!");
}

void CamPreProcess::recvTimerEvent(const ros::WallTimerEvent& event)
{
	do_work();
}

void CamPreProcess::do_work()
{
	struct timespec exec_start, exec_end;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &exec_start);

	if (total_pub_count < 50)
	{
		std_msgs::Header hdr, hdre;
		std::stringstream ss_e1, ss_e2;
		if (total_pub_count%2 == 1)
		{

			ss_e1 << ::getpid() << " ppcam " << ::gettid();
			hdr.frame_id = ss_e1.str();
			printf("CAMPP TID: %i, PID : %i hdr msg: %s", ::gettid(), ::getpid(), hdr.frame_id.c_str());
			pp_thr_info_pub.publish(hdr);

		}
		else
		{
			ss_e2 << ::getpid() << " ppcam_extra " << nh.getPMTId();
			hdre.frame_id = ss_e2.str();
			printf("CAMPP extra tid %s", hdre.frame_id.c_str());
			pp_thr_info_pub.publish(hdre);
		}
			
	}

	// sieve
	double tstart = get_time_now(); // ros::Time::now();
	calc_primes(10000, false);
	// publish next img from dataset
	cv::Mat img = cv::imread(data_path+"/"+fname_pre+ std::to_string(img_ct)+".jpg", CV_LOAD_IMAGE_COLOR);
	ROS_INFO("reading file %i sz: %i x %i", img_ct, img.rows, img.cols);
	if (img.empty())
		printf("DAMN!!! Could not open/find the img for file %s %s %i", data_path.c_str(), fname_pre.c_str(), img_ct);
	else
	{
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
		msg->header.frame_id = fname_pre+ std::to_string(img_ct);
		msg->header.stamp = ros::Time(tstart);
		pub_cam.publish(msg);
	}
	img_ct = (img_ct+1)%total_imgs;
	total_pub_count += 1;

	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &exec_end);
        std_msgs::Header pp_exec_time;
	pp_exec_time.frame_id = std::to_string( (exec_end.tv_sec + 1e-9*exec_end.tv_nsec ) - ( exec_start.tv_sec + 1e-9*exec_start.tv_nsec ) ) + " ppcam";
        pp_exec_end_pub.publish(pp_exec_time);
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "cam_preprocess_node");
	ROS_WARN("CAMPREprocess node, beginning. args: %s, %s, %s, %s, %s, %s", argv[1], argv[2], argv[3], argv[4], argv[5], argv[6]);
	CamPreProcess cpp (argv[1], std::atof(argv[2]), argv[3], std::atoi(argv[4]), argv[5], argv[6] );
	ros::spin();
	return 0;
}
