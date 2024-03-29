#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include "sensor_msgs/CameraInfo.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>
#include <cstdlib>
#include <fstream>

class ObjTracker
{
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber roi_sub;
    std::string sub_topic = "roi";
    std::string pub_topic = "cmd_vel";
    std::string rtc_pub_topic = "/camera/rtc_event";
    float max_rotation_speed, min_rotation_speed, gain, x_threshold;
    int img_width, img_height; // 640, 480

    geometry_msgs::Twist move_cmd;

    int cb_count = 0;
    double cb_time_sum = 0.0;
    std::vector<double> cb_time_arr;

    double metric_sum = 0.0;
    std::vector<double> metric_arr;    

    double metric1_sum = 0.0;
    std::vector<double> metric1_arr;    
    
    double rxn_time_sum = 0.0;
    std::vector<double> rxn_time_arr;
    double last_in_time = 0.0;

    std::vector<double> vel_arr;

    double latency_sum = 0.0;
    std::vector<double> latency_arr;

     double td_latency_sum = 0.0;
    std::vector<double> td_latency_arr;

     double td_rxn_time_sum = 0.0;
    std::vector<double> td_rxn_time_arr;
    double td_last_in_time = 0.0;

  double tput_sum = 0.0;
    std::vector<double> tput_arr;
    double last_out_time = 0.0;

    int nz_vel = 0;
    int not_in_frame = 0;
    int in_offset = 0;
    int percentile = 95;
    int perc7 = 75;
    int perc0 = 90;
    int perc2 = 99;
    double perc3 = 99.9;
    double perc4 = 99.99;

    bool rtc_event_driven;
    ros::Publisher rtc_pub;

    int num_cores;
    std::string suffix;

    int offline_durn;
    bool dyn_algo;
    double expt_start_time;
public:
    ObjTracker(double max_rot, bool ed, bool da, double x_thr, int off_durn, int nc, std::string suff)
    {
        img_width = 640;
        img_height = 480;
        min_rotation_speed = 0.5;
        max_rotation_speed = max_rot;
        gain = max_rot;
        x_threshold = x_thr;
	rtc_event_driven = ed;
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(pub_topic, 1);
        roi_sub = nh.subscribe(sub_topic, 1, &ObjTracker::move_robot, this, ros::TransportHints().tcpNoDelay(), true);
	
	dyn_algo = da;
	expt_start_time = 0.0;	

	offline_durn = off_durn;
	num_cores = nc;
	suffix = suff;

	if (rtc_event_driven)
	{
		// this node will publish a msg which will trigger the gz_ros_pkg camera_util to publish
		rtc_pub = nh.advertise<std_msgs::Header>(rtc_pub_topic, 1);
		while (0 == rtc_pub.getNumSubscribers())
        	{
            		ROS_INFO("Tracker : RTCPub: Waiting for subscribers to connect");
            		ros::Duration(0.1).sleep();
        	}
	}

        std::cout << "Subscribed to roi, about to call ros::spin \n" << "rtc?" << rtc_event_driven << ", dyn?" << dyn_algo << std::endl;
    }

    ~ObjTracker()
    {
	std::cout << "SHUTTING DOWN OBJTRACKER!" << std::endl;
	std::ofstream outfile;
	std::string fname = "FJ_Ser_NNP_Expt2_" + get_string(num_cores) + "c_" + get_algo() + "_Logs" + suffix;
        outfile.open(fname.c_str());
    	// need to write rxn Time arr, rel metric, rel metric 1
	// lets remove the first 500 data.
	std::cout << "Writing rxn time, Ignoring 1percent of sz " << rxn_time_arr.size() << std::endl;
	outfile << "RxnTime, ";
	for(int i = rxn_time_arr.size()/100; i < rxn_time_arr.size(); i++)
		outfile << rxn_time_arr[i] << ", ";
	outfile << "\n";	

	std::cout << "Writing TD rxn time, Ignoring 1percent of sz " << td_rxn_time_arr.size() << std::endl;
        outfile << "RxnTmTD, ";
        for (int i = td_rxn_time_arr.size()/100; i < td_rxn_time_arr.size(); i++)
                outfile << td_rxn_time_arr[i] << ", ";
        outfile << "\n";	

	std::cout << "Writing rel metric, Ignoring 1percent of sz " << metric_arr.size() << std::endl;
	outfile << "RelMetric, ";
	for(int i = metric_arr.size()/100; i < metric_arr.size(); i++)
		outfile << metric_arr[i] << ", ";
	outfile << "\n";
	
	std::cout << "Writing rel metric1, Ignoring 1percent of sz " << metric1_arr.size() << std::endl;
	outfile << "RelMetric1, ";
	for(int i = metric1_arr.size()/100; i < metric1_arr.size(); i++)
		outfile << metric1_arr[i] << ", ";
	outfile << "\n";

	std::cout << "Writing vel arr, Ignoring 1percent of sz " << vel_arr.size() << std::endl;
	outfile << "VelArr, ";
	for (int i = vel_arr.size()/100; i < vel_arr.size(); i++)
		outfile << vel_arr[i] << ", ";
	outfile << "\n";
    }

    std::string get_string(int x)
    {
        std::stringstream ss;
        ss << x;
        return ss.str();
    }
 
    std::string get_algo()
    {
	if (rtc_event_driven)
		return "RTC";
	else if (dyn_algo)
		return "Dyn";
	else
		return "Default";
    }

    void print_stats()
    {
	ROS_INFO("tOTAL COUNT %i, Not inf rame %i, NZ vel %i, In thresh %i", cb_count, not_in_frame, nz_vel, in_offset);
        // latency
        print_smt(latency_sum, latency_arr, "N3 latency");
        // tput
        print_smt(tput_sum, tput_arr, "Tput");
        // rxn time
        print_smt(rxn_time_sum, rxn_time_arr, "RxnTime");
        // metric
        print_smt(metric_sum, metric_arr, "Metric");
	// metric1
	print_smt(metric1_sum, metric1_arr, "Metric1");
        // cb time
        print_smt(cb_time_sum, cb_time_arr, "N3 CB Time");
    	// N3Lat w.r.t. TDNode
	print_smt(td_latency_sum, td_latency_arr, "N3 Lat w.r.t. TDNode");
	// TDrxnTime
	print_smt(td_rxn_time_sum, td_rxn_time_arr, "RxnTm w.r.t. TDNode");
    }

    void print_smt(double m_sum, std::vector<double> m_arr, std::string m)
    {
        int l = m_arr.size();
        if (l > 0)
        {
            std::sort(m_arr.begin(), m_arr.end());
            double avg = m_sum/l;
            double med = m_arr[l/2];
            double perc = m_arr[(l*percentile)/100];
	    
            ROS_INFO("Mean, median, tail of %s is %f %f %f , arr sz %i , %i : %f, %i : %f, %i : %f, %f : %f, %f : %f #", m.c_str(), avg, med, perc, l, perc7, m_arr[(l*perc7)/100], perc0, m_arr[(l*perc0)/100], perc2, m_arr[(l*perc2)/100], perc3, m_arr[(l*perc3)/100], perc4, m_arr[(l*perc4)/100]);            
        }
    }
 
    bool add_to_arr()
    {
	if (!dyn_algo)
		return true;
	else
		return ((ros::Time::now().toSec() - expt_start_time) > offline_durn); 
    }

    void move_robot(const std_msgs::Header::ConstPtr& msg)
    {
	if (expt_start_time == 0.0)
		expt_start_time = ros::Time::now().toSec();	
        double start_time = ros::Time::now().toSec();
        int x_offset, y_offset, width, height;
        std::stringstream ss(msg->frame_id);

        ss >> x_offset;
        ss >> y_offset;
        ss >> width;
        ss >> height;

	double td_ts;
	ss >> td_ts;

        if ((width == 0) || (height == 0))
        {
            move_cmd.angular.z = 0.0; // check all others are 0 as well
            move_cmd.angular.x = 0.0;
            move_cmd.angular.y = 0.0;

            move_cmd.linear.z = 0.0;
            move_cmd.linear.x = 0.0;
            move_cmd.linear.y = 0.0;
	    not_in_frame += 1;
	    metric1_sum += 1.0;
	    metric1_arr.push_back(1.0);
        }
        else
        {
            float target_offset_x = x_offset + ((float)width/2) - ((float)img_width/2);
            float perc_offset_x = target_offset_x/(img_width/2.0);

            float m = perc_offset_x;
            if (perc_offset_x < 0.0)
            {
                m = -1.0 * m;
                if (m == perc_offset_x)
                    std::cout << "ERRORRR perc_offset_x was negative, m should've been +ve now...";
            }

	    if (add_to_arr())
	    {
	      metric_sum += m;
              metric_arr.push_back(m);

	      metric1_sum += m;
	      metric1_arr.push_back(m);

	    }

            if (m > x_threshold)
            {
                float speed = gain*perc_offset_x;
                int direction = (speed < 0) ? -1 : 1;
                move_cmd.angular.z = -1 * direction * std::max(min_rotation_speed, std::min(max_rotation_speed, std::abs(speed)));
                nz_vel += 1;
	    }
            else
            {
                move_cmd.angular.z = 0.0; // check all others are 0 as well
                move_cmd.angular.x = 0.0;
                move_cmd.angular.y = 0.0;

                move_cmd.linear.z = 0.0;
                move_cmd.linear.x = 0.0;
                move_cmd.linear.y = 0.0;
                in_offset += 1;
	    }
        }

        cmd_vel_pub.publish(move_cmd);
	if (add_to_arr())
		vel_arr.push_back(move_cmd.angular.z);

	if (rtc_event_driven)
	{
		std_msgs::Header hdr;
		hdr.stamp = ros::Time::now();
		hdr.frame_id = "";
		rtc_pub.publish(hdr);
	}

        if ( (cb_count > 5) && (add_to_arr()) )
        {
            float lat = (ros::Time::now() - msg->stamp).toSec();
            latency_sum += lat;
            latency_arr.push_back(lat);

	    float td_lat = ros::Time::now().toSec() - td_ts;
	    td_latency_sum += td_lat;
	    td_latency_arr.push_back(td_lat);

            if (last_out_time > 0.0)
            {
                double tput = ros::Time::now().toSec() - last_out_time;
                tput_sum += tput;
                tput_arr.push_back(tput);
            }

            if (last_in_time > 0.0)
            {
                double rxn_time = ros::Time::now().toSec() - last_in_time;
                rxn_time_sum += rxn_time;
                rxn_time_arr.push_back(rxn_time);
            }

            if (td_last_in_time > 0.0)
            {
                double td_rxn_time = ros::Time::now().toSec() - td_last_in_time;
                td_rxn_time_sum += td_rxn_time;
                td_rxn_time_arr.push_back(td_rxn_time);
            }


        }

        last_out_time = ros::Time::now().toSec();
        last_in_time = msg->stamp.toSec();
	td_last_in_time = td_ts;
        if (cb_count%800 == 151)
        {
            ROS_INFO("TRacker cb count : %d, max rot speed %f, gain %f, TD TS %f, msg->frame %s", cb_count, max_rotation_speed, gain, td_ts, msg->frame_id.c_str());
            print_stats();
        }

        cb_count += 1;
        double cb_time = ros::Time::now().toSec() - start_time;
        cb_time_sum += cb_time;
        cb_time_arr.push_back(cb_time);
    }
};

int main(int argc, char** argv)
{
    double max_rot = atof(argv[1]);
    bool event_driven = (atoi(argv[2]) == 1);
    // if this is true, start measuring stats after 60sec.
    bool dyn_algo = (atoi(argv[3]) == 1);
    int offline_durn = atoi(argv[4]);
    int nc = atoi(argv[5]);
    std::string suffix = argv[6];
    std::string node_name = "objecttracker";
    ROS_INFO("Init node name %s, max rot %f", node_name.c_str(), max_rot);
    ros::init(argc, argv, node_name);
    ObjTracker ot(max_rot, event_driven, dyn_algo, 0.1, offline_durn, nc, suffix);
    ros::spin();

    return 0;
}
