#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstdlib>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/condition_variable.hpp>
#include <bits/stdc++.h>

class FreqController
{
    // store stats of all the nodes
    std::vector<std::string> node_cb_topics;
    std::vector<ros::Subscriber> cb_topics_subs;
    // dictionary of topic -> index
    std::map<std::string, int> topic_map;

    // stats required for calculating the frequency :
    std::vector<double> mean_cb_times;
    std::vector<double> mean_cb_times_L;    

    std::vector<double> med_cb_times;
    std::vector<double> med_cb_times_L;

    std::vector<double> tail_cb_times;
    std::vector<double> tail_cb_times_L;

    std::vector<int> cb_topics_msg_count;

    int num_cores;
    int num_nodes;

    double current_freq;
    double last_freq_change_time;
    double change_rate;
    double max_change_rate;
    double freq_threshold;
    double right_freq_thresh;
    double eps;

    ros::NodeHandle nh;
    boost::thread controller_thread;
    ros::Timer ct;

    double offline_durn;
    double offline_freq;
    bool use_median;

public:
    FreqController(std::vector<std::string>& cb_topics, int k, double mcr, double thresh, double epsilon, double offln_durn, double offln_freq, bool use_med)
    {
        node_cb_topics = cb_topics;
        num_nodes = cb_topics.size();

        for (int i = 0; i < cb_topics.size(); i++)
        {
            // subcribe to all the topics.
            ros::Subscriber si = nh.subscribe<std_msgs::Header>(cb_topics[i], 1, &FreqController::updateStats, this, ros::TransportHints().tcpNoDelay());
            cb_topics_subs.push_back(si);
            topic_map[cb_topics[i]] = i;
        }
        ROS_INFO("Subscribed to all the topics! About to read from file");
        mean_cb_times = std::vector<double> (num_nodes, 0.0);
        med_cb_times = std::vector<double> (num_nodes, 0.0);
        tail_cb_times = std::vector<double> (num_nodes, 0.0);

        mean_cb_times_L = std::vector<double> (num_nodes, 0.0);
        med_cb_times_L = std::vector<double> (num_nodes, 0.0);
        tail_cb_times_L = std::vector<double> (num_nodes, 0.0);

        cb_topics_msg_count = std::vector<int> (num_nodes, 0);

        // thread to check frequency every second
        // Not needed in v1
        // controller_thread = boost::thread(boost::bind(&FreqController::controllerFunc, this));
        // running = true;

        num_cores = k;
        max_change_rate = mcr;
        change_rate = 1.0;
	eps = epsilon;
        freq_threshold = thresh;
	right_freq_thresh = thresh - 0.01;

	offline_durn = offln_durn;
	offline_freq = offln_freq;
	use_median = use_med;
        // v1 : read stats from file
/*        std::ifstream cb_stats_file("/home/ubuntu/catkin_ws/obj_track_cb_stats_file.txt");
        std::string line;
        while (std::getline(cb_stats_file, line))
        {
            std::stringstream ss;
            ss << line;
            double a,b,c;
            std::string topic_l;
            ss >> a;
            ss >> b;
            ss >> c;
            ss >> topic_l;
            if (topic_map.find(topic_l) != topic_map.end())
            {
                mean_cb_times[topic_map[topic_l]] = a;
                med_cb_times[topic_map[topic_l]] = b;
                perc_cb_times[topic_map[topic_l]] = c;
                // Also initializing window stats to same value [to avoid error on 1st message]
                mean_cb_times_L[topic_map[topic_l]] = a;
                med_cb_times_L[topic_map[topic_l]] = b;
                perc_cb_times_L[topic_map[topic_l]] = c;                
                ROS_INFO("Found stats for topic %s, mean median tail %f %f %f", topic_l.c_str(), a, b, c);
            }
        }

        // init : set current freq based on offline stats
        // v1 : median based. Assuming m >= 0.0
        double m = 0.0;
        double sig = 0.0;
        for (int i = 0; i < num_nodes; i++)
        {
            m = std::max(m, med_cb_times[i]);
            sig += med_cb_times[i];
        }
        current_freq = std::min(1.0/m, (float)num_cores/sig);
        updateFrequency(current_freq);
*/
	// v2 : initialize with low frequency.
	current_freq = offline_freq;
	updateFrequency(current_freq);

ROS_INFO("Finished controller init. Params : k %i, m %i, curr_freq %f, change_rate %f, max_change_rate %f, freq_thresh %f", num_cores, num_nodes, current_freq, change_rate, max_change_rate, freq_threshold);

	// v2 : after one minute, set freq based on all stats arrived.
        ct = nh.createTimer(ros::Duration(offline_durn), &FreqController::controllerFunc, this, true);  
    }

    void updateStats(const std_msgs::Header::ConstPtr& msg)
    {
        // this is called when any of the nodes publishes its stats.
        std::stringstream ss(msg->frame_id);
        std::string topic, stage;
	double update, update2;
        // double mean_cb, med_cb, perc_cb, mean_cb_L, med_cb_L, perc_cb_L;
        ss >> topic;
	ss >> stage;
        // ss >> mean_cb;
        // ss >> med_cb;
        // ss >> perc_cb;

        // v1 : only publishing topic name, median value.
	// v2 : topic name, med, tail
        ss >> update;
	ss >> update2;
        // ss >> mean_cb_L;
        // ss >> perc_cb_L;

        int ind = topic_map[topic];

	if (stage.find('L') != std::string::npos)
	{
		if (cb_topics_msg_count[ind]%20 == 0)
			ROS_INFO("Received an L update : %s, median of %s [ind:%i] is %f & tail is %f, count: %i", msg->frame_id.c_str(), topic.c_str(), ind, update, update2, cb_topics_msg_count[ind]);
		// update med_cb_times_L and check for freq...
		// v1: Only median
        	med_cb_times_L[ind] = update;
		// v2 : tail, median
		tail_cb_times_L[ind] = update2;

		cb_topics_msg_count[ind] += 1;

       		// calculate new optimal freq
        	double m = 0.0;
        	double sig = 0.0;
		if (use_median)
		{
		 	for (int i = 0; i < med_cb_times_L.size(); i++)
        		{
        	    		// find M
            			m = std::max(m, med_cb_times_L[i]);
            			// find Sigma
            			sig += tail_cb_times_L[i];
        		}
		}
		else
		{
	        	for (int i = 0; i < tail_cb_times_L.size(); i++)
        		{
        	    		// find M
            			m = std::max(m, tail_cb_times_L[i]);
            			// find Sigma
            			sig += tail_cb_times_L[i];
        		}
		}
        	double new_freq = (float)num_cores/sig;
        	if (m > 0.0)
            		new_freq = std::min(1.0/m, new_freq);

        	// if [this freq is very different [1.5x ?] AND last_freq_change-now >= 1/max_change_rate] from current freq, notify cv
        	if (  (std::abs((new_freq*(1.0 - eps)) - current_freq) >= (freq_threshold*current_freq) ) && ( (ros::Time::now().toSec() - last_freq_change_time) >= (1.0/max_change_rate) ) )
        	{
            		ROS_INFO("L-UPDATE : NEED to change frequency! Got msg %s, current_freq %f, new_freq %f", msg->frame_id, current_freq, new_freq);
            		updateFrequency(new_freq);
        	}

	}
	else
	{
		ROS_INFO("Received an E update : %s, median of %s [ind:%i] is %f & tail is %f", msg->frame_id.c_str(), topic.c_str(), ind, update, update2);
		// update med_cb_times
		med_cb_times[ind] = update;
		// also initialze med_cb_times_L
		med_cb_times_L[ind] = update;
		// v2 : also update tail
		tail_cb_times[ind] = update2;
		tail_cb_times_L[ind] = update2;
	}

        // mean_cb_times[ind] = mean_cb;
        // med_cb_times[ind] = med_cb;
        // perc_cb_times[ind] = perc_cb_times;

        // update L stats as well
        // mean_cb_times_L[ind] = mean_cb_L;
        // perc_cb_times_L[ind] = perc_cb_L;

    }

    std::string get_string(double x)
    {
        std::stringstream ss;
        ss << x;
        return ss.str();
    }    

    void updateFrequency(double new_freq)
    {
//        double new_new_freq = (current_freq + new_freq)/2.0;
	double new_new_freq = new_freq*(1.0 - eps); // since we're using clock() time, freq value is exact.
        // using dynamic reconfigure to update publishing frequency in gazebo :
        std::string cmd = "rosrun dynamic_reconfigure dynparam set /camera \"{'camera_update_rate': ";
        cmd += get_string(new_new_freq);
        cmd += ", 'imager_rate': 0.0}\"";
        system(cmd.c_str());

        ROS_INFO("Changed frequency to %f, using cmd %s", new_new_freq, cmd.c_str());
        current_freq = new_new_freq;
        last_freq_change_time = ros::Time::now().toSec();
    }

    void controllerFunc(const ros::TimerEvent& event)
    {
	ROS_INFO("In controllerFunc");
	// now calculate the best freq based on stats received
        	double m = 0.0;
        	double sig = 0.0;

		if (use_median)
		{
			for (int i = 0; i < med_cb_times_L.size(); i++)
        		{
            			// find M
				double x = med_cb_times_L[i];
            			m = std::max(m, x);
            			// find Sigma
            			sig += x;
        		}
		}
        	else
		{
			// use tail
			for (int i = 0; i < tail_cb_times_L.size(); i++)
        		{
            			// find M
				double x = tail_cb_times_L[i];
            			m = std::max(m, x);
            			// find Sigma
            			sig += x;
			}
		}
		double new_freq = (float)num_cores/sig;
        	if (m > 0.0)
            		new_freq = std::min(1.0/m, new_freq);
		ROS_INFO("Controller : New freq : %f", new_freq);
	
	if ( (std::abs((new_freq - eps) - current_freq) >= (freq_threshold*current_freq) ) && ( (ros::Time::now().toSec() - last_freq_change_time) >= (1.0/max_change_rate) ) )
                {
                        ROS_INFO("Controller : NEED to change frequency! current_freq %f, new_freq %f", current_freq, new_freq);
                        updateFrequency(new_freq);
                }

    }
};

int main(int argc, char **argv)
{
    std::string node_name = "controller";
    int k = atoi(argv[1]); // num of cores
    int m = atoi(argv[2]); // number of nodes
    double freq_thresh = atof(argv[3]);
    double epsi = atof(argv[4]);
    double offln_durn = atof(argv[5]); 
    double offln_freq = atof(argv[6]);
    bool use_med = (atoi(argv[7]) == 1);
    ROS_INFO("Init node name %s, num_cores %i, num ndoes %i, offline durn %f, offline freq %f", node_name.c_str(), k, m, offln_durn, offln_freq);
    ros::init(argc, argv, node_name);
    std::vector<std::string> topic_list (m);
    for (int i = 0; i < m; i++)
        topic_list[i] = argv[8+i];

    FreqController fc(topic_list, k, 10.0, freq_thresh, epsi, offln_durn, offln_freq, use_med);
    ros::spin();

    return 0;
}
