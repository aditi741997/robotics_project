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
#include <utility>

class NewFreqController
{
    // store stats of all the nodes
    std::vector<std::string> node_cb_topics;
    std::vector<ros::Subscriber> cb_topics_subs;
    // dictionary of topic -> index
    std::map<std::string, int> topic_map;

    // stats required for calculating the frequency :
    // for each of these arrays :
    // arr[j] = ci's if bin sz is j
    // Length of each of these arrays is max_qi.
    std::vector<std::vector<double> > mean_cb_times;
    std::vector<std::vector<double> > mean_cb_times_L;    

    std::vector<std::vector<double> > med_cb_times;
    std::vector<std::vector<double> > med_cb_times_L;

    std::vector<std::vector<double> > tail_cb_times;
    std::vector<std::vector<double> > tail_cb_times_L;

    std::vector<int> qi; // denotes the max number of cores the ith node can use.
    int sum_qi, max_qi; // remain constant during a run.

    std::vector<int> cb_topics_msg_count;

    int num_cores;
    int num_nodes;

    double current_freq;
    double current_bin_sz;
    double max_change_rate;
    double freq_threshold;
    double mult_eps;
    bool use_median;
    double last_update_time;

    double offline_durn;

    ros::NodeHandle nh;
    ros::Timer ct, ct2;

public:
    NewFreqController(std::vector<std::string>& cb_topics, std::vector<int> cores_i, int k, double mcr, double thresh, double epsilon, bool use_med)
    {
        node_cb_topics = cb_topics;
        num_nodes = cb_topics.size();
        qi = cores_i;

        use_median = use_med;

        sum_qi = 0;
        max_qi = 0;
        for (int i = 0; i < cores_i.size(); i++)
        {
            sum_qi += cores_i[i];
            max_qi = std::max(max_qi, cores_i[i]);
        }

        for (int i = 0; i < cb_topics.size(); i++)
        {
            // subcribe to all the topics.
            ros::Subscriber si = nh.subscribe<std_msgs::Header>(cb_topics[i], 1, &NewFreqController::updateStats, this, ros::TransportHints().tcpNoDelay());
            cb_topics_subs.push_back(si);
            topic_map[cb_topics[i]] = i;
        }
        ROS_INFO("Subscribed to all the topics! About to read from file");

        mean_cb_times = std::vector<std::vector<double> > (max_qi, std::vector<double> (num_nodes, 0.0));
        med_cb_times = std::vector<std::vector<double> > (max_qi, std::vector<double> (num_nodes, 0.0));
        tail_cb_times = std::vector<std::vector<double> > (max_qi, std::vector<double> (num_nodes, 0.0));

        mean_cb_times_L = std::vector<std::vector<double> > (max_qi, std::vector<double> (num_nodes, 0.0));
        med_cb_times_L = std::vector<std::vector<double> > (max_qi, std::vector<double> (num_nodes, 0.0));
        tail_cb_times_L = std::vector<std::vector<double> > (max_qi, std::vector<double> (num_nodes, 0.0));

        cb_topics_msg_count = std::vector<int> (num_nodes, 0);

        num_cores = k;
        max_change_rate = mcr;
        mult_eps = epsilon;
        freq_threshold = thresh;

        // Start with binsz = 1 & low freq.
        current_bin_sz = 1;
        current_freq = 10;
        updateParams(current_bin_sz, current_freq);
        // Timer for 40sec : Switch to binsz = 2.
        ct = nh.createTimer(ros::Duration(offline_durn+1.0), &NewFreqController::offlineEval, this, true);
        // Another timer to finish offline eval and determine best schedule.
        // TODO:Later : if qi > 2: we will use a for loop to make qi timers.
        ct = nh.createTimer(ros::Duration(2*offline_durn+1.0), &NewFreqController::offlineEval, this, true);
    }

    void offlineEval(const ros::TimerEvent& event)
    {
        if (current_bin_sz < max_qi)
            updateBinSz(current_bin_sz+1);
        else
        {
            // All qi's have been covered. Time to find the best bin_Sz, freq.
            std::pair<double, int> best_freq_binsz = schedAlgo();
            updateParams((best_freq_binsz.second), (best_freq_binsz.first));
        }
    }

    void updateParams(int bin_sz, double freq)
    {
        updateFrequency(freq);
        updateBinSz(bin_sz);
        last_update_time = ros::Time::now().toSec();
    }

    void updateFrequency(double new_freq)
    {
        // scale down by a factor :
        double new_new_freq = (1.0-mult_eps)*new_freq;
        // Since the compute values will be inflated due to resource contention,
        // we don't directly move to new_freq.
        new_new_freq = current_freq*0.2 + new_freq*(1.0-0.2);
        std::string cmd = "rosrun dynamic_reconfigure dynparam set /camera \"{'camera_update_rate': ";
        cmd += get_string(new_new_freq);
        cmd += ", 'imager_rate': 0.0}\"";
        system(cmd.c_str());

        ROS_INFO("Changed frequency to %f, using cmd %s", new_new_freq, cmd.c_str());
        current_freq = new_new_freq;
        last_update_time = ros::Time::now().toSec();
    }

    void updateBinSz(int bin_sz)
    {
        std::string cmd = "rosrun dynamic_reconfigure dynparam set /obj_detector \"{'bin_size': ";
        cmd += get_string(bin_sz);
        cmd += "}\"";
        system(cmd.c_str());

        ROS_INFO("Changed bin size to %i, usimg cmd %s", bin_sz, cmd.c_str());
        current_bin_sz = bin_sz;
        last_update_time = ros::Time::now().toSec();
    }

    void updateStats(const std_msgs::Header::ConstPtr& msg)
    {
        std::stringstream ss(msg->frame_id);
        std::string topic, stage;
        double update, update2;

        ss >> topic;
        ss >> update; // med
        ss >> update2; // 95% tail

        int ind = topic_map[topic];

        if (stage.find('L') != std::string::npos)
        {
            if (cb_topics_msg_count[ind]%20 == 5)
                ROS_INFO("Received an L update : %s, median of %s [ind:%i] is %f & tail is %f, count: %i", msg->frame_id.c_str(), topic.c_str(), ind, update, update2, cb_topics_msg_count[ind]);
            cb_topics_msg_count[ind] += 1;

            // update L stats.
            med_cb_times_L[current_bin_sz][ind] = update;
            tail_cb_times_L[current_bin_sz][ind] = update2;

            // check if we need to change freq.
            // If bin size needs to change OR if for same bin sz the freq is diff by more than threshold.
            std::pair<double, int> new_sched = schedAlgo();
            int bs = (new_sched.second);
            double fr = (new_sched.first);
            if ( ( (bs != current_bin_sz) || (std::abs( ((1.0-mult_eps)*fr) - current_freq) >= (freq_threshold*current_freq) ) ) && ( (ros::Time::now().toSec() - last_update_time) >= 1.0/max_change_rate) )
            {
                ROS_INFO("L-UPDATE NEED to change schedule! Got msg %s, current_bin_sz %i, current_freq %f, NEW bin_sz : %i, NEW freq : %f", msg->frame_id.c_str(), current_bin_sz, current_freq, bs, fr);
                // TODO: update the schedule!
            }
        }
        else
        {
            // offline eval stage.
            // if t<40 & [TODO:within 20% of current value] then Stage1
            // else Stage2
            med_cb_times[current_bin_sz][ind] = update;
            med_cb_times_L[current_bin_sz][ind] = update;

            tail_cb_times[current_bin_sz][ind] = update2;
            tail_cb_times_L[current_bin_sz][ind] = update2;
        }
    }

    // Returns freq, rxnTime pair by using the single chain optimal frequency formula.
    std::pair<double, double> bestFreq(std::vector<double>& cb_times, int k)
    {
        // can be used for any bin sz : we give k as num_Cores/binsz and the cb_times_arr[binsz]
        double m = 0.0;
        double sig = 0.0;
        for (int i = 0; i < cb_times.size(); i++)
        {
            m = std::max(m, cb_times[i]);
            sig += cb_times[i];
        }
        double new_freq = (float)k/sig;
        if (m > 0.0)
            new_freq = std::min(1.0/m, new_freq);
        double rxn_time = sig + 1.0/new_freq;
        return std::make_pair(new_freq, rxn_time);
    }

    std::string get_string(double x)
    {
        std::stringstream ss;
        ss << x;
        return ss.str();
    }

    std::string get_string(int x)
    {
        std::stringstream ss;
        ss << x;
        return ss.str();
    }

    // Returns best freq, bin_sz combo
    // obtained by iterating over all possible bin sizes.
    std::pair<double, int> schedAlgo()
    {
        int bin_sz = 1;
        double freq = 1.0;
        std::pair<double, double> freq_rt;

        if (num_cores >= sum_qi)
        {
            // Handle this case differently. This means that there are sufficient resources.
            // This can be treated as a single chain with serial nodes, each with compute times ci[qi]
            // the frequency in this case is the optimal. Sig' + M'
            if (use_median)
                freq_rt = bestFreq(med_cb_times_L[max_qi-1], num_nodes);
            else
                freq_rt = bestFreq(tail_cb_times_L[max_qi-1], num_nodes);
            
            bin_sz = max_qi;
            freq = (freq_rt.first);
            ROS_INFO("SCHED_ALGO : Have sufficient cores, hence bin sz : %i, freq : %f", bin_sz, freq);
        }
        else
        {
            double rxntm = 5.0; // need to find the best bin sz.
            double best_freq = 1.0;
            int best_binsz = 1;
            for (int binsz = 1; binsz <= (std::min(max_qi, num_cores)); binsz++)
            {
                int num_bins = num_cores/binsz;
                if (use_median)
                    freq_rt = bestFreq(med_cb_times_L[binsz-1], num_bins);
                else
                    freq_rt = bestFreq(tail_cb_times_L[binsz-1], num_bins);
                
                if ((freq_rt.second) < rxntm)
                {
                    best_freq = (freq_rt.first);
                    rxntm = (freq_rt.second);
                    best_binsz = binsz;
                }
            }
            bin_sz = best_binsz;
            freq = best_freq;
            ROS_INFO("SCHED_ALGO : Insufficient cores, found best bin sz : %i, freq : %f", bin_sz, freq);
        }
        // bin_sz = max_qi essentially means that each node will get qi cores whenever its running.
        // (since we choose the freq accordingly at every bin_sz)
        return std::make_pair(freq, bin_sz);
    }
};

int main(int argc, char **argv)
{
    std::string node_name = "new_controller";
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
    return 0;
}
