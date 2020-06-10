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
// For adding a heavy thread :
#include <thread>
#include <cstdlib>
#include <stdint.h>

double sum_latency = 0.0;
double sum_latency_sq = 0.0;
int msg_count = 0;

int msg_size = 0;
int pub_queue_len = 0;
float ros_rate = 0.0;
int num_msgs = 0;
int sub_queue_len = 0;
int transport_type = 0;

int sub_id = 0;
bool do_heavy = false;

const double megabyte = 1024*1024;
double used_ram = 0.0;

ros::Time last_recv_time;
double sum_recv_delta = 0.0;
std::vector<double> recv_delta_arr;

bool write_lat = false;
std::vector<double> latencies;

double sum_total_delay = 0.0;
std::vector<double> total_delay;

// for calculating tput : keeping array of times b/w outputs of subscriber [i.e. after heavy is over]
double last_sub_output = 0.0;
double sum_op_delta = 0.0;
std::vector<double> op_delta;

// for d+1/t : separate array
double sum_dt_sum = 0.0;
std::vector<double> dt_sum;

double avg_heavy_time = 0.0;
std::vector<double> heavy_times;

double compute_rt_sum = 0.0;
std::vector<double> compute_rt_arr;

double compute_ts_sum = 0.0;
std::vector<double> compute_ts_arr;

int percentile = 95;

int64_t limit;
bool stop_thread = false;
bool threaded = false;

bool to_publish = false;
std::string expt;
std::string node_name;
std::string publish_topic = "";
ros::Publisher chatter_pub;
std_msgs::Header hdr;

double pub_time = 0.0;
std::vector<double> pub_times;

double td_latency_sum = 0.0;
std::vector<double> td_latency_arr;

bool add_rand_noise = false;

void calc_primes(int64_t limit, bool duh)
{
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
}

void exec(int64_t limit)
{
	if (threaded)
	{
		std::thread t1(calc_primes, limit, true);
		std::thread t2(calc_primes, limit, true);
		t1.join();
		t2.join();
	}
	else
		calc_primes(limit, true);
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
            ROS_INFO("Mean, median, tail of %s is %f %f %f # Len : %i #", m.c_str(), avg, med, perc, l);            
        }
    }

void chatterCallBack(const std_msgs::Header::ConstPtr& msg)
{
    ros::Time recv_time = ros::Time::now();
    clock_t cb_start_rt = clock();

    double recv_delta = (msg_count == 0) ? 0.0 : (recv_time - last_recv_time).toSec();
    double lat = (recv_time - msg->stamp).toSec();
    recv_delta_arr.push_back(recv_delta);
    latencies.push_back(lat);
    if (msg_count%200 == 5)
    	ROS_INFO("sent time : [%f], recv_time : [%f], msg count : %i, msg id : %i", msg->stamp.toSec(), recv_time.toSec(), msg_count, msg->seq);

    msg_count += 1;
    sum_latency += lat;
    sum_latency_sq += lat * lat;
    sum_recv_delta += recv_delta;

    last_recv_time = recv_time;

    // struct sysinfo si;
    // sysinfo(&si);
    // used_ram += (si.totalram - si.freeram)/megabyte;

    if (do_heavy)
    {
        ros::Time heavy_start = ros::Time::now();
        // Single heavy function :
        // sieve(limit);
        //calc_primes(limit, true);
	exec(limit);
        // Adding 2 threads :
        // std::thread t1(sieve, limit);
        // std::thread t2(sieve, limit);
        // t1.join();
        // t2.join();
	
	double compute_rt = (double)(clock() - cb_start_rt)/CLOCKS_PER_SEC;
    	compute_rt_sum += compute_rt;
    	compute_rt_arr.push_back(compute_rt);

        avg_heavy_time += (ros::Time::now() - heavy_start).toSec();
        heavy_times.push_back((ros::Time::now() - heavy_start).toSec());
    }
    
    double dt = 0.0;
    ros::Time to = ros::Time::now();

    double delay = (to - msg->stamp).toSec();
    total_delay.push_back(delay);
    sum_total_delay += delay;

    if (msg_count >= 2)
    {
        double op_del = (to).toSec() - last_sub_output;
        op_delta.push_back(op_del);
        sum_op_delta += op_del;

        // dt is the d+1/t for this cycle. Add to array, sum value.
        // dt += op_del + delay;
        // sum_dt_sum += dt;
        // dt_sum.push_back(dt);

    	dt = op_del + total_delay[total_delay.size() - 2];
    	sum_dt_sum += dt;
    	dt_sum.push_back(dt);
    }
    last_sub_output = to.toSec();

    if (to_publish)
    {
	ros::Time pub_start = ros::Time::now();
	// ROS_INFO("About to publish!");
	hdr.seq = msg->seq;
	hdr.stamp = msg->stamp;
	hdr.frame_id = msg->frame_id;
	chatter_pub.publish(hdr);
	double pub_t = (ros::Time::now() - pub_start).toSec();
	pub_time += pub_t;
	pub_times.push_back(pub_t);
	if (pub_t > 0.004)
		ROS_INFO("Subscriber: pub time more than 4ms");
    }

    if (msg_count%50 == 42)
    {
	print_smt(sum_latency, latencies, "Latency at " + node_name);
	print_smt(compute_rt_sum, compute_rt_arr, "RT Compute time at " + node_name);
	print_smt(avg_heavy_time, heavy_times, "Ros::Time Compute time at " + node_name);
    }

    if (msg->seq >= ((num_msgs*98)/100))
    {
        int last_recv_msg = msg->seq;
        ROS_INFO("Last received msg Id %i", last_recv_msg);
        
        double std_dev = sqrt(sum_latency_sq - (sum_latency*sum_latency/msg_count))/msg_count;
        sum_latency = sum_latency/msg_count;
        
        sum_recv_delta /= (msg_count - 1);
        // sort the arr
        std::sort(recv_delta_arr.begin(), recv_delta_arr.end());
        double median_recv_delta = recv_delta_arr[msg_count/2];

        std::sort(latencies.begin(), latencies.end());
        int index = (percentile*(msg_count))/100;
        // std::cout << "index : " << index << "\n";
        double percentile_lat = latencies[index];
        double median_lat = latencies[latencies.size()/2];

        std::sort(total_delay.begin(), total_delay.end());
        double perc_total_delay = total_delay[index];
        double median_total_delay = total_delay[total_delay.size()/2];
        double mean_total_delay = sum_total_delay/(total_delay.size());

        std::sort(op_delta.begin(), op_delta.end());
        index = (percentile*(op_delta.size()))/100;
        double perc_op_delta = op_delta[index];
        double median_op_delta = op_delta[op_delta.size()/2];
        double mean_op_delta = sum_op_delta/(op_delta.size());

        std::sort(dt_sum.begin(), dt_sum.end());
        double perc_dt_sum = dt_sum[index];
        double median_dt_sum = dt_sum[dt_sum.size()/2];
        double mean_dt_sum = sum_dt_sum/(dt_sum.size());

        std::sort(heavy_times.begin(), heavy_times.end());
	double perc_heavy = heavy_times[index];
	double median_heavy = heavy_times[heavy_times.size()/2];

	print_smt(compute_rt_sum, compute_rt_arr, "Real time compute time");

	if (to_publish)
	{
		std::sort(pub_times.begin(), pub_times.end());
	}

        // used_ram /= msg_count;std::to_string(sub_id) + 
        avg_heavy_time /= msg_count;

	// compute RT :
	std::sort(compute_rt_arr.begin(), compute_rt_arr.end());
	int num_crt = compute_rt_arr.size();
	double perc_ci_rt = compute_rt_arr[(percentile*num_crt)/100];
	double med_ci_rt = compute_rt_arr[num_crt/2];
	double mean_ci_rt = compute_rt_sum/num_crt;

        std::ofstream outfile;
        outfile.open((expt + "_" + node_name + "_May.txt").c_str(), std::ios_base::app);
        outfile << msg_size << ", " << pub_queue_len << ", " << num_msgs << ", " << sub_queue_len << ", " << ros_rate << ", " << transport_type << ", " << do_heavy << ", " << perc_total_delay << ", " << median_total_delay << ", " << mean_total_delay << ", " << perc_op_delta << ", " << median_op_delta << ", " << mean_op_delta << ", rxnTime, " << perc_dt_sum << ", " << median_dt_sum << ", " << mean_dt_sum << ", " << "lost_msgs, " << (msg->seq + 1 - msg_count) << ", " << (msg->seq + 1) << ", c1n_latency, " << percentile_lat << ", " << median_lat << ", " << sum_latency << ", c2, " << perc_heavy << ", " << median_heavy << ", " << avg_heavy_time << ", c2 rt, " << perc_ci_rt << ", " << med_ci_rt << ", " << mean_ci_rt << ", ";
	if (to_publish)
	{
		outfile << "pub_time, " << pub_times[index] << ", " << pub_times[pub_times.size()/2] << ", " << pub_time/msg_count << ", ";
	}
	outfile << "\n";
        // outfile << msg_size << ", " << pub_queue_len << ", " << num_msgs << ", " << sub_queue_len << ", " << ros_rate << ", " << transport_type << ", " << do_heavy << ", " << sum_latency << ", " << percentile_lat << ", " << std_dev << ", " << sum_recv_delta << ", " << median_recv_delta << ", " << avg_heavy_time  << ", " << msg_count << ", " << last_recv_msg << ", \n";

        if (write_lat)
        {
            std::ofstream outfile;
            outfile.open("latencies.txt", std::ios_base::app);
            outfile << msg_size << ", " << pub_queue_len << ", " << num_msgs << ", " << sub_queue_len << ", " << ros_rate << ", " << transport_type << ", " << do_heavy << "\n";
            for (double &d: latencies)
            {
                outfile << d << "\n";
            }
        }

        ROS_INFO("ALL PARAMS : %i, %i, %i, %i, %i, %i, %i, %i", msg_size, pub_queue_len, ros_rate, num_msgs, sub_queue_len, transport_type, sub_id, do_heavy);
        ROS_INFO("FINAL VALUE OF LATENCY : [%f], STD DEV : [%f], used ram : [%f], sum rec delta : [%f]", sum_latency, std_dev, used_ram, sum_recv_delta);
        ros::shutdown();
    }
}

int main (int argc, char **argv)
{
    if (argc < 13)
    {
        std::cout << "Not enough inputs. Need msg size, pub queue len, ros rate, num msgs, sub queue len, transport type (0 tcp, 1 tcpNoDelay, 2 udp), sub id, do heavy, limit for sieve";
        return 0;
    }

    msg_size = atoi(argv[1]);
    pub_queue_len = atoi(argv[2]);
    ros_rate = atoi(argv[3]);
    num_msgs = atoi(argv[4]);
    sub_queue_len = atoi(argv[5]);
    transport_type = atoi(argv[6]);
    // sub_id = atoi(argv[7]);
    do_heavy = atoi(argv[7]);

    limit = atoi(argv[8]);
    node_name = argv[9];

    ROS_INFO("Init node %s, sub_topic %s, expt %s, num_msgs : %i, ros_rate : %i", node_name.c_str(), argv[10], argv[11], num_msgs, ros_rate);
    ROS_INFO("Other params : pql %i, sql %i, doheavy %i, limit %i", pub_queue_len, sub_queue_len, do_heavy, limit);
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    
    std::string sub_topic = argv[10];
    expt = argv[11];

    to_publish = atoi(argv[12]) == 1;
    if (to_publish)
    {
     	publish_topic = argv[13];
    	chatter_pub = n.advertise<std_msgs::Header>(publish_topic, sub_queue_len);
    	 std::stringstream ss;
             char* message;
             message = new char[msg_size - 4];
             memset(message, '0', msg_size - 4);
             message[msg_size - 4 - 1] = '\0';
             ss << message;
    	 hdr.frame_id = ss.str();

     /*  publish_topic = "/camera1/rgb/image_raw";
        chatter_pub = n.advertise<sensor_msgs::Image>(publish_topic, sub_queue_len);
*/

    	ROS_INFO("I will publish on %s", (publish_topic + "BLAH").c_str());

    	while (0 == chatter_pub.getNumSubscribers())
    	{
            ROS_INFO("Subscriber name %s Waiting for subscribers to connect", node_name.c_str());
            ros::Duration(0.1).sleep();
        }
        ros::Duration(1.5).sleep();
    }

    
     // creating a thread
    // std::thread t1(thread_func, limit);
    
  // ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1, chatterImgCallBack, ros::TransportHints().tcpNoDelay(), true);
      ros::Subscriber sub = n.subscribe(sub_topic, sub_queue_len, chatterCallBack, ros::TransportHints().tcpNoDelay(), true);
    // arg14 denotes the algorithm. if 1: RTC, i.e. nothing is dropped. if 0 : drop & hence changeBinSize.
    if ( (publish_topic.find("gcmp") != std::string::npos) && (atoi(argv[14]) == 0) )
	n.changeBinSize(2); // to denote that we should drop every other msg.
    
    threaded = (atoi(argv[15]) == 1);
    ROS_INFO("Node %s threaded? %i", node_name.c_str(), threaded);
    std::cout << "chatter subscribed, about to call spin \n";
    ros::spin();

    stop_thread = true;
    // t1.join();
    return 0;
}
