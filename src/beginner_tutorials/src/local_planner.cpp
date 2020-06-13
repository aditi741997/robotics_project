#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <sstream>
#include <fstream>
#include <time.h>
#include <bits/stdc++.h>
#include <sys/times.h>

// subscribe to odom and Lcmp and GPlan.
// measure rxn time w.r.t. all
// cb (odom) only updates the shared variable and hence does nothing.
// E.D. w.r.t. GPlan if RTC and w.r.t. LCmp if Dyn.
// TODO : See if we need to make separate queues for all callbacks.
class LocalPlanner
{
    double last_lcmp_ts, last_gplan_ts, last_odom_ts;

    double compute_rt_sum, compute_ros_sum;
    std::vector<double> compute_rt_arr, compute_ros_arr;

    double lat_lcmp_sum, lat_gplan_sum, lat_odom_sum;
    std::vector<double> lat_lcmp_arr, lat_gplan_arr, lat_odom_arr;

    double ts_last_gplan_used, ts_last_odom_used, ts_last_lcmp_used; // TS of the globalPlan used by the last exec of LPlan. 
    double last_lcmp_out_ts, last_gplan_out_ts, last_odom_out_ts; // Note when was the last time LPlan got a new output w.r.t LCMp/GPlan/Odom.

    // If current LPlan is using new GPlan or odom, update tput
    double tput_lcmp_sum, tput_gplan_sum, tput_odom_sum;
    std::vector<double> tput_lcmp_arr, tput_gplan_arr, tput_odom_arr;

    // rxnTm w.r.t. 3 chains :
    double rxntm_lcmp_sum, rxntm_gplan_sum, rxntm_odom_sum;
    std::vector<double> rxntm_lcmp_arr, rxntm_gplan_arr, rxntm_odom_arr;

    int climit; // for compute time
    bool rtc; // rtc vs fractional

    int total_exec_count, total_odom_cb_count, total_gplan_cb_count, total_lcmp_cb_count;

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber lcmp_sub, odom_sub, gplan_sub;

    int perc7 = 75;
    int percentile = 95;
    int perc0 = 90;
    int perc2 = 99;
    float perc3 = 99.9;
    float perc4 = 99.99; 

public:
    LocalPlanner(int lim, bool algo, std::string lcmp_t, std::string odom_t, std::string gplan_t)
    {
        climit = lim;
        rtc = algo;

        ts_last_gplan_used = -1.0;
        ts_last_odom_used = -1.0;
        ts_last_lcmp_used = -1.0;

        last_lcmp_ts = -1.0;
        last_gplan_ts = -1.0;
        last_odom_ts = -1.0;

        total_exec_count = 0;
        total_odom_cb_count = 0;
        total_gplan_cb_count = 0;
        total_lcmp_cb_count = 0;

	last_odom_out_ts = 0.0;
	last_gplan_out_ts = 0.0;
	last_lcmp_out_ts = 0.0;

        // Subscribe to all 3!
        lcmp_sub = nh.subscribe(lcmp_t, 1, &LocalPlanner::lcmpCallback, this, ros::TransportHints().tcpNoDelay(), true);
        odom_sub = nh.subscribe(odom_t, 1, &LocalPlanner::odomCallback, this, ros::TransportHints().tcpNoDelay());
        gplan_sub = nh.subscribe(gplan_t, 1, &LocalPlanner::gplanCallback, this, ros::TransportHints().tcpNoDelay(), true);
        
    }

    void odomCallback(const std_msgs::Header::ConstPtr& msg)
    {
        // Will need lock if different callbacks have different queues
        // Default : One single cb queue hence no need to lock.
        last_odom_ts = msg->stamp.toSec();
        total_odom_cb_count += 1;
        // DO nothing!
    }

    void gplanCallback(const std_msgs::Header::ConstPtr& msg)
    {
        // Will need lock if different callbacks have different queues
        // Default : One single cb queue hence no need to lock.
        last_gplan_ts = msg->stamp.toSec();
        total_gplan_cb_count += 1;
        if (rtc)
        {
            // RTC Means LPlan must be E.D. w.r.t. GPlan.
            exec();
        }
    }

    void calcPrimes()
    {
        int i, num = 1, primes = 0;

        while (num <= climit) { 
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

        ROS_INFO("Found %i primes", primes);
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
        
            ROS_INFO("Median, %i tail of %s is %f %f , arr sz %i , %i : %f, %i : %f, %i : %f, %f : %f, %f : %f #", percentile, m.c_str(), med, perc, l, perc7, m_arr[(l*perc7)/100], perc0, m_arr[(l*perc0)/100], perc2, m_arr[(l*perc2)/100], perc3, m_arr[(l*perc3)/100], perc4, m_arr[(l*perc4)/100]);            
        }
    }

    void exec()
    {
        total_exec_count += 1;

        double ros_start = ros::Time::now().toSec();
        clock_t rt_start = clock();
        // This LPlan cb will use the gplan, odom, lcmp with these TS:
        double using_gplan_ts = last_gplan_ts;
        double using_odom_ts = last_odom_ts;
        double using_lcmp_ts = last_lcmp_ts;

        double lat_lcmp = (ros::Time::now()).toSec() - using_lcmp_ts;
        double lat_gplan = (ros::Time::now()).toSec() - using_gplan_ts;
        double lat_odom = ros::Time::now().toSec() - using_odom_ts;

        calcPrimes();
        // TODO : publish cmd_vel.

        if (using_gplan_ts > ts_last_gplan_used)
        {
            // Using a new GPLan : Add lat, tput rxnTm LCMP
            lat_gplan_sum += lat_gplan;
            lat_gplan_arr.push_back(lat_gplan);

            if (last_gplan_out_ts > 0.0)
            {
                double tput_gplan = ros::Time::now().toSec() - last_gplan_out_ts;
                // ROS_INFO("Second output wrt GPlan, last out %f, tput %f", last_gplan_out_ts, tput_gplan);
		tput_gplan_arr.push_back(tput_gplan);
                tput_gplan_sum += tput_gplan;                
            }

            if (tput_gplan_arr.size() > 2)
            {
                // add rxnTime
                double rt = ros::Time::now().toSec() - ts_last_gplan_used;
                rxntm_gplan_arr.push_back(rt);
                rxntm_gplan_sum += rt;
            }

            ts_last_gplan_used = using_gplan_ts;
            last_gplan_out_ts = ros::Time::now().toSec();
        }
        if (using_odom_ts > ts_last_odom_used)
        {
            lat_odom_sum += lat_odom;
            lat_odom_arr.push_back(lat_odom);

            if (last_odom_out_ts > 0.0)
            {
                double tput_odom = ros::Time::now().toSec() - last_odom_out_ts;
                // ROS_INFO("Second output wrt Odom, last out %f, tput %f", last_odom_out_ts, tput_odom);
                tput_odom_arr.push_back(tput_odom);
                tput_odom_sum += tput_odom;
            }

            if (tput_odom_arr.size() > 2)
            {
                double rt = ros::Time::now().toSec() - ts_last_odom_used;
                rxntm_odom_arr.push_back(rt);
                rxntm_odom_sum += rt;
            }

            last_odom_out_ts = ros::Time::now().toSec();
            ts_last_odom_used = using_odom_ts;
        }
        if (using_lcmp_ts > ts_last_lcmp_used)
        {
            lat_lcmp_sum += lat_lcmp;
            lat_lcmp_arr.push_back(lat_lcmp);

            if (last_lcmp_out_ts > 0.0)
            {
                double tput_lcmp = ros::Time::now().toSec() - last_lcmp_out_ts;
                // ROS_INFO("Second output wrt Lcmp, last out %f, tput %f", last_lcmp_out_ts, tput_lcmp);
                tput_lcmp_arr.push_back(tput_lcmp);
                tput_lcmp_sum += tput_lcmp;
            }

            if (tput_lcmp_arr.size() > 2)
            {
                double rt = ros::Time::now().toSec() - ts_last_lcmp_used;
                rxntm_lcmp_arr.push_back(rt);
                rxntm_lcmp_sum += rt;
            }

            last_lcmp_out_ts = ros::Time::now().toSec();
            ts_last_lcmp_used = using_lcmp_ts;
        }

        double ros_ci = ros::Time::now().toSec() - ros_start;
        double rt_ci = (double)(clock() - rt_start)/CLOCKS_PER_SEC;

        compute_rt_arr.push_back(rt_ci);
        compute_rt_sum += rt_ci;

        compute_ros_arr.push_back(ros_ci);
        compute_ros_sum += ros_ci;

        if (total_exec_count%9 == 7)
        {
	    std::cout << "compute ros sum : " << compute_ros_sum << std::endl;
            print_smt(compute_ros_sum, compute_ros_arr, "Local Planner Exec ROS time");
            print_smt(compute_rt_sum, compute_rt_arr, "Local Planner Exec Real time");
	    print_smt(lat_lcmp_sum, lat_lcmp_arr, "Lat w.r.t. LCMP chain (exc LPlan)");
	    print_smt(lat_gplan_sum, lat_gplan_arr, "Lat w.r.t. GPlan chain (exc LPlan)");
	    print_smt(lat_odom_sum, lat_odom_arr, "Lat w.r.t. Odom chain (exc LPlan)");
	    print_smt(tput_lcmp_sum, tput_lcmp_arr, "Tput w.r.t. LCMP chain");	    
	    print_smt(tput_gplan_sum, tput_gplan_arr, "Tput w.r.t. GPLan chain");	    
	    print_smt(tput_odom_sum, tput_odom_arr, "Tput w.r.t. Odom chain");	    
            print_smt(rxntm_lcmp_sum, rxntm_lcmp_arr, "RxnTm for Scan-LCMp-LPlan chain");
	    print_smt(rxntm_gplan_sum, rxntm_gplan_arr, "RxnTm for Scan-GCmp-GPlan-LPlan chain");
	    print_smt(rxntm_odom_sum, rxntm_odom_arr, "RxnTm for Odom-LPlan chain");
	}
    }

    void lcmpCallback(const std_msgs::Header::ConstPtr& msg)
    {
        last_lcmp_ts = msg->stamp.toSec();
        total_lcmp_cb_count += 1;
        // If not RTC, i.e. fractional. E.D. w.r.t. Lcmp
        if (!rtc)
            exec();
    }
};

int main(int argc, char **argv)
{
    int lim = atoi(argv[1]);
    bool rtc = (atoi(argv[2]) == 1);
    std::string lcmp_topic = argv[3];
    std::string odom_topic = argv[4];
    std::string gplan_topic = argv[5];
    ros::init(argc, argv, "dummyLPlan");
    LocalPlanner lp(lim, rtc, lcmp_topic, odom_topic, gplan_topic);
    ros::spin();
    return 0;
}
