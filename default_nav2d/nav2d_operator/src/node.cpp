#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <nav2d_operator/RobotOperator.h>

using namespace ros;

void print_arr(std::vector<double> m_arr, std::string m)
{
        int perc7 = 75;
    int percentile = 95;
    int perc0 = 90;
    int perc2 = 99;
    float perc3 = 99.9;
    float perc4 = 99.99;

        int l = m_arr.size();
    if (l > 0)
    {
        std::sort(m_arr.begin(), m_arr.end());
        double avg = (std::accumulate(m_arr.begin(), m_arr.end(), 0.0))/l;
        double med = m_arr[l/2];
        double perc = m_arr[(l*percentile)/100];

        ROS_ERROR("Mean %f, Median, %i tail of %s is %f %f , arr sz %i , %i : %f, %i : %f, %i : %f, %f : %f, %f : %f #", avg, percentile, m.c_str(), med, perc, l, perc7, m_arr[(l*perc7)/100], perc0, m_arr[(l*perc0)/100], perc2, m_arr[(l*perc2)/100], perc3, m_arr[(l*perc3)/100], perc4, m_arr[(l*perc4)/100]);
    }
}

double get_time_now()
{
        /*
        boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
        boost::chrono::system_clock::duration tse = now.time_since_epoch();
        //using system_clock to measure latency:
        unsigned long long ct = boost::chrono::duration_cast<boost::chrono::milliseconds>(tse).count() - (1603000000000);
        double time_now = ct / (double)(1000.0);
        */

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double time_now = ts.tv_sec + 1e-9*ts.tv_nsec;

        return time_now;
}

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n;

	RobotOperator robOp;
	
	WallRate loopRate(10);

	double last_lp_out;
	long int total_lp_ct = 0;
	std::vector<double> tput_lp;	

	while(ok())
	{
		robOp.executeCommand();
		spinOnce();
	
		double time_now = get_time_now();
		total_lp_ct += 1;
		if (total_lp_ct > 1)
			tput_lp.push_back(time_now - last_lp_out);
		last_lp_out = time_now;

		if (tput_lp.size()%500 == 177)
			print_arr(tput_lp, "LP TPUT!!!");

		loopRate.sleep();
	}
	return 0;	
}
