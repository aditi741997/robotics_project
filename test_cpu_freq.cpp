#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include<algorithm>
#include <tuple>
#include <unordered_map>
#include <set>
#include <chrono>
#include <assert.h>
#include <math.h>
#include <boost/circular_buffer.hpp>
#include <sys/types.h>
#include <signal.h>
#include <sched.h>
#include <pthread.h>
#include <linux/sched.h>
#include<thread>


void sieve(int limit)
{
	int i, num = 1, primes = 0;

	while (num <= limit) 
	{
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
	printf("Found %i primes", primes);
}

int main(int argc, char **argv)
{
	std::vector<double> cpu_times;
	double cpu_times_sum = 0.0;
	int ct = atoi(argv[1]);
	int num = atoi(argv[2]);

	for (int i = 0; i < ct; i++)
	{
		struct timespec solve_start, solve_end;
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_start);
		sieve(num);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_end);
		double t = (solve_end.tv_sec + solve_end.tv_nsec*1e-9) - (solve_start.tv_sec + 1e-9*solve_start.tv_nsec);
		cpu_times.push_back( t );
		cpu_times_sum += t;
	}

	std::vector<int> sleep_times_microsec {1000, 3000, 800, 400, 300};
	for (int si = 0; si < sleep_times_microsec.size(); si++)
	{
		std::vector<int> actual_sleep_ns;
		std::vector<double> cpu_t;
		long int sum_st = 0;
		double cpu_t_s = 0.0;
		for (int i = 0; i < ct; i++)
		{
			long int start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count();
			struct timespec solve_start, solve_end;
			clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_start);
			std::this_thread::sleep_for( std::chrono::microseconds(sleep_times_microsec[si]) );
			long int end = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch() ).count();
			clock_gettime(CLOCK_THREAD_CPUTIME_ID, &solve_end);
			
			actual_sleep_ns.push_back(end-start);
			sum_st += (end-start);

			double t = (solve_end.tv_sec + solve_end.tv_nsec*1e-9) - (solve_start.tv_sec + 1e-9*solve_start.tv_nsec);
			cpu_t_s += t;
			cpu_t.push_back(t);
		}
		std::sort(actual_sleep_ns.begin(), actual_sleep_ns.end());
		int sl = actual_sleep_ns.size();
		printf("FOR SleepTime=%i microsec, actual sleep_for sleep times Median %i, mean %i, 75p %i, 95p %i, \n", sleep_times_microsec[si], actual_sleep_ns[sl/2], sum_st/sl, actual_sleep_ns[(75*sl)/100], actual_sleep_ns[(95*sl)/100] );
	
		std::sort(cpu_t.begin(), cpu_t.end());
		printf("FOR SleepTime=%i microsec, cpu used: median %f, mean %f, 75p %f, 95p %f \n", sleep_times_microsec[si], cpu_t[sl/2], cpu_t_s/sl, cpu_t[(75*sl)/100], cpu_t[(95*sl)/100]);
	}	
	std::sort(cpu_times.begin(), cpu_times.end());
	int l = cpu_times.size();
	printf("FINISHED %i runs for sieve(%i), Mean %f, Median %f, 75ile %f, 90ile %f, 95ile %f, 99ile %f, \n", ct,num, cpu_times_sum/l, cpu_times[l/2], cpu_times[(75*l)/100], cpu_times[(90*l)/100], cpu_times[(95*l)/100], cpu_times[(99*l)/100] );
}
