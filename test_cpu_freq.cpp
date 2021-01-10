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
#include <assert.h>
#include <math.h>
#include <boost/circular_buffer.hpp>

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

	std::sort(cpu_times.begin(), cpu_times.end());
	int l = cpu_times.size();
	printf("FINISHED %i runs for sieve(%i), Mean %f, Median %f, 75ile %f, 90ile %f, 95ile %f, 99ile %f", ct,num, cpu_times_sum/l, cpu_times[l/2], cpu_times[(75*l)/100], cpu_times[(90*l)/100], cpu_times[(95*l)/100], cpu_times[(99*l)/100] );
}
