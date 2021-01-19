#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <sstream>
#include<algorithm>
#include <tuple>
#include <unordered_map>
#include "fusion.h"
#include <set>
#include <assert.h>
#include <math.h>
#include <boost/circular_buffer.hpp>
#include <utility>

#include <dag.h>

class MultiCoreBaselineSolver
{
	public:
		MultiCoreBaselineSolver();

		void solve(DAG& ndag, int numcores);
};
