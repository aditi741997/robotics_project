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

#include <dag.h>

using namespace mosek::fusion;
using namespace monty;

class MultiCoreApproxSolver
{
	public:
		MultiCoreApproxSolver();
		MultiCoreApproxSolver(DAG* ndag, int k);

		std::vector<std::vector<int> > solve();

	private:
		DAG* node_dag;
		int num_cores;
};
