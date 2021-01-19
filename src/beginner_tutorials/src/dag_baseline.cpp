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
#include <map>

#include <dag_baseline.h>

MultiCoreBaselineSolver::MultiCoreBaselineSolver(){};

void MultiCoreBaselineSolver::solve(DAG& node_dag, int numcores)
{
	int k = numcores;

}


