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

class DAGControllerFE
{
public:
	DAGControllerFE() {};
	
	virtual void trigger_node(std::string name, bool reset) {};

	virtual int get_sim_time() { return 0; };

	virtual void update_speriod(double x, std::string name) {};
};
