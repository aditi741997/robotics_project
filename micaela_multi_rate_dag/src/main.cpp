/*
 * main.cpp
 *
 *  Created on: Apr 1, 2019
 *      Author: mirco
 */

#include <DAG/MaxProduct.h>
#include <Evaluation/Evaluation.h>
#include <VariableTaskSet/VariableTaskSet.h>
#include <eigen3/Eigen/Core>
#include <iostream>
#include "MultiRate/MultiRateTaskset.h"

#include <set>

#include "MultiRate/DummyNodes.h"

#include "Evaluation/Scheduling.h"
#include <algorithm>
void
taskset1()
{
	MultiRateTaskset taskSet;

	auto task1 = taskSet.addTask(10, 2, "sensor1");
	auto task2 = taskSet.addTask(10, 4, "sensor2");
	auto task3 = taskSet.addTask(20, 5, "controller");
	auto task4 = taskSet.addTask(20, 3, "actuator");

	taskSet.addPrecedenceEdge(task3, task4);
	taskSet.addDataEdge(task1, task3, 0);
	taskSet.addDataEdge(task2, task3, 1);

	taskSet.createBaselineDAG();

	auto dags = taskSet.createDAGs();

	dags[2].toTikz("test.tex");

}

int
taskset3()
{
	time_t tstart, tend;
	tstart = time(0);
	MultiRateTaskset taskSet;

	auto task1 = taskSet.addTask(5, 3, "imu");
	auto task2 = taskSet.addTask(10, 2, "gps");
	auto task3 = taskSet.addTask(10, 2, "planner");
	auto task4 = taskSet.addTask(5, 2, "controller");
	auto task5 = taskSet.addTask(20, 10, "act");

	task5->bcet = 5;
	task1->bcet = 1;
	//auto task6 = taskSet.addTask(40, 5, "train");
	//auto task7 = taskSet.addTask(160, 50, "independent");

	taskSet.addDataEdge(task3, task4, 0);

	taskSet.addDataEdge(task1, task3, 0);
	taskSet.addDataEdge(task1, task4, 1);
	taskSet.addDataEdge(task2, task3, 1);
	//taskSet.addDataEdge(task2, task4, 2);
	//taskSet.addDataEdge(task4, task5, 1);
	//taskSet.addDataEdge(task1, task6, 6);
	//taskSet.addDataEdge(task2, task6, 0);


	taskSet.createBaselineDAG();

	auto dags = taskSet.createDAGs();

	if (dags.empty())
		return 1;

	float numEdges = 10000;
	unsigned id = 0;
	unsigned k = 0;
	int n_processors = 4;

	for (auto& dag : dags)
	{
		auto info = dag.getLatencyInfo( { 0, 0, 2, 3, 4 });
		if (info.reactionTime < numEdges)
		{
			numEdges = info.reactionTime;
			id = k;
		}
		k++;

		scheduling::scheduleDAG(dag,n_processors);
	}

	dags[id].toTikz("prova.tex");
	scheduling::scheduleDAG(dags[id],n_processors,"schedule_test.tex", true);
	dags[id].getOriginatingTaskset()->toTikz("cool.tex");
	std::cout << dags[id].getNodeInfo() << std::endl;
	std::cout << dags[id].getLatencyInfo( { 0,4 }) << std::endl;
	//dags[id].getLatencyInfoIterative( { 1,5,3,4});

	tend = time(0);
	std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;

	return 0;

}

int
taskset2()
{
	MultiRateTaskset taskSet;

	auto task1 = taskSet.addTask(10, 9, "sensor1");
	auto task2 = taskSet.addTask(10, 5, "sensor2");
	auto task3 = taskSet.addTask(20, 2, "act");

	taskSet.addDataEdge(task1, task2, 1);
	taskSet.addDataEdge(task1, task3, 0);

//	taskSet.addPrecedenceEdge(task1, task2);

	taskSet.createBaselineDAG();

	auto dags = taskSet.createDAGs();

	return 0;
}

int
multiTaskset()
{
	time_t tstart, tend;
	tstart = time(0);
	VariableTaskSet taskSet;

	auto task1 = taskSet.addTask(5, 2.5, "imu");
	auto task2 = taskSet.addTask(20, 5.2, "gps");
	auto task3 = taskSet.addTask(10, 4.8, "planner");
	auto task4 = taskSet.addTask(10, 8, "controller");
	auto task5 = taskSet.addTask(20, 9, "act");
	auto task6 = taskSet.addTask(40, 13, "train");

	task6->bcet = 10;
//	task3->bcet = 4;
//	auto task7 = taskSet.addTask(80, 50, "independent");

	taskSet.addDataEdge(task3, task4, { 0, 1});

	taskSet.addDataEdge(task1, task3, { 0, 1, 2 });
	taskSet.addDataEdge(task1, task4, { 0, 1, 2 });
	taskSet.addDataEdge(task2, task3, { 0, 1, 2 });
	taskSet.addDataEdge(task2, task4, { 0, 1, 2 });
	taskSet.addDataEdge(task4, task5, { 0, 1, 2 });
//	taskSet.addDataEdge(task1, task6,  { 7, 8 });
//	taskSet.addDataEdge(task2, task6,  { 1,2 });
//	taskSet.addDataEdge(task5, task6,  { 1,2 });

	taskSet.createBaselineTaskset();

	auto& allDags = taskSet.createDAGs();

	std::cout << allDags.size() << " total valid DAGs were created" << std::endl;

	Evaluation eval;
	eval.addLatency({task1, task1,task3, task4, task5}, LatencyCost(1,15), LatencyConstraint(200,200));
	eval.addLatency({task2, task2, task3, task4, task5}, LatencyCost(1,3), LatencyConstraint(70,70));
//	eval.addLatency({task1, task3}, LatencyCost(1,1), LatencyConstraint(25,25));
	eval.addScheduling(SchedulingCost(20), SchedulingConstraint(4));

	const auto& bestDAG = eval.evaluate(allDags);

	bestDAG.toTikz("prova.tex");
	bestDAG.getOriginatingTaskset()->toTikz("cool.tex");
	std::cout << bestDAG.getNodeInfo()<< std::endl;
	bestDAG.getLatencyInfo({1,1,2,3,4});
//	scheduling::scheduleDAG(bestDAG, 4, "schedule_test.tex");

	tend = time(0);
	std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;

	return 0;
}

int serNavTaskset()
{
	std::cout << "Starting SER Nav Test" << std::endl;
	time_t tstart, tend;
	clock_t cb_start = clock();
	tstart = time(0);
	// MultiRateTaskset taskSet;
	VariableTaskSet taskSet;
	std::cout << "40,40,40 VT\n";
	auto sensor1 = taskSet.addTask(138, 1, "S");
	auto lcmp = taskSet.addTask(138, 11, "LC");
	auto lplan = taskSet.addTask(138, 18, "LP");

	auto gcmp = taskSet.addTask(276, 16, "GC");
	auto gplan = taskSet.addTask(276, 200, "GP");

	// taskSet.addDataEdge(sensor1, lcmp, 0);
	// taskSet.addDataEdge(lcmp, lplan, 0);

	taskSet.addDataEdge(sensor1, lcmp, {0});
	taskSet.addDataEdge(lcmp, lplan, {0});

	taskSet.addDataEdge(sensor1, gcmp, {0});
	taskSet.addDataEdge(gcmp, gplan, {0});
	taskSet.addDataEdge(gplan, lplan, {0});

	// taskSet.createBaselineDAG();
	taskSet.createBaselineTaskset();

	auto& allDags = taskSet.createDAGs();

	std::cout << allDags.size() << " total valid DAGs were created SER_NAV_frac_0.5" << std::endl;
	std::cout << "SER_NAV_frac_0.5 Node info for first DAG : " << allDags[0].getNodeInfo() << std::endl;

	Evaluation eval;
	// Put data age : combines latency and tput., rxn time (latency) : 
	eval.addLatency({sensor1, lcmp, lplan}, LatencyCost(-1,-1), LatencyConstraint(30, 30)); // local chain, exact constraints. THIS DOES NOT CAPTURE TPUT!!
	eval.addLatency({sensor1, gcmp, gplan, lplan}, LatencyCost(-1,-1), LatencyConstraint(444, 444)); // global chain, exact constraints.
	eval.addScheduling(SchedulingCost(10), SchedulingConstraint(1)); // SchedulingConstraint: #cores.

	const auto& bestDAG = eval.evaluate(allDags);
	std::cout << "Best DAG : \n" << bestDAG.getNodeInfo() << std::endl;
	tend = time(0);
	std::cout << "SER NAV : It took " << difftime(tend, tstart) << " second(s). RT : " <<  (double)(clock() - cb_start)/CLOCKS_PER_SEC << std::endl;

	// scheduling::scheduleDAG(bestDAG, 1, ); // schedule on 1 core.
}

void parSplit1Taskset()
{
	std::cout << "Starting Par Nav SPLIT1 Test" << std::endl;
	time_t tstart, tend;
	tstart = time(0);
	clock_t cb_start = clock();
	// MultiRateTaskset taskSet;
	VariableTaskSet taskSet;
	std::cout << "Putting GP wcet 100\n";

	auto sensor1 = taskSet.addTask(30, 1, "S");
	auto lcmp = taskSet.addTask(30, 11, "LC");
	auto gcmp = taskSet.addTask(116, 16, "GC");
	auto gplan1 = taskSet.addTask(116, 100, "GP1");
	auto gplan2 = taskSet.addTask(116, 100, "GP2");
	auto lplan = taskSet.addTask(30, 18, "LP");

	taskSet.addDataEdge(sensor1, lcmp, {0});
	taskSet.addDataEdge(lcmp, lplan, {0});

	taskSet.addDataEdge(sensor1, gcmp, {0});
	taskSet.addDataEdge(gcmp, gplan1, {0});
	taskSet.addDataEdge(gplan1, lplan, {0});
	taskSet.addDataEdge(gcmp, gplan2, {0});
	taskSet.addDataEdge(gplan2, lplan, {0});

	taskSet.createBaselineTaskset();

	auto& allDags = taskSet.createDAGs();

	std::cout << allDags.size() << " total valid DAGs were created PAR_NAV SPLIT1" << std::endl;
	if (allDags.size() > 0)
		std::cout << "PAR_NAV Node info for first DAG : " << allDags[0].getNodeInfo() << std::endl;

	Evaluation eval;
	// Put data age, rxn time (latency) : 
	eval.addLatency({sensor1, lcmp, lplan}, LatencyCost(-1,-1), LatencyConstraint(30, 30)); // local chain, exact constraints. THIS DOES NOT CAPTURE TPUT!!
	eval.addLatency({sensor1, gcmp, gplan1, lplan}, LatencyCost(-1,-1), LatencyConstraint(326, 326)); // global chain, exact constraints.
	eval.addLatency({sensor1, gcmp, gplan2, lplan}, LatencyCost(-1,-1), LatencyConstraint(326, 326)); // global chain, exact constraints.
	eval.addScheduling(SchedulingCost(10), SchedulingConstraint(3)); // 3 cores. split into 2 bins : 1 for Lchain, 1 for Gchain.

	const auto& bestDAG = eval.evaluate(allDags);
	// std::cout << "PAR_NAV Split1 Best DAG : \n" << bestDAG.getNodeInfo()<< std::endl;

	tend = time(0);
	std::cout << "PAR NAV SPLIT1 : It took " << difftime(tend, tstart) << " second(s). RT : " <<  (double)(clock() - cb_start)/CLOCKS_PER_SEC << std::endl;

}

void parSplit2Taskset()
{
	std::cout << "Starting Par Nav SPLIT2 Test" << std::endl;
	time_t tstart, tend;
	tstart = time(0);
	clock_t cb_start = clock();
	// MultiRateTaskset taskSet;
	VariableTaskSet taskSet;
	std::cout << "Putting GP wcet 200\n";

	auto sensor1 = taskSet.addTask(18, 1, "S");
	auto lcmp = taskSet.addTask(18, 11, "LC");
	auto lplan = taskSet.addTask(18, 18, "LP");

	auto gcmp = taskSet.addTask(216, 16, "GC");
	auto gplan = taskSet.addTask(216, 200, "GP");

	taskSet.addDataEdge(sensor1, lcmp, {0});
	taskSet.addDataEdge(lcmp, lplan, {0});

	taskSet.addDataEdge(sensor1, gcmp, {0});
	taskSet.addDataEdge(gcmp, gplan, {0});
	taskSet.addDataEdge(gplan, lplan, {0});

	taskSet.createBaselineTaskset();

	auto& allDags = taskSet.createDAGs();

	std::cout << allDags.size() << " total valid DAGs were created PAR_NAV Split2 " << std::endl;
	if (allDags.size() > 0)
		std::cout << "PAR_NAV SPLIT2 Node info for first DAG : " << allDags[0].getNodeInfo() << std::endl;

	Evaluation eval;
	// Put data age, rxn time (latency) : 
	eval.addLatency({sensor1, lcmp, lplan}, LatencyCost(-1,-1), LatencyConstraint(30, 30)); // local chain, exact constraints. THIS DOES NOT CAPTURE TPUT!!
	eval.addLatency({sensor1, gcmp, gplan, lplan}, LatencyCost(-1,-1), LatencyConstraint(510, 510)); // global chain, exact constraints.
	eval.addScheduling(SchedulingCost(10), SchedulingConstraint(3)); // 3 cores. split into 2 bins : 1 for Lchain, 1 for Gchain.

	const auto& bestDAG = eval.evaluate(allDags);
	// std::cout << "PAR_NAV Split2 Best DAG : \n" << bestDAG.getNodeInfo()<< std::endl;

	tend = time(0);
	std::cout << "PAR NAV SPLIT2 It took " << difftime(tend, tstart) << " second(s). RT : " <<  (double)(clock() - cb_start)/CLOCKS_PER_SEC << std::endl;
}

int
multiTaskset2()
{
	time_t tstart, tend;
	tstart = time(0);
	VariableTaskSet taskSet;

	auto task1 = taskSet.addTask(5, 3, "imu");
	auto task2 = taskSet.addTask(20, 4, "planner");

	task2->bcet = 3;

	taskSet.addDataEdge(task1, task2, { 0, 1, 2,3,4 });

	taskSet.createBaselineTaskset();

	auto& allDags = taskSet.createDAGs();

	std::cout << allDags.size() << " total valid DAGs were created" << std::endl;

	Evaluation eval;
	eval.addLatency({task2, task1, task1}, LatencyCost(-1,-1), LatencyConstraint(60, 60));

	const auto& bestDAG = eval.evaluate(allDags);

	bestDAG.toTikz("prova.tex");
	bestDAG.getOriginatingTaskset()->toTikz("cool.tex");

	std::cout << bestDAG.getNodeInfo()<< std::endl;

	tend = time(0);
	std::cout << "It took " << difftime(tend, tstart) << " second(s)." << std::endl;

	return 0;
}

int
main()
{
	// multiTaskset();
	// serNavTaskset();
	parSplit1Taskset();
	// parSplit2Taskset();
	return 0;

}

