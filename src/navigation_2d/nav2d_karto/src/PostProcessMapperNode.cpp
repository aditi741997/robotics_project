#include <ros/ros.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>
#include <nav2d_karto/SpaSolver.h>
#include <thread>

#include <sstream>
#include <string>
#include <unistd.h>
#include <cmath>
#include <map>

#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

void print_tf(tf::Transform t, std::string s)
{
	printf("TF %s : x: %f, y: %f, z: %f, R: x: %f, y: %f, z: %F, w: %f \n", s.c_str(), t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(), t.getRotation().x(), t.getRotation().y(), t.getRotation().z(), t.getRotation().w());
}

int main(int argc, char **argv)
{
	        // Initialize ROS
		ros::init(argc, argv, "MultiMapper");
		ros::NodeHandle node;
		
		// Create a scan-solver
		SpaSolver* solver = new SpaSolver();

		ros::Publisher pub1 = node.advertise<std_msgs::Header>("/robot_0/exec_start_mcb_pp", 100, true);
		MultiMapper mapper(pub1);
	        mapper.setScanSolver(solver);


		// first get all the times when mapU ran, with the TS of last scan it used:
		std::ifstream old_mu_log (argv[1]);
		// std::ofstream new_mu_log (argv[2]); Now taking ScanBag name as input:
		
		rosbag::Bag bag;
		bag.open(argv[2], rosbag::bagmode::Read);

		std::vector<sensor_msgs::LaserScan> scans;
		// store all TS for which there is a scan, so we can store corresponding posn.
		std::map<int,int> ts_ind;	

		std::vector<std::string> topics;
		topics.push_back(std::string("/robot_0/scans"));

		rosbag::View view(bag, rosbag::TopicQuery(topics));

		foreach(rosbag::MessageInstance const m, view)
		{
			sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
			
			sensor_msgs::LaserScan latest_scan_recv;
			latest_scan_recv.header.frame_id = scan->header.frame_id;
	                latest_scan_recv.header.seq = scan->header.seq;
			latest_scan_recv.header.stamp = scan->header.stamp;

			latest_scan_recv.angle_min = scan->angle_min;
			latest_scan_recv.angle_max = scan->angle_max;
			latest_scan_recv.angle_increment = scan->angle_increment;
			latest_scan_recv.time_increment = scan->time_increment;
			latest_scan_recv.scan_time = scan->scan_time;
			latest_scan_recv.range_min = scan->range_min;
			latest_scan_recv.range_max = scan->range_max;
			latest_scan_recv.intensities.clear();
																							                
			for (int i = 0; i < scan->ranges.size(); i++)
				latest_scan_recv.ranges.push_back(scan->ranges[i]);
																											               
			for (int i = 0; i < scan->intensities.size(); i++)
				latest_scan_recv.intensities.push_back(scan->intensities[i]);
			
			scans.push_back(latest_scan_recv);
			
			int int_ts = (int)(10*latest_scan_recv.header.stamp.toSec());
			ts_ind[int_ts] = scans.size() - 1;
			std::cout << scan->header.stamp.toSec() << ", " << int_ts << ", " << ts_ind[int_ts] << std::endl;
		}
		bag.close();

		// read poses saved by mapper: for localzn error wrt GT poses.
		// for each TS (of mapper exec) -> save [avg,med,var,25ile,75ile] for x,y,angle error.

		std::vector<tf::StampedTransform> gtposes; // pose corresponding to each scan.
		std::vector<sensor_msgs::LaserScan> scans2; // store the scans for which we do have a pose.

		std::string txto;
		std::ifstream stg_gt_log (argv[3]);
		int line_ct = std::atoi(argv[4]); // num_obst+3
		float startx = std::atof(argv[5]);
		float starty = std::atof(argv[6]);
		int ind = 0;
		while (std::getline(stg_gt_log, txto) )
		{
			// get line_ct-1 more lines.
			std::vector<std::string> txts (1, txto);
			for (int i = 1; i < line_ct; i++)
			{
				std::string ti;
				std::getline(stg_gt_log, ti);
				txts.push_back(ti);
			}
			// for TS:
			std::stringstream ssts(txts[0]);
			std::string s;
			float st_ts;
			ssts >> s >> st_ts;
			int scaled_ts = 10*st_ts;
			if (ts_ind.find(scaled_ts) != ts_ind.end())
			{
				if (ts_ind[scaled_ts] > ind)
					printf("WEIRD, Missed ind %i, current ind %i", ind, ts_ind[scaled_ts]);
				// get x,y pos
				double rx, ry;
				std::stringstream ssxy (txts[1]);
				ssxy >> s >> rx >> ry;

				// get orientation
				std::stringstream ssor (txts[line_ct-1]);
				double rz,rw;
				ssor >> s >> s >> s >> s >> s >> s >> rz >> rw;
				
				printf("FOUND robopos x: %f, y: %f, z: %f, w: %f for scan ts %i ind %i \n", rx,ry,rz,rw,scaled_ts, ts_ind[scaled_ts]);
				
				tf::Transform gtPos;
				gtPos.setRotation(tf::Quaternion(0,0,rz,rw) );
				gtPos.setOrigin(tf::Vector3(0,0,0));				

				tf::Transform rotate;
				rotate.setRotation(tf::Quaternion(0,0,-0.382683,0.92388));

				// print_tf(gtPos, "gtPos original");
				gtPos *= rotate;
				// print_tf(gtPos, "gt pos after rotating orientation by -45deg");

				double newx = ((rx-startx)*sqrt(0.5) + (ry+starty)*sqrt(0.5)) + 0.2;
				double newy = ((rx-startx)*sqrt(0.5)*-1.0 + (ry+starty)*sqrt(0.5));
				double newz = 0.0 + 0.2;

				gtPos.setOrigin(tf::Vector3(newx,newy,newz));
				print_tf(gtPos, "FINAL gtPos");

				gtposes.push_back( tf::StampedTransform(gtPos, ros::Time(st_ts), "/robot_0/odom", "/robot_0/base_laser_link" ) );
				scans2.push_back(scans[ts_ind[scaled_ts]]);
				ind = ts_ind[scaled_ts] + 1;
			}
		}
		
		std::string txt;
		int start_ind = 0;
		// while (std::getline(old_mu_log, txt) )
		{
			// if (txt.find("In MultiMapper::updateMap Current ratio") != std::string::npos)
			while (start_ind < scans2.size() )
			{
				/*
				std::stringstream ss (txt);
				std::string s0,s1,s2,s3,s4,s5,s6;
				ss >> s0 >> s1 >> s2 >> s3 >> s4 >> s5 >> s6;
				// TS of when this was run
				double mu_ts = std::stod(s4);
				// TS of last scan used
				int last_scan_ts = std::stoi(s6);
				
				// call processlatestScans with subset of scans, posns.
				int copy_until_ind = ts_ind[last_scan_ts]; */
				int copy_until_ind = start_ind + std::min(25, (int)(scans2.size() - start_ind - 1) );
				std::vector<sensor_msgs::LaserScan> scans_i (scans2.begin()+start_ind, scans2.begin()+copy_until_ind+1);
				std::vector<tf::StampedTransform> gtposes_i (gtposes.begin()+start_ind, gtposes.begin()+copy_until_ind+1);
				
				// printf("\n MU Exec at TS: %f, used scans with latestTS: %i, will processScans frm ind %i to %i [vec sz: %i]", mu_ts, last_scan_ts, start_ind, copy_until_ind, scans_i.size());
				
				printf("\n PROCESSING scans from ind %i to %i (TS: %f), vec size: %i", start_ind, copy_until_ind, scans[copy_until_ind].header.stamp.toSec(), scans_i.size() );
				// process scans : mcb code
				mapper.processLatestScans( scans_i, gtposes_i );
				// generate map : mu code
				mapper.sendMap();
				start_ind = copy_until_ind+1;
			}
		}
		// publish the final map on _pp topic.
		printf("DONE with reading the file!!!! \n");
		mapper.publishMap("/robot_0/ppmap");
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
		exit(0);
}
