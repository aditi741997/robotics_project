/*
 *  stageros
 *  Copyright (c) 2008, Willow Garage, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**

@mainpage

@htmlinclude manifest.html
**/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <signal.h>

#include <fstream>
#include <sstream>

// libstage
#include <stage.hh>

// roscpp
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>

#include <std_srvs/Empty.h>

#include "tf/transform_broadcaster.h"

// for getting system clock time
#include <boost/chrono/system_clocks.hpp>
#include <boost/chrono/ceil.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>

#define USAGE "stageros <worldfile>"
#define IMAGE "image"
#define DEPTH "depth"
#define CAMERA_INFO "camera_info"
#define ODOM "odom"
#define BASE_SCAN "base_scan1"
#define BASE_POSE_GROUND_TRUTH "base_pose_ground_truth"
#define CMD_VEL "cmd_vel"

#include <ros/console.h>

#include <rosbag/bag.h>

// Our node
class StageNode
{
private:

    // roscpp-related bookkeeping
    ros::NodeHandle n_;

    // A mutex to lock access to fields that are used in message callbacks
    boost::mutex msg_lock;

    // The models that we're interested in
    std::vector<Stg::ModelCamera *> cameramodels;
    std::vector<Stg::ModelRanger *> lasermodels;
    std::vector<Stg::ModelPosition *> positionmodels;

    //a structure representing a robot inthe simulator
    struct StageRobot
    {
        //stage related models
        Stg::ModelPosition* positionmodel; //one position
        std::vector<Stg::ModelCamera *> cameramodels; //multiple cameras per position
        std::vector<Stg::ModelRanger *> lasermodels; //multiple rangers per position

        //ros publishers
        ros::Publisher odom_pub; //one odom
        ros::Publisher ground_truth_pub; //one ground truth 
	ros::Publisher odom_ts_pub; // publishes the real TS for when is odom published.

        std::vector<ros::Publisher> image_pubs; //multiple images
        std::vector<ros::Publisher> depth_pubs; //multiple depths
        std::vector<ros::Publisher> camera_pubs; //multiple cameras
        std::vector<ros::Publisher> laser_pubs; //multiple lasers
        ros::Subscriber cmdvel_sub; //one cmd_vel subscriber
	
    };

    std::vector<StageRobot const *> robotmodels_;

    // Used to remember initial poses for soft reset
    std::vector<Stg::Pose> initial_poses;
    ros::ServiceServer reset_srv_;
  
    ros::Publisher clock_pub_;
    
    bool isDepthCanonical;
    bool use_model_names;

    // A helper function that is executed for each stage model.  We use it
    // to search for models of interest.
    static void ghfunc(Stg::Model* mod, StageNode* node);

    static bool s_update(Stg::World* world, StageNode* node)
    {
        node->WorldCallback();
        // We return false to indicate that we want to be called again (an
        // odd convention, but that's the way that Stage works).
        return false;
    }

    // Appends the given robot ID to the given message name.  If omitRobotID
    // is true, an unaltered copy of the name is returned.
    const char *mapName(const char *name, size_t robotID, Stg::Model* mod) const;
    const char *mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const;

    tf::TransformBroadcaster tf;

    // Last time that we received a velocity command
    ros::Time base_last_cmd;
    ros::Duration base_watchdog_timeout;

    // Current simulation time
    ros::Time sim_time;
    
    // Last time we saved global position (for velocity calculation).
    ros::Time base_last_globalpos_time;
    // Last published global pose of each robot
    std::vector<Stg::Pose> base_last_globalpos;

public:
    // Constructor; stage itself needs argc/argv.  fname is the .world file
    // that stage should load.
    StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names);
    ~StageNode();

    // Subscribe to models of interest.  Currently, we find and subscribe
    // to the first 'laser' model and the first 'position' model.  Returns
    // 0 on success (both models subscribed), -1 otherwise.
    int SubscribeModels();

    // Our callback
    void WorldCallback();
    
    // Do one update of the world.  May pause if the next update time
    // has not yet arrived.
    bool UpdateWorld();

    // Message callback for a MsgBaseVel message, which set velocities.
    void cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg);

    // Service callback for soft reset
    bool cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // The main simulator object
    Stg::World* world;

    // For nav2d scheduling experiments:
    rosbag::Bag robo0_scans_bag;
    std::vector<sensor_msgs::LaserScan> robo0_last_five_scans; // store last 5scans published, to be written to bagfile.
    std::vector<std::string> stalled_robos; // to see if robo collided with one of the moving robots.
    std::vector<double > all_robos_pose_x;
    std::vector<double > all_robos_pose_y;
    std::vector<double > all_robos_twist_lin_z, all_robos_twist_lin_y, all_robos_twist_lin_x, all_robos_twist_ang_z, all_robos_orient_z, all_robos_orient_w;
    bool rob0_stalled = false;
    std::string expt_name;
    int world_update_count;
};

// since stageros is single-threaded, this is OK. revisit if that changes!
const char *
StageNode::mapName(const char *name, size_t robotID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s", robotID, name);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s", ((Stg::Ancestor *) mod)->Token(), name);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s", (unsigned int)robotID, name);
        }

        return buf;
    }
    else
        return name;
}

const char *
StageNode::mapName(const char *name, size_t robotID, size_t deviceID, Stg::Model* mod) const
{
    //ROS_INFO("Robot %lu: Device %s:%lu", robotID, name, deviceID);
    bool umn = this->use_model_names;

    if ((positionmodels.size() > 1 ) || umn)
    {
        static char buf[100];
        std::size_t found = std::string(((Stg::Ancestor *) mod)->Token()).find(":");

        if ((found==std::string::npos) && umn)
        {
            snprintf(buf, sizeof(buf), "/%s/%s_%u", ((Stg::Ancestor *) mod)->Token(), name, (unsigned int)deviceID);
        }
        else
        {
            snprintf(buf, sizeof(buf), "/robot_%u/%s_%u", (unsigned int)robotID, name, (unsigned int)deviceID);
        }

        return buf;
    }
    else
    {
        static char buf[100];
        snprintf(buf, sizeof(buf), "/%s_%u", name, (unsigned int)deviceID);
        return buf;
    }
}

void
StageNode::ghfunc(Stg::Model* mod, StageNode* node)
{
  //printf( "inspecting %s, parent\n", mod->Token() );

  if (dynamic_cast<Stg::ModelRanger *>(mod)) {
     node->lasermodels.push_back(dynamic_cast<Stg::ModelRanger *>(mod));
  }
  if (dynamic_cast<Stg::ModelPosition *>(mod)) {
     Stg::ModelPosition * p = dynamic_cast<Stg::ModelPosition *>(mod);
      // remember initial poses
      node->positionmodels.push_back(p);
      ROS_ERROR("IN STAGE_ROS, Pushing to positionmodels x: %f, y: %f, z: %f, token: %s", p->est_pose.x, p->est_pose.y, p->est_pose.z, p->Token());
      node->initial_poses.push_back(p->GetGlobalPose());
    }
  if (dynamic_cast<Stg::ModelCamera *>(mod)) {
     node->cameramodels.push_back(dynamic_cast<Stg::ModelCamera *>(mod));
  }
}




bool
StageNode::cb_reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Resetting stage!");
  for (size_t r = 0; r < this->positionmodels.size(); r++) {
    this->positionmodels[r]->SetPose(this->initial_poses[r]);
    this->positionmodels[r]->SetStall(false);
  }
  return true;
}



void
StageNode::cmdvelReceived(int idx, const boost::shared_ptr<geometry_msgs::Twist const>& msg)
{
    boost::mutex::scoped_lock lock(msg_lock);
    this->positionmodels[idx]->SetSpeed(msg->linear.x,
                                        msg->linear.y,
                                        msg->angular.z);
    this->base_last_cmd = this->sim_time;
    // if ( (this->sim_time.toSec() > 1.0) && (idx == 0) )
	// ROS_WARN("cmd vel received at curr time %f, for robot id %i, linx %f, liny %f, angz %f ", this->sim_time.toSec(), idx, msg->linear.x, msg->linear.y, msg->angular.z);

}

StageNode::StageNode(int argc, char** argv, bool gui, const char* fname, bool use_model_names)
{
    this->use_model_names = use_model_names;
    this->sim_time.fromSec(0.0);
    this->base_last_cmd.fromSec(0.0);
    double t;
    ros::NodeHandle localn("~");
    if(!localn.getParam("base_watchdog_timeout", t))
        t = 0.2;
    this->base_watchdog_timeout.fromSec(t);

    if(!localn.getParam("is_depth_canonical", isDepthCanonical))
        isDepthCanonical = true;

    // We'll check the existence of the world file, because libstage doesn't
    // expose its failure to open it.  Could go further with checks (e.g., is
    // it readable by this user).
    struct stat s;
    if(stat(fname, &s) != 0)
    {
        ROS_FATAL("The world file %s does not exist.", fname);
        ROS_BREAK();
    }

    // initialize libstage
    Stg::Init( &argc, &argv );

    if(gui)
        this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
    else
        this->world = new Stg::World();

    this->world->Load(fname);

    // todo: reverse the order of these next lines? try it .

    this->world->AddUpdateCallback((Stg::world_callback_t)s_update, this);

    // inspect every model to locate the things we care about
    this->world->ForEachDescendant((Stg::model_callback_t)ghfunc, this);

    std::cerr << "IN StageNode constructor: base_watchdog_timeout " << this->base_watchdog_timeout << " fname: " << fname << std::endl;
    world_update_count = 0;
}


// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int
StageNode::SubscribeModels()
{
    n_.setParam("/use_sim_time", true);

    for (size_t r = 0; r < this->positionmodels.size(); r++)
    {
        StageRobot* new_robot = new StageRobot;
        new_robot->positionmodel = this->positionmodels[r];
        new_robot->positionmodel->Subscribe();

	ROS_INFO( "Subscribed to Stage position model \"%s\"", this->positionmodels[r]->Token() ); 
		      
        for (size_t s = 0; s < this->lasermodels.size(); s++)
        {
	  if (this->lasermodels[s] and this->lasermodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->lasermodels.push_back(this->lasermodels[s]);
                this->lasermodels[s]->Subscribe();
	      ROS_INFO( "subscribed to Stage ranger \"%s\"", this->lasermodels[s]->Token() ); 
            }
        }

        for (size_t s = 0; s < this->cameramodels.size(); s++)
        {
            if (this->cameramodels[s] and this->cameramodels[s]->Parent() == new_robot->positionmodel)
            {
                new_robot->cameramodels.push_back(this->cameramodels[s]);
                this->cameramodels[s]->Subscribe();

		ROS_INFO( "subscribed to Stage camera model \"%s\"", this->cameramodels[s]->Token() ); 
            }
        }

	// TODO - print the topic names nicely as well
        ROS_INFO("Robot %s provided %lu rangers and %lu cameras",
		 new_robot->positionmodel->Token(),
		 new_robot->lasermodels.size(),
		 new_robot->cameramodels.size() );

        new_robot->odom_pub = n_.advertise<nav_msgs::Odometry>(mapName(ODOM, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        new_robot->ground_truth_pub = n_.advertise<nav_msgs::Odometry>(mapName(BASE_POSE_GROUND_TRUTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10);
        // this->robotmodels_[r]->positionmodel->Token()
	new_robot->odom_ts_pub = n_.advertise<std_msgs::Header>("/" + std::string(new_robot->positionmodel->Token())  + "/odom_ts", 1 );

	new_robot->cmdvel_sub = n_.subscribe<geometry_msgs::Twist>(mapName(CMD_VEL, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10, boost::bind(&StageNode::cmdvelReceived, this, r, _1) ); // , ros::TransportHints().tcpNoDelay()

        for (size_t s = 0;  s < new_robot->lasermodels.size(); ++s)
        {
            if (new_robot->lasermodels.size() == 1)
                new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            else
                new_robot->laser_pubs.push_back(n_.advertise<sensor_msgs::LaserScan>(mapName(BASE_SCAN, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));

        }

        for (size_t s = 0;  s < new_robot->cameramodels.size(); ++s)
        {
            if (new_robot->cameramodels.size() == 1)
            {
                new_robot->image_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO, r, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
            else
            {
                new_robot->image_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(IMAGE, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->depth_pubs.push_back(n_.advertise<sensor_msgs::Image>(mapName(DEPTH, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
                new_robot->camera_pubs.push_back(n_.advertise<sensor_msgs::CameraInfo>(mapName(CAMERA_INFO, r, s, static_cast<Stg::Model*>(new_robot->positionmodel)), 10));
            }
        }

        this->robotmodels_.push_back(new_robot);
    }
    clock_pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);

    // advertising reset service
    reset_srv_ = n_.advertiseService("reset_positions", &StageNode::cb_reset_srv, this);

    return(0);
}

StageNode::~StageNode()
{    
    for (std::vector<StageRobot const*>::iterator r = this->robotmodels_.begin(); r != this->robotmodels_.end(); ++r)
        delete *r;
}

bool
StageNode::UpdateWorld()
{
    return this->world->UpdateAll();
}

void
StageNode::WorldCallback()
{
  if( ! ros::ok() ) {
    ROS_INFO( "ros::ok() is false. Quitting." );
    this->world->QuitAll();
    return;
  }
  
    world_update_count += 1;
    boost::mutex::scoped_lock lock(msg_lock);

    this->sim_time.fromSec(world->SimTimeNow() / 1e6);
    // We're not allowed to publish clock==0, because it used as a special
    // value in parts of ROS, #4027.
    if(this->sim_time.sec == 0 && this->sim_time.nsec == 0)
    {
        ROS_DEBUG("Skipping initial simulation step, to avoid publishing clock==0");
        return;
    }

    // getting current RT corr. to Simulated time, to save current positions of robots.
    boost::chrono::time_point<boost::chrono::system_clock> rt_now = boost::chrono::system_clock::now();
    boost::chrono::system_clock::duration rt_tse = rt_now.time_since_epoch();
    unsigned long long rt_ct = boost::chrono::duration_cast<boost::chrono::milliseconds>(rt_tse).count() - (1603000000000);
    double real_time = rt_ct / (double)1000.0;

    struct timespec rt_mono1;
    clock_gettime(CLOCK_MONOTONIC, &rt_mono1);
    double real_mono_time = rt_mono1.tv_sec + 1e-9*rt_mono1.tv_nsec;

    // TODO make this only affect one robot if necessary
    if((this->base_watchdog_timeout.toSec() > 0.0) &&
            ((this->sim_time - this->base_last_cmd) >= this->base_watchdog_timeout))
    {
	// ROS_WARN("STOPPING ALL ROBOS! Curr time : %f, base_last_cmd %f, timeout %f ", this->sim_time, this->base_last_cmd, this->base_watchdog_timeout);
        for (size_t r = 0; r < this->positionmodels.size(); r++)
            this->positionmodels[r]->SetSpeed(0.0, 0.0, 0.0);
    }

    //loop on the robot models
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
        StageRobot const * robotmodel = this->robotmodels_[r];

	//the position of the robot
        tf.sendTransform(tf::StampedTransform(tf::Transform::getIdentity(),
                                              sim_time,
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

        // Get latest odometry data
        // Translate into ROS message format and publish
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x = robotmodel->positionmodel->est_pose.x;
        odom_msg.pose.pose.position.y = robotmodel->positionmodel->est_pose.y;
        odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotmodel->positionmodel->est_pose.a);
        Stg::Velocity v = robotmodel->positionmodel->GetVelocity();
        odom_msg.twist.twist.linear.x = v.x;
        odom_msg.twist.twist.linear.y = v.y;
        odom_msg.twist.twist.angular.z = v.a;

	//@todo Publish stall on a separate topic when one becomes available
        //this->odomMsgs[r].stall = this->positionmodels[r]->Stall();
        //
        odom_msg.header.frame_id = mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        odom_msg.header.stamp = sim_time;

        robotmodel->odom_pub.publish(odom_msg);

	struct timespec rt_mono_o;
	clock_gettime(CLOCK_MONOTONIC, &rt_mono_o);
	double rt_monod_o = rt_mono_o.tv_sec + 1e-9*rt_mono_o.tv_nsec;

	std_msgs::Header odom_tf_ts;
	odom_tf_ts.stamp = ros::Time( rt_monod_o );
	robotmodel->odom_ts_pub.publish(odom_tf_ts);

        // broadcast odometry transform
        tf::Quaternion odomQ;
        tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, odomQ);
        tf::Transform txOdom(odomQ, tf::Point(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, 0.0));
        tf.sendTransform(tf::StampedTransform(txOdom, sim_time,
                                              mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                              mapName("base_footprint", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

        //loop on the laser devices for the current robot
        for (size_t s = 0; s < robotmodel->lasermodels.size(); ++s)
        {
            Stg::ModelRanger const* lasermodel = robotmodel->lasermodels[s];
            const std::vector<Stg::ModelRanger::Sensor>& sensors = lasermodel->GetSensors();

            if( sensors.size() > 1 )
                ROS_WARN( "ROS Stage currently supports rangers with 1 sensor only." );

            // for now we access only the zeroth sensor of the ranger - good
            // enough for most laser models that have a single beam origin
            const Stg::ModelRanger::Sensor& sensor = sensors[0];

	    // Also publish the base->base_laser_link Tx.  This could eventually move
            // into being retrieved from the param server as a static Tx.
            Stg::Pose lp = lasermodel->GetPose();
            tf::Quaternion laserQ;
            laserQ.setRPY(0.0, 0.0, lp.a);
            tf::Transform txLaser =  tf::Transform(laserQ, tf::Point(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z + lp.z));

            if (robotmodel->lasermodels.size() > 1)
                tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
            else
                tf.sendTransform(tf::StampedTransform(txLaser, sim_time,
                                                      mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                      mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

            if( sensor.ranges.size() )
            {
                // Translate into ROS message format and publish
                sensor_msgs::LaserScan msg;
                msg.angle_min = -sensor.fov/2.0;
                msg.angle_max = +sensor.fov/2.0;
                msg.angle_increment = sensor.fov/(double)(sensor.sample_count-1);
                msg.range_min = sensor.range.min;
                msg.range_max = sensor.range.max;
                msg.ranges.resize(sensor.ranges.size());
                msg.intensities.resize(sensor.intensities.size());

                for(unsigned int i = 0; i < sensor.ranges.size(); i++)
                {
                    msg.ranges[i] = sensor.ranges[i];
                    msg.intensities[i] = sensor.intensities[i];
                }

                if (robotmodel->lasermodels.size() > 1)
                    msg.header.frame_id = mapName("base_laser_link", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    msg.header.frame_id = mapName("base_laser_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel));

		boost::chrono::time_point<boost::chrono::system_clock> now = boost::chrono::system_clock::now();
        	boost::chrono::system_clock::duration tse = now.time_since_epoch();
        	std::time_t secs = boost::chrono::system_clock::to_time_t(now);

                msg.header.stamp = sim_time;

		// Setting system clock time as scan TS:
		// Use scan_time field of LaserScan to send real TS. [Changing TS to real tie messes up the timing with TF]
		unsigned long long ct = boost::chrono::duration_cast<boost::chrono::milliseconds>(tse).count() - (1603000000000);
		double rt_boost = ct / (double)(1000.0);

		// Trying clock gettime CLOCK_MONOTONIC
		struct timespec rt_mono;
		clock_gettime(CLOCK_MONOTONIC, &rt_mono);
		double rt_monod = rt_mono.tv_sec + 1e-9*rt_mono.tv_nsec;

		// CLOCK_MONOTONIC_RAW
		struct timespec rt_mono_raw;
		clock_gettime(CLOCK_MONOTONIC, &rt_mono_raw);
		double rt_mono_rawd = rt_mono_raw.tv_sec + 1e-9*rt_mono_raw.tv_nsec;

		// CLOCK_REALTIME 
		struct timespec rt_realtime;
                clock_gettime(CLOCK_REALTIME, &rt_realtime);
		double rt_realtimed = rt_realtime.tv_sec + 1e-9*rt_realtime.tv_nsec;

		rt_realtimed -= 1603000000.00;
		msg.scan_time = rt_monod;

                robotmodel->laser_pubs[s].publish(msg);

		if ( r<1 && (world_update_count%50 == 7) )
			ROS_WARN("WorldUpdateCount %i, BoostSysRT %f, MonoRT %f, MonoRawRT %f, RealRT %f", world_update_count, rt_boost, rt_monod, rt_mono_rawd, rt_realtimed);
                
		if (r < 1 && (world_update_count % 100 == 7)) // 
                {
			// std::cerr << " scanTime: " << msg.scan_time << " " << boost::chrono::duration_cast<boost::chrono::milliseconds>(tse).count() << " ct:" << ct << std::endl;
			ROS_ERROR("Just published scan msg, scantime: %f, boost time: %f", msg.scan_time, rt_boost);
            	}

		if (r < 1 && (world_update_count % 2 == 1) ) // save scans at 25Hz.
			robo0_last_five_scans.push_back(msg);
	    }

        }


        // Also publish the ground truth pose and velocity
        Stg::Pose gpose = robotmodel->positionmodel->GetGlobalPose();
        tf::Quaternion q_gpose;
        q_gpose.setRPY(0.0, 0.0, gpose.a);
        tf::Transform gt(q_gpose, tf::Point(gpose.x, gpose.y, 0.0));
        // Velocity is 0 by default and will be set only if there is previous pose and time delta>0
        Stg::Velocity gvel(0,0,0,0);
        if (this->base_last_globalpos.size()>r){
            Stg::Pose prevpose = this->base_last_globalpos.at(r);
            double dT = (this->sim_time-this->base_last_globalpos_time).toSec();
            if (dT>0)
                gvel = Stg::Velocity(
                            (gpose.x - prevpose.x)/dT,
                            (gpose.y - prevpose.y)/dT,
                            (gpose.z - prevpose.z)/dT,
                            Stg::normalize(gpose.a - prevpose.a)/dT
                            );
            this->base_last_globalpos.at(r) = gpose;
        }else //There are no previous readings, adding current pose...
            this->base_last_globalpos.push_back(gpose);

        nav_msgs::Odometry ground_truth_msg;
        ground_truth_msg.pose.pose.position.x     = gt.getOrigin().x();
        ground_truth_msg.pose.pose.position.y     = gt.getOrigin().y();
        ground_truth_msg.pose.pose.position.z     = gt.getOrigin().z();
        ground_truth_msg.pose.pose.orientation.x  = gt.getRotation().x();
        ground_truth_msg.pose.pose.orientation.y  = gt.getRotation().y();
        ground_truth_msg.pose.pose.orientation.z  = gt.getRotation().z();
        ground_truth_msg.pose.pose.orientation.w  = gt.getRotation().w();
        ground_truth_msg.twist.twist.linear.x = gvel.x;
        ground_truth_msg.twist.twist.linear.y = gvel.y;
        ground_truth_msg.twist.twist.linear.z = gvel.z;
        ground_truth_msg.twist.twist.angular.z = gvel.a;

        ground_truth_msg.header.frame_id = mapName("odom", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
        ground_truth_msg.header.stamp = sim_time;

        robotmodel->ground_truth_pub.publish(ground_truth_msg);

	/*
	if ( std::string( this->robotmodels_[r]->positionmodel->Token() ).find("0") != std::string::npos && (odom_gt_log.is_open()) )
	{
		float odom_actual_x = odom_msg.pose.pose.position.x/sqrt(2) - odom_msg.pose.pose.position.y/sqrt(2) + est_origin_x;
                float odom_actual_y = odom_msg.pose.pose.position.x/sqrt(2) + odom_msg.pose.pose.position.y/sqrt(2) + est_origin_y;
		tf::Transform txGT(gt.getRotation(), tf::Point(0.0, 0.0, 0.0));

		tf::Quaternion odomQ1;
                tf::quaternionMsgToTF(odom_msg.pose.pose.orientation, odomQ1);
                tf::Transform txOdom1(odomQ1, tf::Point(0.0, 0.0, 0.0));
		
		tf::Transform pose_diff = txGT.inverseTimes(txOdom1);
	        tf::Quaternion diffQ = pose_diff.getRotation();
		
		odom_gt_log << (odom_actual_x - gt.getOrigin().x()) << ", ";
                odom_gt_log << (odom_actual_y - gt.getOrigin().y()) << ", ";
		odom_gt_log << float(diffQ.getAngle()) << " \n";
	}
	odom vs GT published by stage. */

        // for nav2d scheduling expts:
        all_robos_pose_x.push_back(gt.getOrigin().x());
        all_robos_pose_y.push_back(gt.getOrigin().y());
	all_robos_orient_z.push_back( gt.getRotation().z() );
	all_robos_orient_w.push_back( gt.getRotation().w() );

		all_robos_twist_lin_z.push_back(gvel.z);
		all_robos_twist_lin_y.push_back(gvel.y);
		all_robos_twist_lin_x.push_back(gvel.x);
		all_robos_twist_ang_z.push_back(gvel.a);

	// update rob0_stalled only if 0 in robotName.	
	if ( std::string( this->robotmodels_[r]->positionmodel->Token() ).find("0") != std::string::npos )
		rob0_stalled = this->positionmodels[r]->Stalled();        
	else if (this->positionmodels[r]->Stalled())
		stalled_robos.push_back( std::string( this->robotmodels_[r]->positionmodel->Token() ) );
	

	if (robo0_last_five_scans.size() > 5)
	{
		std::string ename;
                ros::NodeHandle nh;
		nh.param<std::string>("/expt_name", ename, "");
		struct stat buffer;
                std::string fname = "/home/ubuntu/nav2d_stage_scans_"+ename+".bag";
		if (stat (fname.c_str(), &buffer) == 0)
			robo0_scans_bag.open(fname, rosbag::bagmode::Append);
		else
			robo0_scans_bag.open(fname, rosbag::bagmode::Write);
		for (int ssi = 0; ssi < robo0_last_five_scans.size(); ssi++)
			robo0_scans_bag.write("/robot_0/scans", robo0_last_five_scans[ssi].header.stamp, robo0_last_five_scans[ssi]);
		robo0_last_five_scans.clear();
		robo0_scans_bag.close();
	}

	//cameras
        for (size_t s = 0; s < robotmodel->cameramodels.size(); ++s)
        {
            Stg::ModelCamera* cameramodel = robotmodel->cameramodels[s];
            // Get latest image data
            // Translate into ROS message format and publish
            if (robotmodel->image_pubs[s].getNumSubscribers() > 0 && cameramodel->FrameColor())
            {
                sensor_msgs::Image image_msg;

                image_msg.height = cameramodel->getHeight();
                image_msg.width = cameramodel->getWidth();
                image_msg.encoding = "rgba8";
                //this->imageMsgs[r].is_bigendian="";
                image_msg.step = image_msg.width*4;
                image_msg.data.resize(image_msg.width * image_msg.height * 4);

                memcpy(&(image_msg.data[0]), cameramodel->FrameColor(), image_msg.width * image_msg.height * 4);

                //invert the opengl weirdness
                int height = image_msg.height - 1;
                int linewidth = image_msg.width*4;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&image_msg.data[y*linewidth],linewidth);
                    memcpy(&(image_msg.data[y*linewidth]),&(image_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(image_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    image_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    image_msg.header.frame_id = mapName("camera", r,static_cast<Stg::Model*>(robotmodel->positionmodel));
                image_msg.header.stamp = sim_time;

                robotmodel->image_pubs[s].publish(image_msg);
            }

            //Get latest depth data
            //Translate into ROS message format and publish
            //Skip if there are no subscribers
            if (robotmodel->depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth())
            {
                sensor_msgs::Image depth_msg;
                depth_msg.height = cameramodel->getHeight();
                depth_msg.width = cameramodel->getWidth();
                depth_msg.encoding = this->isDepthCanonical?sensor_msgs::image_encodings::TYPE_32FC1:sensor_msgs::image_encodings::TYPE_16UC1;
                //this->depthMsgs[r].is_bigendian="";
                int sz = this->isDepthCanonical?sizeof(float):sizeof(uint16_t);
                size_t len = depth_msg.width * depth_msg.height;
                depth_msg.step = depth_msg.width * sz;
                depth_msg.data.resize(len*sz);

                //processing data according to REP118
                if (this->isDepthCanonical){
                    double nearClip = cameramodel->getCamera().nearClip();
                    double farClip = cameramodel->getCamera().farClip();
                    memcpy(&(depth_msg.data[0]),cameramodel->FrameDepth(),len*sz);
                    float * data = (float*)&(depth_msg.data[0]);
                    for (size_t i=0;i<len;++i)
                        if(data[i]<=nearClip)
                            data[i] = -INFINITY;
                        else if(data[i]>=farClip)
                            data[i] = INFINITY;
                }
                else{
                    int nearClip = (int)(cameramodel->getCamera().nearClip() * 1000);
                    int farClip = (int)(cameramodel->getCamera().farClip() * 1000);
                    for (size_t i=0;i<len;++i){
                        int v = (int)(cameramodel->FrameDepth()[i]*1000);
                        if (v<=nearClip || v>=farClip) v = 0;
                        ((uint16_t*)&(depth_msg.data[0]))[i] = (uint16_t) ((v<=nearClip || v>=farClip) ? 0 : v );
                    }
                }

                //invert the opengl weirdness
                int height = depth_msg.height - 1;
                int linewidth = depth_msg.width*sz;

                char* temp = new char[linewidth];
                for (int y = 0; y < (height+1)/2; y++)
                {
                    memcpy(temp,&depth_msg.data[y*linewidth],linewidth);
                    memcpy(&(depth_msg.data[y*linewidth]),&(depth_msg.data[(height-y)*linewidth]),linewidth);
                    memcpy(&(depth_msg.data[(height-y)*linewidth]),temp,linewidth);
                }

                if (robotmodel->cameramodels.size() > 1)
                    depth_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    depth_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                depth_msg.header.stamp = sim_time;
                robotmodel->depth_pubs[s].publish(depth_msg);
            }

            //sending camera's tf and info only if image or depth topics are subscribed to
            if ((robotmodel->image_pubs[s].getNumSubscribers()>0 && cameramodel->FrameColor())
                    || (robotmodel->depth_pubs[s].getNumSubscribers()>0 && cameramodel->FrameDepth()))
            {

                Stg::Pose lp = cameramodel->GetPose();
                tf::Quaternion Q; Q.setRPY(
                            (cameramodel->getCamera().pitch()*M_PI/180.0)-M_PI,
                            0.0,
                            lp.a+(cameramodel->getCamera().yaw()*M_PI/180.0) - robotmodel->positionmodel->GetPose().a
                            );

                tf::Transform tr =  tf::Transform(Q, tf::Point(lp.x, lp.y, robotmodel->positionmodel->GetGeom().size.z+lp.z));

                if (robotmodel->cameramodels.size() > 1)
                    tf.sendTransform(tf::StampedTransform(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel))));
                else
                    tf.sendTransform(tf::StampedTransform(tr, sim_time,
                                                          mapName("base_link", r, static_cast<Stg::Model*>(robotmodel->positionmodel)),
                                                          mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel))));

                sensor_msgs::CameraInfo camera_msg;
                if (robotmodel->cameramodels.size() > 1)
                    camera_msg.header.frame_id = mapName("camera", r, s, static_cast<Stg::Model*>(robotmodel->positionmodel));
                else
                    camera_msg.header.frame_id = mapName("camera", r, static_cast<Stg::Model*>(robotmodel->positionmodel));
                camera_msg.header.stamp = sim_time;
                camera_msg.height = cameramodel->getHeight();
                camera_msg.width = cameramodel->getWidth();

                double fx,fy,cx,cy;
                cx = camera_msg.width / 2.0;
                cy = camera_msg.height / 2.0;
                double fovh = cameramodel->getCamera().horizFov()*M_PI/180.0;
                double fovv = cameramodel->getCamera().vertFov()*M_PI/180.0;
                //double fx_ = 1.43266615300557*this->cameramodels[r]->getWidth()/tan(fovh);
                //double fy_ = 1.43266615300557*this->cameramodels[r]->getHeight()/tan(fovv);
                fx = cameramodel->getWidth()/(2*tan(fovh/2));
                fy = cameramodel->getHeight()/(2*tan(fovv/2));

                //ROS_INFO("fx=%.4f,%.4f; fy=%.4f,%.4f", fx, fx_, fy, fy_);


                camera_msg.D.resize(4, 0.0);

                camera_msg.K[0] = fx;
                camera_msg.K[2] = cx;
                camera_msg.K[4] = fy;
                camera_msg.K[5] = cy;
                camera_msg.K[8] = 1.0;

                camera_msg.R[0] = 1.0;
                camera_msg.R[4] = 1.0;
                camera_msg.R[8] = 1.0;

                camera_msg.P[0] = fx;
                camera_msg.P[2] = cx;
                camera_msg.P[5] = fy;
                camera_msg.P[6] = cy;
                camera_msg.P[10] = 1.0;

                robotmodel->camera_pubs[s].publish(camera_msg);

            }

        }
    }

    // For nav2d scheduling expts:
    ros::NodeHandle nh_dummy;
    nh_dummy.param<std::string>("/expt_name", expt_name, "");
    // std::cerr << "SET expt_name to " << expt_name << std::endl;

    std::ofstream of;
    of.open( ("/home/ubuntu/robot_nav2d_obstacleDist_logs_" + expt_name + ".txt").c_str() , std::ios_base::app);
    of << "ST: " << this->sim_time.toSec() << " RT: " << (real_mono_time) << " \n";
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
        of << this->robotmodels_[r]->positionmodel->Token() << " " << all_robos_pose_x[r] << " " << all_robos_pose_y[r] << "\n";
    }
    for (size_t r = 0; r < this->robotmodels_.size(); ++r)
    {
	if ( std::string( this->robotmodels_[r]->positionmodel->Token() ).find("0") != std::string::npos )
		of << this->robotmodels_[r]->positionmodel->Token() << " " << all_robos_twist_lin_x[r] << " " << all_robos_twist_lin_y[r] << " " << all_robos_twist_lin_z[r] << " " << all_robos_twist_ang_z[r] << " orient: " << all_robos_orient_z[r] << " " << all_robos_orient_w[r] << " stall: " << rob0_stalled << " ";
	
	

    }
    if (stalled_robos.size() > 0)
	{
		of << "St_O: ";
		for (int i = 0; i < stalled_robos.size(); i++)
			of << stalled_robos[i] << " ";
	}
    of << "\n";
    
    all_robos_pose_x.clear();
    all_robos_pose_y.clear();
    all_robos_orient_z.clear();
    all_robos_orient_w.clear();
    all_robos_twist_lin_x.clear();
    all_robos_twist_lin_y.clear();
    all_robos_twist_lin_z.clear();
    all_robos_twist_ang_z.clear();
    stalled_robos.clear();
    rob0_stalled = false;

    this->base_last_globalpos_time = this->sim_time;
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = sim_time;
    this->clock_pub_.publish(clock_msg);
}

int 
main(int argc, char** argv)
{
    if( argc < 2 )
    {
        puts(USAGE);
        exit(-1);
    }

    ros::init(argc, argv, "stageros");

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
   	ros::console::notifyLoggerLevelsChanged();
    }

    bool gui = true;
    bool use_model_names = false;
    for(int i=0;i<(argc-1);i++)
    {
        if(!strcmp(argv[i], "-g"))
            gui = false;
        if(!strcmp(argv[i], "-u"))
            use_model_names = true;
    }

    StageNode sn(argc-1,argv,gui,argv[argc-1], use_model_names);

    if(sn.SubscribeModels() != 0)
        exit(-1);

    boost::thread t = boost::thread(boost::bind(&ros::spin));

    sn.world->Start();

    Stg::World::Run();
    
    t.join();

    exit(0);
}

