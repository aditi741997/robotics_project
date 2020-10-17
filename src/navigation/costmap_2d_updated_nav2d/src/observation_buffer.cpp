/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#include <costmap_2d/observation_buffer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <string>
#include <algorithm>
#include <iterator>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace tf2;

namespace costmap_2d
{
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, tf2_ros::Buffer& tf2_buffer, string global_frame,
                                     string sensor_frame, double tf_tolerance) :
    tf2_buffer_(tf2_buffer), observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    last_updated_(ros::Time::now()), global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance)
{
}

std::string convert_point_to_str(const geometry_msgs::PointStamped& pt)
{
  std::string a = "";
  a += to_string(pt.point.x) + " ";
  a += to_string(pt.point.y) + " ";
  a += to_string(pt.point.z) + " ";
  return a;
}

ObservationBuffer::~ObservationBuffer()
{
}

bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
  ROS_WARN("Inside ObservationBuffer::setGlobalFrame. WHO called this?? new_global_frame : %s", new_global_frame.c_str());
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  geometry_msgs::TransformStamped transformStamped;
  if (!tf2_buffer_.canTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }

  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    try
    {
      Observation& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      tf2_buffer_.transform(origin, origin, new_global_frame);
      obs.origin_ = origin.point;

      // we also need to transform the cloud of the observation to the new global frame
      tf2_buffer_.transform(*(obs.cloud_), *(obs.cloud_), new_global_frame);
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame_.c_str(),
                new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  geometry_msgs::PointStamped global_origin;
  // ROS_WARN("In ObservationBuffer::bufferCloud!!!");
  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;
  if (origin_frame.substr(0,1).compare("/") == 0)
  {
    origin_frame = origin_frame.substr(1); // removing 1st char
    // ROS_WARN("In obsBuffer, origin_frame BEGINS with /, removed it. %s", origin_frame.c_str());
  }
  // ROS_WARN("IN ObservationBuffer:: bufferCloud, here is the cloud frame : %s", origin_frame,c_str());
  try
  {
    // ROS_WARN("INside costmap_2d::obstacle_layer::ObservationBuffer bufferCloud(). Will transform bw local frame %s to global_frame_ %s. Cloud's frame : %s", origin_frame.c_str(), global_frame_.c_str(), cloud.header.frame_id.c_str());
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    geometry_msgs::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    tf2_buffer_.transform(local_origin, global_origin, global_frame_);

    // ROS_WARN("In ObservationBuffer::bufferCloud, got the transform from scan to global frame %s", global_frame_,c_str());

    // convert global_origin into string : Is the transform always the same during Map/Explore?
    std::string transform = convert_point_to_str(global_origin);
    if (last_odom_bll_tf.compare("") == 0)
    {
      // ROS_WARN("In ObservationBuffer::bufferCloud. First odom - blaserlink TF : %s", transform.c_str());
    }
    else if (last_odom_bll_tf.compare(transform) != 0)
    {
      // ROS_WARN("In ObservationBuffer::bufferCloud. Current odom - blaserlink TF : %s, DIfferent from old TF : %s", transform.c_str(), last_odom_bll_tf.c_str());
    }
    last_odom_bll_tf = transform;

    tf2::convert(global_origin.point, observation_list_.front().origin_);

    // make sure to pass on the raytrace/obstacle range of the observation buffer to the observations
    observation_list_.front().raytrace_range_ = raytrace_range_;
    observation_list_.front().obstacle_range_ = obstacle_range_;

    sensor_msgs::PointCloud2 global_frame_cloud;

    // TODO: to run with multiple robots, need to copy cloud into a sensor msg
    // with no / in the frame_id.
    sensor_msgs::PointCloud2 cloud_fid;
    cloud_fid.height = cloud.height;
    cloud_fid.width = cloud.width;
    cloud_fid.is_bigendian = cloud.is_bigendian;
    cloud_fid.point_step = cloud.point_step;
    cloud_fid.row_step = cloud.row_step;
    cloud_fid.is_dense = cloud.is_dense;
    // sensor_msgs::PointField cloud_fid_fields [ sizeof(cloud.fields)/sizeof(cloud.fields[0]) ];
    // std::copy(std::begin(cloud.fields), std::end(cloud.fields), std::begin(cloud_fid.fields));
    // cloud_fid.fields = cloud_fid_fields;
    // ROS_WARN("CLoud fid COPIED FIELDs");
    // std::copy(std::begin(cloud.data), std::end(cloud.data), std::begin(cloud_fid.data));
    cloud_fid.fields = cloud.fields;
    cloud_fid.data = cloud.data;
    cloud_fid.header.frame_id = (cloud.header.frame_id.substr(0,1).compare("/") == 0) ? (cloud.header.frame_id.substr(1)) : cloud.header.frame_id;
    cloud_fid.header.stamp = cloud.header.stamp;
    cloud_fid.header.seq = cloud.header.seq;

    // ROS_WARN("IN OBsBuffer:: bufferCloud, Copied cloud data, cloud %i, cloud_fid %i", cloud.data[2], cloud_fid.data[2]);

    // transform the point cloud
    tf2_buffer_.transform(cloud_fid, global_frame_cloud, global_frame_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // now we need to remove observations from the cloud that are below or above our height thresholds
    sensor_msgs::PointCloud2& observation_cloud = *(observation_list_.front().cloud_);
    observation_cloud.height = global_frame_cloud.height;
    observation_cloud.width = global_frame_cloud.width;
    observation_cloud.fields = global_frame_cloud.fields;
    observation_cloud.is_bigendian = global_frame_cloud.is_bigendian;
    observation_cloud.point_step = global_frame_cloud.point_step;
    observation_cloud.row_step = global_frame_cloud.row_step;
    observation_cloud.is_dense = global_frame_cloud.is_dense;

    unsigned int cloud_size = global_frame_cloud.height*global_frame_cloud.width;
    sensor_msgs::PointCloud2Modifier modifier(observation_cloud);
    modifier.resize(cloud_size);
    unsigned int point_count = 0;

    // copy over the points that are within our height bounds
    sensor_msgs::PointCloud2Iterator<float> iter_z(global_frame_cloud, "z");
    std::vector<unsigned char>::const_iterator iter_global = global_frame_cloud.data.begin(), iter_global_end = global_frame_cloud.data.end();
    std::vector<unsigned char>::iterator iter_obs = observation_cloud.data.begin();
    for (; iter_global != iter_global_end; ++iter_z, iter_global += global_frame_cloud.point_step)
    {
      if ((*iter_z) <= max_obstacle_height_
          && (*iter_z) >= min_obstacle_height_)
      {
        std::copy(iter_global, iter_global + global_frame_cloud.point_step, iter_obs);
        iter_obs += global_frame_cloud.point_step;
        ++point_count;
      }
    }

    // resize the cloud for the number of legal points
    modifier.resize(point_count);
    observation_cloud.header.stamp = cloud.header.stamp;
    observation_cloud.header.frame_id = global_frame_cloud.header.frame_id;
  }
  catch (TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    observations.push_back(*obs_it);
  }
}

void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == ros::Duration(0.0))
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      Observation& obs = *obs_it;
      // check if the observation is out of date... and if it is, remove it and those that follow from the list
      if ((last_updated_ - obs.cloud_->header.stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    ROS_WARN(
        "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
}

ros::Time ObservationBuffer::getLatestObsTimeStamp()
{
   // since we would take max, we return 1hr old TimeStamp if no obs.
   if (observation_list_.size() == 0)
       return ros::Time(0.001);
   else
   {
       Observation& obs = *(observation_list_.begin());
       return obs.cloud_->header.stamp;
       // return pcl_conversions::fromPCL(obs.cloud_->header).stamp;
   }
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = ros::Time::now();
}
}  // namespace costmap_2d

