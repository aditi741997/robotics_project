/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "ros/subscription_queue.h"
#include "ros/message_deserializer.h"
#include "ros/subscription_callback_helper.h"
#include <bits/stdc++.h>
#include <sys/time.h>

namespace ros
{

SubscriptionQueue::SubscriptionQueue(const std::string& topic, int32_t queue_size, bool allow_concurrent_callbacks, ros::Publisher cb_T_p)
: topic_(topic)
, size_(queue_size)
, full_(false)
, queue_size_(0)
, allow_concurrent_callbacks_(allow_concurrent_callbacks)
{
  ROS_INFO("In SubscriptionQ constructor %s", topic_.c_str());
  // long term stats : Not needed in v1
  total_count = 0;
  max_count = 5000;

  sum_cb_time = 0.0;
  mean_cb = 0.0;
  med_cb = 0.0;
  tail_cb = 0.0;

  // current stats :
  max_count_L = 50; // current cb time window size

  sum_cb_L = 0.0;
  arr_cb_L = std::vector<double> (max_count_L, 0.0); // we will write new time at total_count%max_count_L
  mean_cb_L = 0.0;
  med_cb_L = 0.0;
  tail_cb_L = 0.0;

  cb_time_pub = cb_T_p;
  std::string cb_time_topic = cb_time_pub.getTopic();
  publish_cb_time = !(cb_time_topic.empty());
  last_pub_time = 0.0;
  num_publish = 0;

  // Params of the algorithm
  max_pub_freq = 20; // max freq at which this node can publish its cb time stats
  cb_threshold = 0.05; // min. %age difference in stats to be published.
  
  ROS_INFO("Made subscription queue! Publisher topic %s, publish? %i", cb_time_topic.c_str(), publish_cb_time);
  
  percentile = 95;

  // v1 : using offline-computed estimate of mean,med,tail cb for now.
  if (publish_cb_time)
  {
    cb_eval_durn = 60.0;
    cb_eval_init = false;
    // read current stats from file...
/*    std::ifstream cb_stats_file("/home/ubuntu/catkin_ws/obj_track_cb_stats_file.txt");
    std::string line;
    while (std::getline(cb_stats_file, line))
    {
      // for each line
      if (line.find(cb_time_topic) != std::string::npos)
      {
        // split the line string and get the mean median tail stats.
        std::stringstream ss;
        ss << line;
        ss >> mean_cb;
        ss >> med_cb;
        ss >> tail_cb;
      }
    }

    ROS_INFO("Found cb time stats : mean med tail : %f %f %f, cb_eval_start_time is %f", mean_cb, med_cb, tail_cb, cb_eval_start_time);
  */
  }
  
  full_total_msgs_recv = 0;
  drop_fraction = 1;
}

SubscriptionQueue::~SubscriptionQueue()
{

}

// Oct: Drop fractions should be handled at the subscription object, handleMessage.
void SubscriptionQueue::changeDropFraction(int dropf)
{
	drop_fraction = dropf;
	ROS_WARN("CHANGED DropFraction to %i", dropf);
}

void SubscriptionQueue::push(const SubscriptionCallbackHelperPtr& helper, const MessageDeserializerPtr& deserializer,
                                 bool has_tracked_object, const VoidConstWPtr& tracked_object, bool nonconst_need_copy,
                                 ros::Time receipt_time, bool* was_full)
{
  boost::mutex::scoped_lock lock(queue_mutex_);

  if (was_full)
  {
    *was_full = false;
  }

  if(fullNoLock())
  {
    queue_.pop_front();
    --queue_size_;

    if (!full_)
    {
      ROS_DEBUG("Incoming queue was full for topic \"%s\". Discarded oldest message (current queue size [%d])", topic_.c_str(), (int)queue_.size());
    }

    full_ = true;

    if (was_full)
    {
      *was_full = true;
    }
  }
  else
  {
    full_ = false;
  }

  Item i;
  i.helper = helper;
  i.deserializer = deserializer;
  i.has_tracked_object = has_tracked_object;
  i.tracked_object = tracked_object;
  i.nonconst_need_copy = nonconst_need_copy;
  i.receipt_time = receipt_time;
  queue_.push_back(i);
  ++queue_size_;
}

void SubscriptionQueue::clear()
{
  boost::recursive_mutex::scoped_lock cb_lock(callback_mutex_);
  boost::mutex::scoped_lock queue_lock(queue_mutex_);

  queue_.clear();
  queue_size_ = 0;
}

CallbackInterface::CallResult SubscriptionQueue::call()
{
  // The callback may result in our own destruction.  Therefore, we may need to keep a reference to ourselves
  // that outlasts the scoped_try_lock
  if ((!cb_eval_init) && publish_cb_time)
  {
    cb_eval_init = true;
    cb_eval_start_time = ros::Time::now().toSec();
    ROS_INFO("For topic %s cb_eval_start_time is now %f", cb_time_pub.getTopic().c_str(), cb_eval_start_time);
  }
  ros::Time call_start = ros::Time::now();
  clock_t call_start_real = clock();

  boost::shared_ptr<SubscriptionQueue> self;
  boost::recursive_mutex::scoped_try_lock lock(callback_mutex_, boost::defer_lock);

  if (!allow_concurrent_callbacks_)
  {
    lock.try_lock();
    if (!lock.owns_lock())
    {
      return CallbackInterface::TryAgain;
    }
  }

  VoidConstPtr tracker;
  Item i;

  {
    boost::mutex::scoped_lock lock(queue_mutex_);

    if (queue_.empty())
    {
      return CallbackInterface::Invalid;
    }

    i = queue_.front();

    if (queue_.empty())
    {
      return CallbackInterface::Invalid;
    }

    if (i.has_tracked_object)
    {
      tracker = i.tracked_object.lock();

      if (!tracker)
      {
        return CallbackInterface::Invalid;
      }
    }

    queue_.pop_front();
    --queue_size_;
  }

  VoidConstPtr msg = i.deserializer->deserialize();

  // msg can be null here if deserialization failed
  if (msg)
  {
    try
    {
      self = shared_from_this();
    }
    catch (boost::bad_weak_ptr&) // For the tests, where we don't create a shared_ptr
    {}

    SubscriptionCallbackHelperCallParams params;
    params.event = MessageEvent<void const>(msg, i.deserializer->getConnectionHeader(), i.receipt_time, i.nonconst_need_copy, MessageEvent<void const>::CreateFunction());
    i.helper->call(params);
  }

 // double cb_time = (ros::Time::now() - call_start).toSec();
  double cb_time = (double)(clock() - call_start_real)/CLOCKS_PER_SEC;

  if (publish_cb_time)
  {
    // Update long term stats : not needed in v1
    // sum_cb_time += cb_time;
    // arr_cb_time[total_count%max_count] = cb_time;

    sum_cb_L += cb_time;
    sum_cb_L -= arr_cb_L[total_count%max_count_L];
    arr_cb_L[total_count%max_count_L] = cb_time;
    
    total_count += 1;

    if ((ros::Time::now().toSec() - cb_eval_start_time) < cb_eval_durn)
    {
	sum_cb_time += cb_time;
	arr_cb_time.push_back(cb_time);

	if (total_count%100 == 51)
	{
		std::sort(arr_cb_time.begin(), arr_cb_time.end());
		med_cb = arr_cb_time[arr_cb_time.size()/2];
		tail_cb = arr_cb_time[(percentile*(arr_cb_time.size()))/100];
		ROS_INFO("Publishing! med_cb : %f, tail_cb : %f, total_count : %i, cb_eval_start_time %f, cb_eval_durn %f", med_cb, tail_cb, total_count, cb_eval_start_time, cb_eval_durn);	
		// publish current cb time stats. 
		std_msgs::Header h;
        	h.stamp = ros::Time::now();
        	std::stringstream ss;
        	ss << cb_time_pub.getTopic() << " E ";
        	ss << med_cb << " ";
		ss << tail_cb;
        	h.frame_id = ss.str();
        	// ROS_INFO("About to publish!");
        	cb_time_pub.publish(h);
	}
    }
    else
    {
    if (total_count >= max_count_L)
    {

      // calculate mean, median, tail of current window
      std::vector<double> sorted_cb_times_L (max_count_L, 0.0);
      std::partial_sort_copy(arr_cb_L.begin(), arr_cb_L.end(), sorted_cb_times_L.begin(), sorted_cb_times_L.end());
      med_cb_L = sorted_cb_times_L[max_count_L/2];
      mean_cb_L = sum_cb_L/max_count_L;
      tail_cb_L = sorted_cb_times_L[(percentile*max_count_L)/100];

      if (total_count%500 == 3)
        ROS_INFO("Window sz : %i, total_count %i, num_publish %i, mean med tail : %f %f %f, Current med_cb %f, tail_cb %f", max_count_L, total_count, num_publish, mean_cb_L, med_cb_L, tail_cb_L, med_cb, tail_cb);
      
      // If current stats are quite different than expected 
      // AND time since last pub >= 1.0/max_pub_freq, then publish
      // v1 : using median
      if ( ( ( tail_cb_L > ((1.0+cb_threshold)*tail_cb) ) || ( (tail_cb_L) < ((1.0-cb_threshold)*tail_cb) ) ) && ( (ros::Time::now().toSec() - last_pub_time) >= (1.0/max_pub_freq) ) )
      {
        // publish!
        // v1 : only publish the short term median CBTime
        std_msgs::Header h;
        h.seq = num_publish;
        h.stamp = ros::Time::now();
        std::stringstream ss;
        ss << cb_time_pub.getTopic() << " L ";
        ss << med_cb_L << " ";
	ss << tail_cb_L;
        h.frame_id = ss.str();
        // ROS_INFO("About to publish!");
        cb_time_pub.publish(h);

        // update :
        num_publish += 1;
        last_pub_time = ros::Time::now().toSec();
      }
    } 
 
    }

 }

  return CallbackInterface::Success;
}

bool SubscriptionQueue::ready()
{
  return true;
}

bool SubscriptionQueue::full()
{
  boost::mutex::scoped_lock lock(queue_mutex_);
  return fullNoLock();
}

bool SubscriptionQueue::fullNoLock()
{
  return (size_ > 0) && (queue_size_ >= (uint32_t)size_);
}

}

