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

#ifndef ROSCPP_SUBSCRIPTION_QUEUE_H
#define ROSCPP_SUBSCRIPTION_QUEUE_H

#include "forwards.h"
#include "common.h"
#include "ros/message_event.h"
#include "publisher.h"
// #include "node_handle.h"
#include "callback_queue_interface.h"

#include "std_msgs/Header.h"

#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <deque>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <fstream>

namespace ros
{

class MessageDeserializer;
typedef boost::shared_ptr<MessageDeserializer> MessageDeserializerPtr;

class SubscriptionCallbackHelper;
typedef boost::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

class ROSCPP_DECL SubscriptionQueue : public CallbackInterface, public boost::enable_shared_from_this<SubscriptionQueue>
{
private:
  struct Item
  {
    SubscriptionCallbackHelperPtr helper;
    MessageDeserializerPtr deserializer;

    bool has_tracked_object;
    VoidConstWPtr tracked_object;

    bool nonconst_need_copy;
    ros::Time receipt_time;
  };
  typedef std::deque<Item> D_Item;

public:
  SubscriptionQueue(const std::string& topic, int32_t queue_size, bool allow_concurrent_callbacks, ros::Publisher cb_T_p = ros::Publisher());
  ~SubscriptionQueue();

  void push(const SubscriptionCallbackHelperPtr& helper, const MessageDeserializerPtr& deserializer, 
	    bool has_tracked_object, const VoidConstWPtr& tracked_object, bool nonconst_need_copy, 
	    ros::Time receipt_time = ros::Time(), bool* was_full = 0);
  void clear();

  virtual CallbackInterface::CallResult call();
  virtual bool ready();
  bool full();

  void changeBinSize(int binsz);

  // store stats since t=0
  double cb_eval_start_time;
  double cb_eval_durn;
  bool cb_eval_init;
  int cb_eval_num_stages;
  std::vector<bool> cb_eval_restart; // used if multiple stages in offline eval. We clear up the arr, sum after every stage : PROBABLY Don't need this now cuz the change bin sz function can do this...
  std::vector<double> med_cb_arr;
  std::vector<double> tail_cb_arr;
  int current_bin_size; // denotes bin sz or stage

  double sum_cb_time;
  std::vector<double> arr_cb_time;
  // double mean_cb, med_cb, tail_cb;
  long int total_count;
  long int max_count;

  // store latest stats : last ? calls.
  double sum_cb_L;
  std::vector<double> arr_cb_L;
  double mean_cb_L, med_cb_L, tail_cb_L;
  long int max_count_L;

  double last_pub_time;

  ros::Publisher cb_time_pub; // publishing the current stats of cb time.
  bool publish_cb_time;
  int num_publish;
  int max_pub_freq;
  double cb_threshold;
  int percentile;

  int full_total; // will be used to drop alt msgs in gcmp node. Will overload the current_bin_sz param : 2 will denote gcmp

private:
  bool fullNoLock();
  std::string topic_;
  int32_t size_;
  bool full_;

  boost::mutex bin_sz_mutex;
  boost::mutex queue_mutex_;
  D_Item queue_;
  uint32_t queue_size_;
  bool allow_concurrent_callbacks_;

  boost::recursive_mutex callback_mutex_;
};

}

#endif // ROSCPP_SUBSCRIPTION_QUEUE_H
