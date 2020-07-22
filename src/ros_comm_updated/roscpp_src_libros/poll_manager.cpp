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

#include "ros/poll_manager.h"
#include "ros/common.h"
#include <boost/lexical_cast.hpp>
#include "ros/this_node.h"

#include <signal.h>

namespace ros
{

const PollManagerPtr& PollManager::instance()
{
  static PollManagerPtr poll_manager = boost::make_shared<PollManager>();
  return poll_manager;
}

PollManager::PollManager()
  : shutting_down_(false)
{
}

PollManager::~PollManager()
{
  shutdown();
}

void PollManager::start()
{
  shutting_down_ = false;
  thread_ = boost::thread(&PollManager::threadFunc, this);
}

void PollManager::shutdown()
{
  if (shutting_down_) return;

  shutting_down_ = true;
  if (thread_.get_id() != boost::this_thread::get_id())
  {
    thread_.join();
  }

  boost::recursive_mutex::scoped_lock lock(signal_mutex_);
  poll_signal_.disconnect_all_slots();
}

void PollManager::threadFunc()
{
  ROS_INFO("PollMgr threadFunc.... BOOST thread id : %s, node name : %s", boost::lexical_cast<std::string>( boost::this_thread::get_id() ).c_str(), this_node::getName().c_str() );
  disableAllSignalsInThisThread();

   struct sched_attr attr;
      int policy = SCHED_DEADLINE;
      attr.sched_policy = SCHED_DEADLINE;
      float ci = 5.5;
      float period = 160;
      if (this_node::getName().find("global") != std::string::npos)
      {
	      period *= 2;
      }
      attr.sched_runtime = ci*1000*1000; // nanosec
      attr.sched_period = period*1000*1000;
      attr.sched_deadline = period*1000*1000;
      int oout = sched_setattr(0, &attr, 0);
      ROS_INFO("Output of sched_setattr for PollMgr thread [processPublishQue] : %i, attr ci %i, ddl %i, period : %i", oout, attr.sched_runtime, attr.sched_deadline, attr.sched_period);


  while (!shutting_down_)
  {
    {
      boost::recursive_mutex::scoped_lock lock(signal_mutex_);
      // calling all the functions connected to the poll_signal_ : i.e. TopicMgr.processPublishQueues, ConnectionManager::removeDroppedConnections, checkForShutdown.
      poll_signal_();
    }

    if (shutting_down_)
    {
      return;
    }

    // poll call timeout in ms
    // update function actually calls poll on available sockets.
    poll_set_.update(period);
  }
}

boost::signals2::connection PollManager::addPollThreadListener(const VoidFunc& func)
{
  boost::recursive_mutex::scoped_lock lock(signal_mutex_);
  return poll_signal_.connect(func);
}

void PollManager::removePollThreadListener(boost::signals2::connection c)
{
  boost::recursive_mutex::scoped_lock lock(signal_mutex_);
  c.disconnect();
}

}
