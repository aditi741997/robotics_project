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

#include <signal.h>

#include "ros/this_node.h"
#include <unistd.h>
#include <sched.h>
#include <sys/syscall.h>
#define gettid() syscall(SYS_gettid)

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

int PollManager::getThreadId()
{
	return threadFunc_tid;
}

void PollManager::threadFunc()
{
  disableAllSignalsInThisThread();

  threadFunc_tid = ::gettid();
  ROS_ERROR("In PollManager::threadFunc() for node %s, tid %i, pid %i", this_node::getName().c_str(), ::gettid(), ::getpid() );
  
  struct timespec pmt_start, pmt_end;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &pmt_start);
  int pmt_ct = 0;

  /*
  if ( (this_node::getName().find("perator") != std::string::npos) || ( this_node::getName().find("apper") != std::string::npos) )
  if ( this_node::getName().find("avigator") != std::string::npos )
  {
	ROS_INFO("About to change PMThread prio.");
	int ret = 7;
	struct sched_param sp = { .sched_priority = 2,};
	ret = sched_setscheduler(::gettid(), SCHED_FIFO, &sp);
	ROS_INFO("PMThread prio changed retval: %i", ret);
  }
  */

  while (!shutting_down_)
  {
    {
      boost::recursive_mutex::scoped_lock lock(signal_mutex_);
      poll_signal_();
    }

    if (shutting_down_)
    {
      return;
    }

    poll_set_.update(1000);
    pmt_ct += 1;

    if (pmt_ct % 1500 == 311)
    {
	    struct timespec pmt_i;
	    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &pmt_i);
	    std::cerr << "POLLManager THREAD of node " << this_node::getName() << " has used up " << ( (pmt_i.tv_sec + pmt_i.tv_nsec*1e-9) - (pmt_start.tv_sec + pmt_start.tv_nsec*1e-9)) << "s compute since start!! \n";
    }
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
