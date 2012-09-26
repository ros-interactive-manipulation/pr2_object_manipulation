/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

#ifndef MSG_SAVER_H_
#define MSG_SAVER_H_

#include <boost/thread/mutex.hpp>

/// Helper class. Receives and stores the last instance of a message (thread-safe)
template<class MsgType>
class MsgSaver
{
public:
  typedef boost::shared_ptr< MsgType const> MsgTypeConstPtr;

  MsgSaver( std::string topic )
  {
    boost::mutex::scoped_lock lock( mutex_ );
    sub_ = root_nh_.subscribe<MsgType>(topic, 1, boost::bind(&MsgSaver<MsgType>::callback, this, _1 ) );
    topic_ = root_nh_.resolveName(topic);
  }

  void callback(const MsgTypeConstPtr& msg )
  {
    boost::mutex::scoped_lock lock( mutex_ );
    if (!msg_) ROS_INFO_STREAM( "Message received on " << topic_ );
    msg_ = msg;
  }

  bool hasMsg()
  {
    boost::mutex::scoped_lock lock( mutex_ );
    if (!msg_) ROS_INFO_STREAM( "Waiting for " << topic_ );
    bool hasMsg = msg_;
    return hasMsg;
  }

  MsgTypeConstPtr getMsg()
  {
    boost::mutex::scoped_lock lock( mutex_ );
    MsgTypeConstPtr msg = msg_;
    return msg;
  }

private:

  boost::mutex mutex_;
  MsgTypeConstPtr msg_;
  ros::Subscriber sub_;
  ros::NodeHandle root_nh_;
  std::string topic_;
};

#endif /* MSG_SAVER_H_ */
