/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

// author: Adam Leeper

#ifndef _TUCK_ARMS_CLIENT_H_
#define _TUCK_ARMS_CLIENT_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <actionlib/client/simple_action_client.h>

#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_common_action_msgs/TuckArmsGoal.h>

namespace pr2_wrappers {

class TuckArmsClient{
private:

  //! The node handle we'll be using
  ros::NodeHandle nh_;

  //! The timeout duration for the tuck arms action
  ros::Duration timeout_;
  
  //! A actionlib client to the move_base action server
  actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> tuck_arms_action_client_;


public:
  //! Initialization
  TuckArmsClient(ros::NodeHandle &nh, const ros::Duration &timeout);

  ~TuckArmsClient();

  //! Calls the tuck arms action. Optional "wait" argument will block until result.
  bool tuckArms( bool tuck_right, bool tuck_left, bool wait = false);

};

}


#endif
