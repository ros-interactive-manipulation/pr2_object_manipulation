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

#include <pr2_wrappers/tuck_arms_client.h>

namespace pr2_wrappers {

TuckArmsClient::TuckArmsClient(ros::NodeHandle &nh, const ros::Duration &timeout) :
  nh_(nh),
  timeout_(timeout),  
  tuck_arms_action_client_("tuck_arms", true)
{}

TuckArmsClient::~TuckArmsClient(){
}

bool TuckArmsClient::tuckArms( bool tuck_right, bool tuck_left, bool wait)
{
  std::string right_action = (tuck_right)?("tucking"):("untucking");
  std::string left_action =  (tuck_left)?("tucking"):("untucking");
  ROS_INFO_STREAM("Calling tuck arms action: " << right_action << " right arm, " << left_action << " left arm.");
  
  pr2_common_action_msgs::TuckArmsGoal goal;
  goal.tuck_left = tuck_left;
  goal.tuck_right = tuck_right;
  if(wait)
  {
    actionlib::SimpleClientGoalState state = tuck_arms_action_client_.sendGoalAndWait(goal, timeout_, ros::Duration(5.0));
    if(state.state_ != state.SUCCEEDED)
    { 
      if(! state.isDone()) ROS_WARN("Tuck arms action timed out with result %d!", state.state_);
      else ROS_WARN("Tuck arms action returned with result %d!", state.state_);
      return false;
    }
  }
  else
  {
    tuck_arms_action_client_.sendGoal(goal);
  }
  return true;
}

}
