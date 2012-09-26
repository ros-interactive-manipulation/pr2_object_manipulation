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

#include <pr2_wrappers/torso_client.h>

namespace pr2_wrappers {

TorsoClient::TorsoClient()
{
  torso_client_ = new pr2_wrappers::TorsoActionClient("torso_controller/position_joint_action", true);
  //wait for the action server to come up
  while(!torso_client_->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the torso action server to come up");
  }
}

TorsoClient::~TorsoClient()
{
  delete torso_client_;
}

//! tell the torso to go to a specific position
void TorsoClient::moveTo(const float &p){

  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = p;
  goal.min_duration = ros::Duration(0.1);
  goal.max_velocity = 1.0;
  ROS_INFO("Sending torso goal %f", p);
  torso_client_->sendGoal(goal);
}

//! Tell the torso to go to the top
void TorsoClient::top(){
  moveTo(0.4); // all the way up is 0.2 ... should maybe read from URDF
}

//! Tell the troso to go to the bottom
void TorsoClient::bottom(){
  moveTo(0.0);
}

}

