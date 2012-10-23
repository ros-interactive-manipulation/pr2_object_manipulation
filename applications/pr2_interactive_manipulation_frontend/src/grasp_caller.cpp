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

/* Simply calls the grasp action of the interactive manipulation backend, again
   and again until the end of time. */

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <pr2_interactive_manipulation/advanced_options_dialog.h>
#include <pr2_object_manipulation_msgs/IMGUIOptions.h>
#include <pr2_object_manipulation_msgs/IMGUIAdvancedOptions.h>
#include <pr2_object_manipulation_msgs/IMGUIAction.h>

using namespace pr2_interactive_manipulation;

class GraspCaller
{
private:
  ros::NodeHandle nh_;
  ros::Timer slow_sync_timer_;
  actionlib::SimpleActionClient<pr2_object_manipulation_msgs::IMGUIAction> *imgui_action_client_;
  int task_number_;

public:
  GraspCaller() : nh_("") 
  {
    imgui_action_client_ = new actionlib::SimpleActionClient<pr2_object_manipulation_msgs::IMGUIAction>("imgui_action", true);
    slow_sync_timer_ =  nh_.createTimer(ros::Duration(0.5), boost::bind( &GraspCaller::slowSync, this ) );
    nh_.param<int>("interactive_grasping/task_number", task_number_, 0);
  }

  void slowSync()
  {

    pr2_object_manipulation_msgs::IMGUIAdvancedOptions adv_options;
    adv_options = AdvancedOptionsDialog::getDefaultsMsg();
    if (task_number_ == 3) adv_options.lift_direction_choice = 1;

    pr2_object_manipulation_msgs::IMGUIGoal imgui_goal;
    imgui_goal.options.collision_checked = true;
    imgui_goal.options.grasp_selection = 0;
    imgui_goal.options.arm_selection = 0;
    imgui_goal.options.adv_options = adv_options;
    imgui_goal.command.command = imgui_goal.command.PICKUP;

    actionlib::SimpleClientGoalState state = imgui_action_client_->getState();
    if (state != actionlib::SimpleClientGoalState::PENDING && 
        state != actionlib::SimpleClientGoalState::ACTIVE)
    {
      imgui_action_client_->sendGoal(imgui_goal);
    }
  }

};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "grasp_caller");
  GraspCaller caller;
  ros::spin();
}
