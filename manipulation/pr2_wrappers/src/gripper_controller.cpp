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

#include <pr2_wrappers/gripper_controller.h>

namespace pr2_wrappers {

static const std::string JOINT_STATES_TOPIC = "joint_states";

GripperController::GripperController() :
  root_nh_(""),
  priv_nh_("~"),
  gripper_action_client_("","/gripper_action", true, true)
{
    priv_nh_.param<double>("l_gripper_open_gap_value", l_gripper_open_gap_value_, 0.086);
    priv_nh_.param<double>("l_gripper_closed_gap_value", l_gripper_closed_gap_value_, 0.0);
    priv_nh_.param<double>("l_gripper_max_effort", l_gripper_max_effort_, 50);
    priv_nh_.param<std::string>("l_gripper_type", l_gripper_type_, "pr2");

    priv_nh_.param<double>("r_gripper_open_gap_value", r_gripper_open_gap_value_, 0.086);
    priv_nh_.param<double>("r_gripper_closed_gap_value", r_gripper_closed_gap_value_, 0.0);
    priv_nh_.param<double>("r_gripper_max_effort", r_gripper_max_effort_, 50);
    priv_nh_.param<std::string>("r_gripper_type", r_gripper_type_, "pr2");
}

GripperController::~GripperController()
{
}

std::string GripperController::virtualJointName(std::string arm_name)
{
  if (arm_name=="right_arm") return "r_gripper_joint";
  return "l_gripper_joint";
}

bool GripperController::getGripperValue(std::string arm_name, double &value)
{
  //get the joint states
  sensor_msgs::JointState::ConstPtr joint_states = 
    ros::topic::waitForMessage<sensor_msgs::JointState>(JOINT_STATES_TOPIC, root_nh_, ros::Duration(2.0));
  if (!joint_states)
  {
    ROS_ERROR("pr2 gripper grasp status: joint states not received");
    return false;
  }
  
  //find the gripper joint
  size_t i;
  std::string virtual_joint_name = virtualJointName(arm_name);
  for (i=0; i<joint_states->name.size(); i++)
  {
    if (joint_states->name[i] == virtual_joint_name) break;
  }
  if (i==joint_states->name.size())
  {
    ROS_ERROR("pr2_gripper grasp status: gripper joint %s not found in joint state", virtual_joint_name.c_str());
    return false;
  }
  if (joint_states->position.size() <= i)
  {
    ROS_ERROR("pr2_gripper grasp status: malformed joint state message received");
    return false;
  }
  value = joint_states->position[i];
    return true;
}

bool GripperController::commandGripperValue(std::string arm_name, double value)
{
  pr2_controllers_msgs::Pr2GripperCommandGoal gripper_command;
  gripper_command.command.position = value;
  if (arm_name=="right_arm") gripper_command.command.max_effort = r_gripper_max_effort_;
  else gripper_command.command.max_effort = l_gripper_max_effort_;
  try
  {
    gripper_action_client_.client(arm_name).sendGoal(gripper_command);
  }
  catch(object_manipulator::ServiceNotFoundException &ex)
  {
    ROS_ERROR("Gripper action not available");
    return false;
  }
  return true;
}

}
