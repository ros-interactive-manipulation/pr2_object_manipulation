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

#ifndef GRIPPER_CONTROLLER_H
#define GRIPPER_CONTROLLER_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

#include <sensor_msgs/JointState.h>

#include <object_manipulator/tools/service_action_wrappers.h>

namespace pr2_wrappers {

class GripperController
{
private:

  //! The root namespace node handle
  ros::NodeHandle root_nh_;

  //! The private namespace node handle
  ros::NodeHandle priv_nh_;

  //! The gripper command action
  object_manipulator::MultiArmActionWrapper<pr2_controllers_msgs::Pr2GripperCommandAction> gripper_action_client_;

  //! Gets the virtual gripper joint name for the given arm
  std::string virtualJointName(std::string arm_name);

  //! Given a value of a gripper joint, converts that to a value of the gap between the fingers
  double jointToGap(double jointValue);

  //! Gap value for gripper fully open
  double l_gripper_open_gap_value_;
  double r_gripper_open_gap_value_;

  //! Gap value for gripper fully closed
  double l_gripper_closed_gap_value_;
  double r_gripper_closed_gap_value_;

  //! Max effort used when achieving a position (e.g. pre-grasp or release) without epxecting an object
  double l_gripper_max_effort_;
  double r_gripper_max_effort_;

  //! The type of gripper, for now "pr2" or "lcg"
  std::string l_gripper_type_;
  std::string r_gripper_type_;

public:
  GripperController();

  ~GripperController();

  //------------------------------ Functionality -------------------------------

  //! Returns the current value of the gripper joint
  bool getGripperValue(std::string arm_name, double &value);

  //! Sends a command to the gripper action, does not wait for result
  bool commandGripperValue(std::string arm_name, double value);

  //-------------------------------- Constants ---------------------------------

  //! Gets the gap value for an open gripper
  double getGripperOpenGap(std::string arm_name) const 
  {
    if (arm_name == "right_arm") return r_gripper_open_gap_value_;
    else return l_gripper_open_gap_value_;
  }

  //! Gets the gap value for a closed gripper
  double getGripperClosedGap(std::string arm_name) const 
  {
    if (arm_name == "right_arm") return r_gripper_closed_gap_value_;
    else return l_gripper_closed_gap_value_;
  }

};

}

#endif
