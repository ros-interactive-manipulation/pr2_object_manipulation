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
 *
 * \author Joe Romano
 */

// @author: Joe Romano
// @email: joeromano@gmail.com 
// @brief: pr2_gripper_slipServo_action.cpp - action server for the 
//         pr2_gripper_slipServo action command

#include "ros/ros.h"

#include <actionlib/server/action_server.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoData.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoCommand.h>
#include <std_srvs/Empty.h>

class Pr2GripperSlipServo
{
private:
  typedef actionlib::ActionServer<pr2_gripper_sensor_msgs::PR2GripperSlipServoAction> GAS;
  typedef GAS::GoalHandle GoalHandle;
public:
  Pr2GripperSlipServo(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "slip_servo",
                   boost::bind(&Pr2GripperSlipServo::goalCB, this, _1),
                   boost::bind(&Pr2GripperSlipServo::cancelCB, this, _1)),
    has_active_goal_(false)
  {
    ros::NodeHandle pn("~");

    // param server stuff?
    //pn.param("goal_threshold", goal_threshold_, 0.01);
    //pn.param("stall_velocity_threshold", stall_velocity_threshold_, 1e-6);
    //pn.param("stall_timeout", stall_timeout_, 0.1);

    // we will listen for messages on "state" and send messages on "find_contact"
    pub_controller_command_ =
      node_.advertise<pr2_gripper_sensor_msgs::PR2GripperSlipServoCommand>("slip_servo", 1);
    sub_controller_state_ =
      node_.subscribe("slip_servo_state", 1, &Pr2GripperSlipServo::controllerStateCB, this);

    watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &Pr2GripperSlipServo::watchdog, this);

  }

  ~Pr2GripperSlipServo()
  {
    pub_controller_command_.shutdown();
    sub_controller_state_.shutdown();
    watchdog_timer_.stop();
  }

private:

  ros::NodeHandle node_;
  GAS action_server_;
  ros::Publisher pub_controller_command_;
  ros::Subscriber sub_controller_state_;
  ros::Timer watchdog_timer_;

  bool has_active_goal_;
  bool firstDrop;
  GoalHandle active_goal_;
  ros::Time goal_received_;

  double goal_threshold_;
  double stall_velocity_threshold_;
  double stall_timeout_;
  ros::Time last_movement_time_;
  ros::Time action_start_time;


  void watchdog(const ros::TimerEvent &e)
  {
    ros::Time now = ros::Time::now();

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      bool should_abort = false;
      if (!last_controller_state_)
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we have never heard a controller state message.");
      }
      else if ((now - last_controller_state_->stamp) > ros::Duration(5.0))
      {
        should_abort = true;
        ROS_WARN("Aborting goal because we haven't heard from the controller in %.3lf seconds",
                 (now - last_controller_state_->stamp).toSec());
      }

      if (should_abort)
      {
        // Marks the current goal as aborted.
        active_goal_.setAborted();
        has_active_goal_ = false;
      }
    }
  }

  void goalCB(GoalHandle gh)
  {
    // Cancels the currently active goal.
    if (has_active_goal_)
    {
      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
 
    gh.setAccepted();
    active_goal_ = gh;
    has_active_goal_ = true;
    firstDrop = true;  // set our first drop flag
    goal_received_ = ros::Time::now();
    
    // Sends the command along to the controller.
    pub_controller_command_.publish(active_goal_.getGoal()->command);
    
    last_movement_time_ = ros::Time::now();
    action_start_time = ros::Time::now();
  }

  void cancelCB(GoalHandle gh)
  {
    
    if (active_goal_ == gh)
    {
      // stop the real-time motor control
      std_srvs::Empty::Request req;
      std_srvs::Empty::Response resp;
      if(ros::service::exists("stop_motor_output",true))
      {
        ROS_INFO("Stopping Motor Output");
        ros::service::call("stop_motor_output",req,resp);
      }

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }


  pr2_gripper_sensor_msgs::PR2GripperSlipServoDataConstPtr last_controller_state_;
  void controllerStateCB(const pr2_gripper_sensor_msgs::PR2GripperSlipServoDataConstPtr &msg)
  {
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
      return;

    pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal goal;
    goal.command = active_goal_.getGoal()->command;

    pr2_gripper_sensor_msgs::PR2GripperSlipServoFeedback feedback;
    feedback.data = *msg;

    pr2_gripper_sensor_msgs::PR2GripperSlipServoResult result;
    result.data = *msg;
    
    // do not check until some dT has expired from message start
    double dT = 0.1;
    if(feedback.data.stamp.toSec() < action_start_time.toSec()+dT ){}

    // if we are actually in a slip_servo control state
    else if(feedback.data.rtstate.realtime_controller_state == 6)
    {
    }
    else
      has_active_goal_ = false;
    
    active_goal_.publishFeedback(feedback);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "slip_servo_node");
  ros::NodeHandle node;
  Pr2GripperSlipServo jte(node);

  ros::spin();

  return 0;
}
