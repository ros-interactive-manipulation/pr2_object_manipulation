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
// @brief: pr2_gripper_event_detector_action.cpp - action server for the pr2_gripper_event_detector
//         action command

#include "ros/ros.h"

#include <actionlib/server/action_server.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorData.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand.h>


class Pr2GripperEventDetector
{
private:
  typedef actionlib::ActionServer<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> GAS;
  typedef GAS::GoalHandle GoalHandle;
public:
  Pr2GripperEventDetector(ros::NodeHandle &n) :
    node_(n),
    action_server_(node_, "event_detector",
                   boost::bind(&Pr2GripperEventDetector::goalCB, this, _1),
                   boost::bind(&Pr2GripperEventDetector::cancelCB, this, _1)),
    has_active_goal_(false)
  {
    ros::NodeHandle pn("~");

    // we will listen for messages on "state" and send messages on "find_contact"
    pub_controller_command_ =
      node_.advertise<pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand>("event_detector", 1);
    sub_controller_state_ =
      node_.subscribe("event_detector_state", 1, &Pr2GripperEventDetector::controllerStateCB, this);

    watchdog_timer_ = node_.createTimer(ros::Duration(1.0), &Pr2GripperEventDetector::watchdog, this);

  }

  ~Pr2GripperEventDetector()
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
  ros::Time action_start_time;

  bool has_active_goal_;
  GoalHandle active_goal_;
  ros::Time goal_received_;

  double stall_timeout_;
  ros::Time last_movement_time_;


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
    goal_received_ = ros::Time::now();

    // Sends command to trigger controller
    pub_controller_command_.publish(gh.getGoal()->command);
    
    last_movement_time_ = ros::Time::now();
    action_start_time = ros::Time::now();
  }

  void cancelCB(GoalHandle gh)
  {
    
    if (active_goal_ == gh)
    {
      // Stops the controller
      if (last_controller_state_)
      {

      }

      // Marks the current goal as canceled.
      active_goal_.setCanceled();
      has_active_goal_ = false;
    }
  }


  pr2_gripper_sensor_msgs::PR2GripperEventDetectorDataConstPtr last_controller_state_;
  void controllerStateCB(const pr2_gripper_sensor_msgs::PR2GripperEventDetectorDataConstPtr &msg)
  {
    last_controller_state_ = msg;
    ros::Time now = ros::Time::now();

    if (!has_active_goal_)
      return;

    pr2_gripper_sensor_msgs::PR2GripperEventDetectorFeedback feedback;
    feedback.data = *msg;

    pr2_gripper_sensor_msgs::PR2GripperEventDetectorResult result;
    result.data = *msg;

    // check if our success-condition is met, but only listen to feedback messages
    // that were published some dT after the action was started
    double dT = 0.1;
    if(feedback.data.trigger_conditions_met && (feedback.data.stamp.toSec() > action_start_time.toSec()+dT ))
    {
      ROS_INFO("Event Detector Triggered: slip %d, acceleration %d",feedback.data.slip_event,feedback.data.acceleration_event);
      active_goal_.setSucceeded(result);
      has_active_goal_ = false;
    }

    active_goal_.publishFeedback(feedback);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_contact_node");
  ros::NodeHandle node;
  Pr2GripperEventDetector jte(node);

  ros::spin();

  return 0;
}
