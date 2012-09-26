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
// @brief: pr2_gripper_release_action.cpp - action server for the 
//         pr2_gripper_release action command

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorAction.h>
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperEventDetectorAction> EventClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;



class Gripper{
private:
  GripperClient* gripper_client_;  
  EventClient* event_client_;


public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("gripper_action", true);
    event_client_  = new EventClient("event_detector",true);


    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the gripper_action action server to come up");
    }

    while(!event_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the event_detector action server to come up");
    } 
  }

  ~Gripper(){
    delete gripper_client_;
    delete event_client_;
  }

  //Open the gripper
  void open(double position_open){

    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = position_open;    // position open (9 cm)
    open.command.max_effort = -1.0;           // max force
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    // do not wait, return right away so that the user knows contact was made
  }

  //move into event_detector mode to drop an object
  void place(int trigger_conditions, double acc_trigger, double slip_trigger){
    pr2_gripper_sensor_msgs::PR2GripperEventDetectorGoal place_goal;
    place_goal.command.trigger_conditions = trigger_conditions;  
    place_goal.command.acceleration_trigger_magnitude = acc_trigger;  // set the contact acceleration to n m/s^2
    place_goal.command.slip_trigger_magnitude = slip_trigger;

    ROS_INFO("Waiting for object placement contact...");
    event_client_->sendGoal(place_goal);
    event_client_->waitForResult();
    if(event_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Place Success");
      ROS_INFO("cond met: %d, acc_met: %d, slip_met: %d", event_client_->getResult()->data.trigger_conditions_met, event_client_->getResult()->data.acceleration_event, event_client_->getResult()->data.slip_event);
    }
    else
      ROS_INFO("Place Failure");
  }  


};



class PR2GripperReleaseAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pr2_gripper_sensor_msgs::PR2GripperReleaseAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  pr2_gripper_sensor_msgs::PR2GripperReleaseFeedback feedback_;
  pr2_gripper_sensor_msgs::PR2GripperReleaseResult result_;
  Gripper gripper;

public:

  PR2GripperReleaseAction(std::string name) :
    as_(nh_, name, boost::bind(&PR2GripperReleaseAction::executeCB, this, _1)),
    action_name_(name)
  {
  }

  ~PR2GripperReleaseAction(void)
  {
  }

  void executeCB(const pr2_gripper_sensor_msgs::PR2GripperReleaseGoalConstPtr &goal)
  {
      double position_open; 
      if(!nh_.getParam("position_open", position_open))
        ROS_ERROR("No position_open given in namespace: '%s')", nh_.getNamespace().c_str());

      // helper variables
      bool prempted = false;
    
      gripper.place(goal->command.event.trigger_conditions, goal->command.event.acceleration_trigger_magnitude, goal->command.event.slip_trigger_magnitude);

      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        prempted = true;
      }
      if(!prempted)
        gripper.open(position_open);
      

      result_.data.rtstate.realtime_controller_state = feedback_.data.rtstate.realtime_controller_state;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      if(!prempted)
        as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "release");

  PR2GripperReleaseAction Release("release");
  ros::spin();

  return 0;
}

