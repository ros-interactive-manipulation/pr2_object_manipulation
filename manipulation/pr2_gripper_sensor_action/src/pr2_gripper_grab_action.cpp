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
// @brief: pr2_gripper_grab_action.cpp - action server for the 
//         pr2_gripper_grab action command

#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperForceServoAction.h>
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperSlipServoAction> SlipClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperFindContactAction> ContactClient;
// Our Action interface type, provided as a typedef for convenience                         
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
// Our Action interface type, provided as a typedef for convenience             
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperForceServoAction> ForceClient;


class Gripper{
private:
  GripperClient* gripper_client_;  
  ContactClient* contact_client_;
  SlipClient* slip_client_;
  ForceClient* force_client_;


public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("gripper_action", true);
    contact_client_  = new ContactClient("find_contact",true);
    slip_client_  = new SlipClient("slip_servo",true);
    force_client_  = new ForceClient("force_servo",true);


    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the gripper_action action server to come up");
    }

    while(!contact_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the find_contact action server to come up");
    }
    
    while(!slip_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the slip_servo action server to come up");
    }    
    

    while(!force_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the force_servo action server to come up");
    }    
 
  }

  ~Gripper(){
    delete gripper_client_;
    delete contact_client_;
    delete slip_client_;
    delete force_client_;
  }

  //Open the gripper
  void open(double position_open){

    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    open.command.position = position_open;    // position open (9 cm)
    open.command.max_effort = -1.0;

    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult(ros::Duration(4.0));
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }



  //Hold somethign with a constant force in the gripper
  int hold( double holdForce){
    pr2_gripper_sensor_msgs::PR2GripperForceServoGoal squeeze;
    squeeze.command.fingertip_force = holdForce;   // hold with X N of force
    
    ROS_INFO("Sending hold goal");
    force_client_->sendGoal(squeeze);
    force_client_->waitForResult(ros::Duration(6.0));
    if(force_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Stable force was achieved");
    else
      ROS_INFO("Stable force was NOT achieved");
    return force_client_->getResult()->data.rtstate.realtime_controller_state;
  }

  //Find two contacts and go into force control mode
  int findTwoContacts(double *contact_force){
    pr2_gripper_sensor_msgs::PR2GripperFindContactGoal findTwo;
    findTwo.command.contact_conditions = findTwo.command.BOTH;  // close until both fingers contact
    findTwo.command.zero_fingertip_sensors = true;   // zero fingertip sensor values before moving
 
    ROS_INFO("Sending find 2 contact goal");
    contact_client_->sendGoal(findTwo);
    contact_client_->waitForResult(ros::Duration(5.0));
    if(contact_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Contact found. Left: %d, Right: %d", contact_client_->getResult()->data.left_fingertip_pad_contact, 
           contact_client_->getResult()->data.right_fingertip_pad_contact);
      ROS_INFO("Contact force. Left: %f, Right: %f", contact_client_->getResult()->data.left_fingertip_pad_force, 
           contact_client_->getResult()->data.right_fingertip_pad_force);
    }
    else
      ROS_INFO("The gripper did not find a contact.");
    *contact_force = (contact_client_->getResult()->data.right_fingertip_pad_force + contact_client_->getResult()->data.left_fingertip_pad_force)/2.0;

    return contact_client_->getResult()->data.rtstate.realtime_controller_state;

  }
  
  
  //Slip servo the robot
  void slipServo(){
    pr2_gripper_sensor_msgs::PR2GripperSlipServoGoal slip_goal;

    ROS_INFO("Slip Servoing");
    slip_client_->sendGoal(slip_goal);
  }  

};



class PR2GripperGrabAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<pr2_gripper_sensor_msgs::PR2GripperGrabAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  pr2_gripper_sensor_msgs::PR2GripperGrabFeedback feedback_;
  pr2_gripper_sensor_msgs::PR2GripperGrabResult result_;
  Gripper gripper;

public:

  PR2GripperGrabAction(std::string name) :
    as_(nh_, name, boost::bind(&PR2GripperGrabAction::executeCB, this, _1)),
    action_name_(name)
  {
  }

  ~PR2GripperGrabAction(void)
  {
  }

  void executeCB(const pr2_gripper_sensor_msgs::PR2GripperGrabGoalConstPtr &goal)
  {
      double close_speed; 
      if(!nh_.getParam("close_speed", close_speed))
        ROS_ERROR("No close_speed given in namespace: '%s')", nh_.getNamespace().c_str());

      double fingertip_force_limit; 
      if(!nh_.getParam("fingertip_force_limit", fingertip_force_limit))
        ROS_ERROR("No fingertip_force_limit given in namespace: '%s')", nh_.getNamespace().c_str());

      double position_open; 
      if(!nh_.getParam("position_open", position_open))
        ROS_ERROR("No position_open given in namespace: '%s')", nh_.getNamespace().c_str());


      // helper variables
      bool prempted = false;
      double contact_force, servo_force;
    
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
      
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        prempted = true;
      }
      if(!prempted)
      {
        feedback_.data.rtstate.realtime_controller_state = gripper.findTwoContacts(&contact_force);
        as_.publishFeedback(feedback_);
      }
      
      ROS_INFO("Find contact found force: %f",contact_force);
       // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        prempted = true;
      }
      if(!prempted)
      {
        servo_force = (contact_force/close_speed)*goal->command.hardness_gain;
        if(servo_force > fingertip_force_limit && fingertip_force_limit > 0)
          servo_force = fingertip_force_limit;
        feedback_.data.rtstate.realtime_controller_state = gripper.hold(servo_force);
        as_.publishFeedback(feedback_);
      }
      
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        prempted = true;
      }
      if(!prempted)
        gripper.slipServo();

      result_.data.rtstate.realtime_controller_state = feedback_.data.rtstate.realtime_controller_state;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      if(!prempted)
        as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "grab");

  PR2GripperGrabAction Grab("grab");
  ros::spin();

  return 0;
}

