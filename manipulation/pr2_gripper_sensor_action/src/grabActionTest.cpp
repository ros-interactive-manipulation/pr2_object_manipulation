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
// @brief: grabActionTest.cpp - example for using the grab
//         action server

#include <ros/ros.h>
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction> GrabClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperReleaseAction> ReleaseClient;


class Gripper{
private:
  GrabClient* grab_client_;
  ReleaseClient* release_client_;

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    grab_client_  = new GrabClient("r_gripper_sensor_controller/grab",true);
    release_client_  = new ReleaseClient("r_gripper_sensor_controller/release",true);

    while(!grab_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/grab action server to come up");
    }

    while(!release_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/release action server to come up");
    }
  }

  ~Gripper(){
    delete grab_client_;
    delete release_client_;
  }

  //Find contact on both fingers and go into slip-servo control mode
  void grab(){
    pr2_gripper_sensor_msgs::PR2GripperGrabGoal grip;
    grip.command.hardness_gain = 0.03;
    
    ROS_INFO("Sending grab goal");
    grab_client_->sendGoal(grip);
    grab_client_->waitForResult(ros::Duration(20.0));
    if(grab_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully completed Grab");
    else
      ROS_INFO("Grab Failed");
  }

  void release(){
    pr2_gripper_sensor_msgs::PR2GripperReleaseGoal place;
    // set the robot to release on a figner-side impact, fingerpad slip, or acceleration impact with hand/arm
    place.command.event.trigger_conditions = place.command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;  
    // set the acceleration impact to trigger on to 5 m/s^2
    place.command.event.acceleration_trigger_magnitude = 5.0; 
    // set our slip-gain to release on to .005
    place.command.event.slip_trigger_magnitude = .01;


    ROS_INFO("Waiting for object placement contact...");
    release_client_->sendGoal(place);
    release_client_->waitForResult();
    if(release_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Release Success");
    else
      ROS_INFO("Place Failure");

  }


};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");

  Gripper gripper;

  // wait a few seconds so we can put something in the robot's gripper
  sleep(5.0);

  // grab something
  gripper.grab();

  // move the robot arm all ove the place here!
  sleep(5.0);
  
  // now that we've decided we want to look for contact and let go
  gripper.release();
  

  return 0;
}
