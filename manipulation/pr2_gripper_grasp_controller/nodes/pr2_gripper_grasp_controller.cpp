/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>

#include <sensor_msgs/JointState.h>

#include <object_manipulation_msgs/GraspHandPostureExecutionAction.h>
#include <object_manipulation_msgs/GraspStatus.h>

static const std::string JOINT_STATES_TOPIC = "joint_states";

//! Converts GraspHandPostureExecution goals or queries into controller commands for the PR2 or LCG grippers
/*! GraspHandPostureExecution reasons in terms of joint values, while the PR2 gripper (and for now the
  LCG) controllers reason in terms of gap between the fingers. This node makes the translation.

  For both grippers, only looks at the first joint value in the grasp, and uses it to convert to a gap value.
 */
class PR2GripperGraspController
{
private:

  //! The root namespace node handle
  ros::NodeHandle root_nh_;

  //! The private namespace node handle
  ros::NodeHandle priv_nh_;

  //! Action client for moving the PR2 gripper
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> *gripper_action_client_;

  //! Action server for the grasp poture action
  actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction> *action_server_;

  //! Server for the posture query service
  ros::ServiceServer query_srv_;

  //! The name of the virtual joint of the gripper 
  std::string gripper_virtual_joint_name_;

  //! Whether we are in simulation or on a real robot
  bool sim_;

  //! How long we wait for the gripper to settle in simulation
  double sim_wait_;

  //-------------------------------- Constants ---------------------------------

  //! Gap value for gripper fully open
  double gripper_open_gap_value_;
  //! Default gap value for gripper fully open
  static const double DEFAULT_GRIPPER_OPEN;

  //! Gap value for gripper fully closed
  double gripper_closed_gap_value_;
  //! Default gap value for gripper fully closed
  static const double DEFAULT_GRIPPER_CLOSED;

  //! A value of [rl]_gripper_joint below this indicates there is nothing inside the gripper
  double gripper_object_presence_threshold_;  
  //! Default value in case this parameter is not in the launch file
  static const double DEFAULT_GRIPPER_OBJECT_PRESENCE_THRESHOLD;

  //! Max effort used when achieving a position (e.g. pre-grasp or release) without epxecting an object
  double gripper_max_effort_;
  //! Default max effort for achieving a position
  static const double DEFAULT_GRIPPER_MAX_EFFORT;

  //! The type of gripper, for now "pr2" or "lcg"
  std::string gripper_type_;

  //------------------------------ Functionality -------------------------------

  //! Returns the current value of the gripper joint
  bool getGripperValue(double &value)
  {
    //get the joint states
    sensor_msgs::JointState::ConstPtr joint_states = 
      ros::topic::waitForMessage<sensor_msgs::JointState>(JOINT_STATES_TOPIC, root_nh_);
    if (!joint_states)
    {
      ROS_ERROR("pr2 gripper grasp status: joint states not received");
      return false;
    }
    
    //find the gripper joint
    size_t i;
    for (i=0; i<joint_states->name.size(); i++)
    {
      if (joint_states->name[i] == gripper_virtual_joint_name_) break;
    }
    if (i==joint_states->name.size())
    {
      ROS_ERROR("pr2_gripper grasp status: gripper joint %s not found in joint state", 
                gripper_virtual_joint_name_.c_str());
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

  //! Given a value of a gripper joint, converts that to a value of the gap between the fingers
  /*! This is different for the PR2 gripper and LCG */
  double jointToGap(double jointValue)
  {
    if (gripper_type_ == "pr2") return jointValue / 0.5 * 0.0857;
    else if (gripper_type_ == "lcg") return 2.0 * ( cos(jointValue)*60.0 + 17.5 - 6.0 ) * 1.0e-3;
    else {ROS_ERROR_STREAM("pr2_gripper_grasp_controller: unknown gripper type: " << gripper_type_); return 0.0;}
  }

  void executeCB(const object_manipulation_msgs::GraspHandPostureExecutionGoalConstPtr &goal)
  {
    //prepare the gripper command based on the goal
    pr2_controllers_msgs::Pr2GripperCommandGoal gripper_command;
    switch (goal->goal)
    {
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP:
      if (goal->grasp.grasp_posture.position.empty())
      {
        ROS_ERROR("pr2 gripper grasp execution: position vector empty in requested grasp");
        action_server_->setAborted();
        return;
      }
      gripper_command.command.position = gripper_closed_gap_value_;
      //gripper_command.command.position = jointToGap( goal->grasp.grasp_posture.position[0] );
      if (goal->grasp.grasp_posture.effort.empty())
      {
        if(goal->max_contact_force > 0)
        {
	  ROS_INFO("limiting max contact force to %f", goal->max_contact_force);
          gripper_command.command.max_effort = goal->max_contact_force;
        }
        else gripper_command.command.max_effort = gripper_max_effort_;
        ROS_WARN("pr2 gripper grasp execution: effort vector empty in requested grasp, using max force");
        //action_server_->setAborted();
        //return;
      }
      //we use the effort in the first joint for going to the grasp
      else 
      {
        if(goal->grasp.grasp_posture.effort.at(0) < goal->max_contact_force || goal->max_contact_force == 0)
          gripper_command.command.max_effort = goal->grasp.grasp_posture.effort.at(0);
        else gripper_command.command.max_effort = goal->max_contact_force;
      }
      break;
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::PRE_GRASP:
      if (goal->grasp.pre_grasp_posture.position.empty())
      {
        ROS_ERROR("pr2 gripper grasp execution: position vector empty in requested pre_grasp");
        action_server_->setAborted();
        return;
      }
      //gripper_command.command.position = GRIPPER_OPEN;
      gripper_command.command.position = jointToGap( goal->grasp.pre_grasp_posture.position.at(0) );
      ROS_DEBUG("pre grasp posture opening is %.3f", gripper_command.command.position);
      if (goal->grasp.pre_grasp_posture.effort.empty())
      {
        gripper_command.command.max_effort = gripper_max_effort_;
        ROS_WARN("pr2 gripper grasp execution: effort vector empty in requested pre_grasp, using max");
      }
      else 
      {
        gripper_command.command.max_effort = goal->grasp.pre_grasp_posture.effort.at(0);
      }
      break;      
    case object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE:
      gripper_command.command.position = gripper_open_gap_value_;
      gripper_command.command.max_effort = gripper_max_effort_;
      break;
    default:
      ROS_ERROR("pr2 gripper grasp execution: unknown goal code (%d)", goal->goal);
      action_server_->setAborted();
      return;
    }

    //check the requested effort
    if (gripper_command.command.max_effort > gripper_max_effort_)
    {
      ROS_WARN("pr2 gripper grasp execution: effort exceeds max value for gripper, reducing to max");
      gripper_command.command.max_effort = gripper_max_effort_;
    }

    //send the command to the gripper
    gripper_action_client_->sendGoal(gripper_command);
    if (!sim_) 
    {
      gripper_action_client_->waitForResult();
    }
    else 
    {
      //simulated gripper has a habit of never settling and thus never returning
      //so in sim we just continue after a hard-coded interval here
      bool withinWait = gripper_action_client_->waitForResult(ros::Duration(sim_wait_));
      if (!withinWait)
      {
        ROS_WARN("pr2 gripper grasp execution: controller has not returned within limit, "
                 "but since we are in simulation we are continuing anyway.");
      }
    }

    //process the result
    if(gripper_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
    {
      ROS_INFO("Gripper goal achieved");
      action_server_->setSucceeded();
    } 
    else 
    {
      if (goal->goal == object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP)
      {
        //this is probably correct behavior, since there is an object in the gripper
        //we can not expect to reach our goal
        action_server_->setSucceeded();
      }
      else
      {
        ROS_WARN("Gripper goal not achieved for pre-grasp or release");
        //this might become a failure later, for now we're still investigating
        action_server_->setSucceeded();
      }
    }
  }

  bool serviceCallback(object_manipulation_msgs::GraspStatus::Request &request,
		       object_manipulation_msgs::GraspStatus::Response &response)
  {
    double gripper_value;
    if (!getGripperValue(gripper_value)) return false;    

    double min_gripper_opening = gripper_object_presence_threshold_;
    if (gripper_value < min_gripper_opening) 
    {
      ROS_INFO("Gripper grasp query false: gripper value %f below threshold %f", 
	       gripper_value, min_gripper_opening);
      response.is_hand_occupied = false;
    }
    else 
    {
      ROS_DEBUG("Gripper grasp query true: gripper value %f above threshold %f", 
	       gripper_value, min_gripper_opening);
      response.is_hand_occupied = true;
    }
    return true;
  }
		       

public:
  PR2GripperGraspController() :
    root_nh_(""),
    priv_nh_("~")
  {
    std::string gripper_action_name = root_nh_.resolveName("gripper_action_name");
    gripper_action_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>
      (gripper_action_name, true);
    while(!gripper_action_client_->waitForServer(ros::Duration(2.0)) && root_nh_.ok())
    {
      ROS_INFO_STREAM("Waiting for gripper action client on topic " << gripper_action_name);
    }
    if (!root_nh_.ok()) exit(0);
    ROS_INFO_STREAM("Using gripper action client on topic " << gripper_action_name);

    priv_nh_.param<std::string>("gripper_virtual_joint_name", gripper_virtual_joint_name_, "undefined");
    if ( gripper_virtual_joint_name_ == "undefined" )
    {
      ROS_ERROR("Gripper virtual joint name not defined; set the parameter "
                "gripper_virtual_joint_name in the launch file");
      exit(0);
    }

    priv_nh_.param<double>("gripper_object_presence_threshold", gripper_object_presence_threshold_,
			   DEFAULT_GRIPPER_OBJECT_PRESENCE_THRESHOLD);
    priv_nh_.param<double>("gripper_open_gap_value", gripper_open_gap_value_, DEFAULT_GRIPPER_OPEN);
    priv_nh_.param<double>("gripper_closed_gap_value", gripper_closed_gap_value_, DEFAULT_GRIPPER_CLOSED);
    priv_nh_.param<double>("gripper_max_effort", gripper_max_effort_, DEFAULT_GRIPPER_MAX_EFFORT);
    priv_nh_.param<std::string>("gripper_type", gripper_type_, "pr2");
    priv_nh_.param<bool>("sim", sim_, false);
    priv_nh_.param<double>("sim_wait", sim_wait_, 8.0);

    std::string query_service_name = root_nh_.resolveName("grasp_query_name");   
    query_srv_ = root_nh_.advertiseService(query_service_name, &PR2GripperGraspController::serviceCallback, this);    
    ROS_INFO_STREAM("pr2_gripper grasp query service started on topic " << query_service_name);

    std::string posture_action_name = root_nh_.resolveName("posture_action_name");
    action_server_ = new actionlib::SimpleActionServer<object_manipulation_msgs::GraspHandPostureExecutionAction>
      (root_nh_, posture_action_name, boost::bind(&PR2GripperGraspController::executeCB, this, _1), false);
    action_server_->start();
    ROS_INFO_STREAM("pr2_gripper grasp hand posture action server started on topic " << posture_action_name);
  }

  ~PR2GripperGraspController()
  {
    delete action_server_;
    delete gripper_action_client_;
  }

};

//Note that all these values are set for the PR2 gripper
const double PR2GripperGraspController::DEFAULT_GRIPPER_OPEN = 0.086;
const double PR2GripperGraspController::DEFAULT_GRIPPER_CLOSED = 0.0;
const double PR2GripperGraspController::DEFAULT_GRIPPER_OBJECT_PRESENCE_THRESHOLD = 0.0021;
const double PR2GripperGraspController::DEFAULT_GRIPPER_MAX_EFFORT = 100;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_gripper_grasp_controller");
  PR2GripperGraspController node;
  ros::spin();
  return 0;  
}
