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

#include <pr2_wrappers/base_client.h>

namespace pr2_wrappers {


BaseClient::BaseClient(ros::NodeHandle &nh, const ros::Duration &timeout, tf::TransformListener *tfl) :
    delete_tfl_(false),
    move_base_client_("move_base", true),
    move_base_active_(false),
    last_linear_(0,0,0),
    last_angular_(0,0,0),
    timeout_(timeout),
    local_costmap_reset_srv_("/move_base/clear_costmaps")
{
  nh_ = nh;
  tfl_ = tfl;

  // Create a new tfl if we didn't receive a pointer to one, and mark it for deletion.
  if(!tfl_)
  {
    tfl_ = new tf::TransformListener();
    delete_tfl_ = true;
  }

  //set up the publisher for twist messages
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("base_controller/command", 1);

  //set up the publisher for pose estimates
  pose_estimate_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

  // Subscribe to the status message for the move_base action server
  sub_nav_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("move_base/status", 10, 
                                                                   boost::bind(&BaseClient::moveBaseStatusCB, this, _1));

  // Subscribe to the result message for the move_base action server
  sub_nav_result_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("move_base/result", 10, 
                                                                        boost::bind(&BaseClient::moveBaseResultCB, this, _1));

  // Setup the timer callback for publishing updates
  publish_timer_ = nh_.createTimer(ros::Duration(0.05), boost::bind(&BaseClient::publishTwist, this, _1));
}

BaseClient::~BaseClient(){
  if(delete_tfl_)
    delete tfl_;
}

//! Public method to check if the action server has a goal
bool BaseClient::hasGoal()
{
  return move_base_active_;
}

////! Queries the move_base server
//void BaseClient::queryGoalState()
//{
//  actionlib::SimpleClientGoalState state = move_base_client_.getState();
//  move_base_active_ = ( state == state.ACTIVE || state == state.PENDING );
//}

//! Sends a "cancel all goals" request to the move_base server
void BaseClient::cancelGoals()
{
  ROS_INFO("Canceling all move_base goals!");
  move_base_client_.client().cancelAllGoals();
}

//! Callback for receiving status of move_base_client
void BaseClient::moveBaseResultCB( const move_base_msgs::MoveBaseActionResultConstPtr &result)
{
  if(result->status.status == result->status.SUCCEEDED)
  {
    ROS_INFO("Move base reports succeeded, text: %s", result->status.text.c_str());
  }
  else if(result->status.status == result->status.PREEMPTED)
  {
    ROS_INFO("Move base reports preempted, text: %s", result->status.text.c_str());
  }
  else if(result->status.status == result->status.ABORTED)
  {
    ROS_INFO("Move base reports aborted, text: %s", result->status.text.c_str());
  }
  else if(result->status.status == result->status.REJECTED)
  {
    ROS_INFO("Move base reports goal rejected, text: %s", result->status.text.c_str());
  }
  else
  {
    ROS_INFO("Move base result was %d, text: %s", result->status.status, result->status.text.c_str());
  }
}

//! Callback for receiving status of move_base_client
void BaseClient::moveBaseStatusCB( const actionlib_msgs::GoalStatusArrayConstPtr &array)
{
  size_t num = array->status_list.size();
  bool is_active = false;
  for(size_t i = 0; i < num; i++)
  {
    actionlib_msgs::GoalStatus goal_status = array->status_list[i];
    if(goal_status.status == goal_status.ACTIVE || goal_status.status == goal_status.PENDING)
    {
      ROS_DEBUG("Got a move_base goal status that is active or pending!");
      is_active = true;
    }
    else
    {
      ROS_DEBUG("Base client for goal %zd got status update: %d", i, goal_status.status);
    }
  }
  move_base_active_ = is_active;
}

//! Stores a new command to send to the base
void BaseClient::applyTwist(const geometry_msgs::TwistStamped &ts)
{
  //store the base command
  base_cmd_ = ts;
}

//! Publishes the most recent twist message to the base controller
void BaseClient::publishTwist(const ros::TimerEvent &e)
{
  //use the command if it not too old, or use zero instead
  geometry_msgs::TwistStamped cmd = base_cmd_;
  if( ( (e.current_real - base_cmd_.header.stamp) > timeout_)  )
  {
    cmd.twist.linear.x = cmd.twist.linear.y = cmd.twist.linear.z = 0.0;
    cmd.twist.angular.x = cmd.twist.angular.y = cmd.twist.angular.z = 0.0;
  }

  tf::Vector3 linear;
  tf::Vector3 angular;

  tf::vector3MsgToTF(cmd.twist.linear, linear);
  tf::vector3MsgToTF(cmd.twist.angular, angular);

  float nu = 0.4;

  last_linear_ = nu*linear + (1.0 -nu)*last_linear_;
  last_angular_ = nu*angular + (1.0 -nu)*last_angular_;

  //if we have a zero command are we've already sent it, nothing to do
  double EPS = 1.0e-10;
  if (  linear.length() < EPS &&  last_linear_.length() < EPS && 
       angular.length() < EPS && last_angular_.length() < EPS )
  {
    return;
  }

  geometry_msgs::Twist newcmd;
  tf::vector3TFToMsg(last_linear_, newcmd.linear);
  tf::vector3TFToMsg(last_angular_, newcmd.angular);
  cmd_vel_pub_.publish(newcmd);
}

void BaseClient::sendNavGoal( const geometry_msgs::PoseStamped &ps)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = ps;
  ROS_DEBUG_STREAM("Sending move_base_goal \n" << ps);
  // aleeper: I think we can just send the goal here and not wait for a result, because we already have
  //          functionality above for monitoring the state of the nav stack.
  move_base_client_.client().sendGoal( goal ); //, boost::bind(&BaseClient::testGripperResultCallback, this, _1, _2));
}

void BaseClient::sendPoseEstimate(const geometry_msgs::PoseWithCovarianceStamped &ps)
{
  ROS_DEBUG_STREAM("Sending pose estimate. \n" << ps);
  pose_estimate_pub_.publish(ps);
}

void BaseClient::clearLocalCostmap( )
{


  std_srvs::Empty srv;
  if (!local_costmap_reset_srv_.client().call(srv)) ROS_ERROR("failed to call costmap reset client");

}

} // namespace
