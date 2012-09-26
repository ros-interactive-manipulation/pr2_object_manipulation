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

#ifndef _BASE_CLIENT_H_
#define _BASE_CLIENT_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <object_manipulator/tools/service_action_wrappers.h>
#include <std_srvs/Empty.h>

namespace pr2_wrappers {

class BaseClient{
private:

  //! The node handle we'll be using
  ros::NodeHandle nh_;

  //! Publisher for twist messages
  ros::Publisher cmd_vel_pub_;

  //! Publisher for initial pose estimates; default topic is "/initialpose".
  ros::Publisher pose_estimate_pub_;

  //! TF listener pointer; a new one only gets created if one is not passed in on construction.
  tf::TransformListener *tfl_;

  //! A flag to determine whether to delete the TF listener on destruction.
  bool delete_tfl_;

  //! A subscriber for listening to the status of the nav stack.
  ros::Subscriber sub_nav_status_;

  //! A subscriber for the result of the nav stack
  ros::Subscriber sub_nav_result_;

  //! A actionlib client to the move_base action server.
  object_manipulator::ActionWrapper<move_base_msgs::MoveBaseAction> move_base_client_;
  //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;

  //! A bool representing if the move_base action server is active.
  bool move_base_active_;


  geometry_msgs::TwistStamped base_cmd_;
  tf::Vector3 last_linear_;
  tf::Vector3 last_angular_;

  ros::Duration timeout_;

  ros::Timer publish_timer_;

  // Service clients
  object_manipulator::ServiceWrapper<std_srvs::Empty> local_costmap_reset_srv_;

  //! Publish last stored twist message at each timer interval; has a (very CRITICAL) time-out for safety.
  void publishTwist(const ros::TimerEvent &e);

  //! Callback for receiving status of move base client
  void moveBaseStatusCB( const actionlib_msgs::GoalStatusArrayConstPtr &array);

  //! Callback for receiving results of move base client
  void moveBaseResultCB( const move_base_msgs::MoveBaseActionResultConstPtr &result);

public:
  //! Initialization
  BaseClient(ros::NodeHandle &nh, const ros::Duration &timeout, tf::TransformListener *tfl = 0);

  ~BaseClient();

  //! Commands a straight-up movement command to the base.
  void applyTwist(const geometry_msgs::TwistStamped &twist);

  //! Returns true if the nav stack is active.
  bool hasGoal();

  //! Deprecated...
  //void queryGoalState();

  //! Tells the nav stack to STOP!
  void cancelGoals();

  void sendNavGoal(const geometry_msgs::PoseStamped &ps);

  void sendPoseEstimate(const geometry_msgs::PoseWithCovarianceStamped &ps);

  void clearLocalCostmap( );


};

}


#endif
