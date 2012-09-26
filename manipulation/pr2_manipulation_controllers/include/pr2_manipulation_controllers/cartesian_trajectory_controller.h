/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/*
 * Author: Wim Meeussen
 */

#ifndef CARTESIAN_TRAJECTORY_CONTROLLER_H
#define CARTESIAN_TRAJECTORY_CONTROLLER_H

#include <vector>
#include <string>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pr2_controller_interface/controller.h>
#include <robot_mechanism_controllers/cartesian_pose_controller.h>
#include <pr2_manipulation_controllers/MoveToPose.h>
#include <std_srvs/Empty.h>
#include <boost/scoped_ptr.hpp>

using namespace controller;

namespace pr2_manipulation_controllers {


class CartesianTrajectoryController : public pr2_controller_interface::Controller
{
public:
  CartesianTrajectoryController();
  ~CartesianTrajectoryController();

  bool init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle& n);

  void starting();
  void update();
  bool moveTo(const geometry_msgs::PoseStamped& pose, 
              const geometry_msgs::Twist& tolerance=geometry_msgs::Twist(), double duration=0);
  bool checkMoving(pr2_manipulation_controllers::CheckMoving::Request &req, pr2_manipulation_controllers::CheckMoving::Response &res);

private:
  KDL::Frame getPose();
  void TransformToFrame(const tf::Transform& trans, KDL::Frame& frame);

  // topic
  void command(const tf::MessageFilter<geometry_msgs::PoseStamped>::MConstPtr& pose_msg);

  // service calls
  bool moveTo(pr2_manipulation_controllers::MoveToPose::Request &req, pr2_manipulation_controllers::MoveToPose::Response &resp);
  bool preempt(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

  ros::NodeHandle node_;
  ros::ServiceServer move_to_srv_, preempt_srv_, check_moving_srv_;

  std::string controller_name_;
  ros::Time last_time_, time_started_;
  double time_passed_, max_duration_;
  bool is_moving_, request_preempt_, exceed_tolerance_;

  KDL::Frame pose_begin_, pose_end_, pose_current_;
  KDL::Twist twist_current_, tolerance_;

  // robot structure
  pr2_mechanism_model::RobotState *robot_state_;       
  pr2_mechanism_model::Chain chain_;

  // kdl stuff for kinematics
  KDL::Chain             kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> jnt_to_pose_solver_;
  KDL::JntArray          jnt_pos_;

  // motion profiles
  std::vector<KDL::VelocityProfile_Trap> motion_profile_;

  // pose controller
  CartesianPoseController* pose_controller_;

  tf::TransformListener tf_;
  //boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PoseStamped> > command_notifier_;
  //ros::Subscriber command_sub_;

  std::string root_name_;
};


}
#endif
