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

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

#include "pr2_marker_control/generate_robot_model.h"

#include <ros/ros.h>
#include <math.h>

#include <arm_navigation_msgs/GetStateValidity.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
   
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

#include <interactive_marker_helpers/interactive_marker_helpers.h>
#include <pr2_object_manipulation_msgs/GetNavPoseAction.h>
#include <pr2_marker_control/cloud_handler.h>
#include <pr2_marker_control/marker_control.h>

using namespace object_manipulator;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace im_helpers;

////////////////////////////////////////////////////////////////////////////////////

// First define a useful helper function...
/** \brief Transforms a StampedTransform by some offset (expressed in the original transform frame).
* \param st The original transform
* \param offset The offset transformation to apply
*/
void offsetPS(tf::StampedTransform &st, const tf::Transform &offset)
{
  tf::Transform pose = tf::Transform(st.getRotation(), st.getOrigin());
  pose = pose * offset;
  st.setData(pose);
}

//! Translate the control pose to the wrist.
geometry_msgs::PoseStamped PR2MarkerControl::toolToWrist(const geometry_msgs::PoseStamped &ps)
{
  geometry_msgs::PoseStamped out;
  out.header = ps.header;
  tf::Transform T, P;
  tf::poseMsgToTF(ps.pose, P);
  tf::poseMsgToTF(tool_frame_offset_, T);
  tf::poseTFToMsg( P*T.inverse(), out.pose);
  //out.pose = object_manipulator::msg::applyShift(ps.pose, tf::Vector3(-0.15,0,0));
  return out;
}

//! Translate to the control pose.
geometry_msgs::PoseStamped PR2MarkerControl::wristToTool(const geometry_msgs::PoseStamped &ps)
{
  geometry_msgs::PoseStamped out;
  out.header = ps.header;
  tf::Transform T, P;
  tf::poseMsgToTF(ps.pose, P);
  tf::poseMsgToTF(tool_frame_offset_, T);
  tf::poseTFToMsg( P*T, out.pose);
  //out.pose = object_manipulator::msg::applyShift(ps.pose, tf::Vector3(-0.15,0,0));
  return out;
}

// This is a convenience function (rather specific to this file) to get the arm name from one of several marker names.
std::string getArmNameFromMarkerName(const std::string &name){
  std::string arm_name = "";
  if( name == "r_upper_arm_link" || name == "r_gripper_palm_link"
      || name == "r_gripper_l_finger_link" || name == "r_gripper_r_finger_link"
      || name == "r_gripper_l_finger_tip_link" || name == "r_gripper_r_finger_tip_link")
    arm_name = "right_arm";
  else if( name == "l_upper_arm_link" || name == "l_gripper_palm_link"
           || name == "l_gripper_l_finger_link" || name == "l_gripper_r_finger_link"
           || name == "l_gripper_l_finger_tip_link" || name == "l_gripper_r_finger_tip_link")
    arm_name = "left_arm";
  else
    ROS_WARN("Marker name [%s] not handled!", name.c_str());

  return arm_name;
}

////////////////////////////////////////////////////////////////////////////////////


PR2MarkerControl::PR2MarkerControl() :
  nh_("/"),
  pnh_("~"),
  server_("pr2_marker_control", "marker_control", false),
  tfl_(nh_),
  torso_client_(),
  base_client_(nh_, ros::Duration(2.0), &tfl_),
  tuck_arms_client_(nh_, ros::Duration(20.0)),
  plugs_client_(nh_, ros::Duration(90.0)),
  check_state_validity_client_("/current_state_validator/get_state_validity"),
  collider_node_reset_srv_("/collider_node/reset"),
  snapshot_client_(&nh_, &tfl_, "interactive_manipulation_snapshot", 
                   "interactive_point_cloud", "interactive_manipulation_snapshot_server", 
                   mechanism_, "odom_combined"),
  object_cloud_left_(&nh_, &tfl_, "in_hand_object_cloud_left", 
                     "in_hand_objects", "in_hand_object_server_left", 
                     mechanism_, "l_wrist_roll_link"),
  object_cloud_right_(&nh_, &tfl_, "in_hand_object_cloud_right", 
                      "in_hand_objects", "in_hand_object_server_right", 
                      mechanism_, "r_wrist_roll_link"),
  base_pose_client_("pr2_interactive_nav_action", true),
  gripper_pose_client_("pr2_interactive_gripper_pose_action", true),
  alignedOdomValid_(false),
  alignedOdomThread_( boost::bind( &PR2MarkerControl::publishAlignedOdom, this ) )
{
  ROS_INFO( "-------------------------- Starting up PR2 MARKER CONTROL application. --------------------------------" );

  ros::Duration(2.0).sleep();

  pnh_.param<bool>("use_right_arm", use_right_arm_, true);
  pnh_.param<bool>("use_left_arm", use_left_arm_, true);
  pnh_.param<bool>("use_state_validator", use_state_validator_, true);

  pnh_.param<double>("gripper_control_linear_deadband",   gripper_control_linear_deadband_,  0.002);
  pnh_.param<double>("gripper_control_angular_deadband",  gripper_control_angular_deadband_, 0.01);
  pnh_.param<double>("update_period",  update_period_, 0.05);
  pnh_.param<double>("cartesian_clip_distance",  cartesian_clip_distance_, 2);
  pnh_.param<double>("cartesian_clip_angle",  cartesian_clip_angle_, 10);
  pnh_.param<std::string>("head_pointing_frame", head_pointing_frame_, "default_head_pointing_frame");
  pnh_.param<std::string>("move_base_node_name", move_base_node_name_, "move_base_node");
  pnh_.param<bool>("nav_3d", using_3d_nav_, false);

  pnh_.param<double>("max_direct_nav_radius",   max_direct_nav_radius_,  0.4);

  nh_.param<int>("interactive_grasping/interface_number", interface_number_, 0);
  if (interface_number_) ROS_INFO("Using interface number %d for grasping study", interface_number_);
  nh_.param<int>("interactive_grasping/task_number", task_number_, 0);
  if (task_number_) ROS_INFO("Using task number %d for grasping study", task_number_);

  pnh_.param<bool>("planar_only", control_state_.planar_only_, false);
  pnh_.param<std::string>("marker_frame", manipulator_base_frame_, "base_link");

  pnh_.param<std::string>("l_gripper_type", l_gripper_type_, "pr2");
  pnh_.param<std::string>("r_gripper_type", r_gripper_type_, "pr2");

  object_cloud_left_sub_ = nh_.subscribe("in_hand_object_left", 1,
                                         &PR2MarkerControl::inHandObjectLeftCallback, this);
  object_cloud_right_sub_ = nh_.subscribe("in_hand_object_right", 1,
                                          &PR2MarkerControl::inHandObjectRightCallback, this);

  ROS_INFO("***************************** %s *****************************", manipulator_base_frame_.c_str());

  if (interface_number_ == 1)
  {
    control_state_.r_gripper_.on_ = true;
    control_state_.posture_r_ = true;
    switchToCartesian();
  }

  in_collision_r_ = false;
  in_collision_l_ = false;

  control_state_.r_gripper_.torso_frame_ = true;
  control_state_.l_gripper_.torso_frame_ = true;
  control_state_.dual_grippers_.torso_frame_ = true;
   
  tf::Transform offset = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0, 0));
  pose_offset_.push_back(offset);
  pose_offset_.push_back(offset);

  dual_gripper_offsets_.push_back(offset);
  dual_gripper_offsets_.push_back(offset);

  dual_pose_offset_ = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));

  tool_frame_offset_ = msg::createPoseMsg(tf::Pose(tf::Quaternion::getIdentity(), tf::Vector3(0.18,0,0)));

  head_goal_pose_.pose.position.x = 1.0;
  head_goal_pose_.pose.position.z = 1.0;
  head_goal_pose_.header.frame_id = "base_link";

  base_goal_pose_.header.frame_id = "base_link";
  base_goal_pose_.pose.orientation.w = 1;

  // Must initialize the menus before creating the markers!
  initMenus();

  // Initialize (or hard "reset") of all markers.
  initAllMarkers();

  spin_timer_ =  nh_.createTimer(ros::Duration(update_period_), boost::bind( &PR2MarkerControl::fastUpdate, this ) );
  slow_sync_timer_ =  nh_.createTimer(ros::Duration(0.5), boost::bind( &PR2MarkerControl::slowUpdate, this ) );
}


void PR2MarkerControl::fastUpdate()
{
  ros::Time now = ros::Time(0);

  tf::StampedTransform stamped;
  geometry_msgs::TransformStamped ts;

  if(control_state_.r_gripper_.on_)
  {
    geometry_msgs::PoseStamped ps;
    static geometry_msgs::PoseStamped last;
    tfl_.waitForTransform("r_wrist_roll_link", "base_link", now, ros::Duration(2.0));
    tfl_.lookupTransform("base_link", "r_wrist_roll_link", now, stamped);
    offsetPS(stamped, pose_offset_[0]);
    tf::transformStampedTFToMsg(stamped, ts);
    ps = object_manipulator::msg::createPoseStampedMsg(ts);
    ps.header.stamp = ros::Time(0);  // Time(0) makes this stay fixed in base_link

    double pos_dist = 0, angle = 0;
    mechanism_.poseDists(ps.pose, last.pose, pos_dist, angle);
    if(pos_dist > gripper_control_linear_deadband_ || angle > gripper_control_angular_deadband_)
    {
      server_.setPose("r_gripper_control", ps.pose, ps.header);
      last = ps;
    }
  }
  if(control_state_.l_gripper_.on_)
  {
    geometry_msgs::PoseStamped ps;
    static geometry_msgs::PoseStamped last;
    tfl_.waitForTransform("l_wrist_roll_link", manipulator_base_frame_, now, ros::Duration(2.0));
    tfl_.lookupTransform(manipulator_base_frame_, "l_wrist_roll_link", now, stamped);
    offsetPS(stamped, pose_offset_[1]);
    tf::transformStampedTFToMsg(stamped, ts);
    ps = object_manipulator::msg::createPoseStampedMsg(ts);
    ps.header.stamp = ros::Time(0); // Time(0) makes this stay fixed in base_link

    double pos_dist = 0, angle = 0;
    mechanism_.poseDists(ps.pose, last.pose, pos_dist, angle);
    if(pos_dist > gripper_control_linear_deadband_ || angle > gripper_control_angular_deadband_)
    {
      server_.setPose("l_gripper_control", ps.pose, ps.header);
      last = ps;
    }
  }

  server_.applyChanges();
}

void PR2MarkerControl::slowUpdate()
{
  static bool last_r_cart, last_l_cart, last_move_base_active;

  bool r_cart = false, l_cart = false;
  bool init_mesh_markers = false, init_all_markers = false, init_control_markers = false;


  if(use_right_arm_)
  {
    r_cart = mechanism_.checkController("r_cart");
    if(! r_cart)
    {
      control_state_.r_gripper_.on_ = false;
      control_state_.posture_r_ = false;
      control_state_.dual_grippers_.on_ = false;
    }
    // It's much smoother if we don't do this!
    //if( r_cart && !control_state_.posture_r_ ) refreshPosture("right_arm");
    //if( r_cart ) refreshPosture("right_arm");
  }
  if(use_left_arm_)
  {
    l_cart = mechanism_.checkController("l_cart");
    if(! l_cart)
    {
      control_state_.l_gripper_.on_ = false;
      control_state_.posture_l_ = false;
      control_state_.dual_grippers_.on_ = false;
    }
    // It's much smoother if we don't do this!
    //if( l_cart && !control_state_.posture_l_ ) refreshPosture("left_arm");
    //if( l_cart ) refreshPosture("left_arm");
  }

//  if( r_cart || l_cart )
//  {
//    if(joint_handle_)       menu_arms_.setCheckState(joint_handle_, MenuHandler::UNCHECKED);
//    if(jtranspose_handle_)  menu_arms_.setCheckState(jtranspose_handle_, MenuHandler::CHECKED);
//  }
//  else
//  {
//    if(joint_handle_)       menu_arms_.setCheckState(joint_handle_, MenuHandler::CHECKED);
//    if(jtranspose_handle_)  menu_arms_.setCheckState(jtranspose_handle_, MenuHandler::UNCHECKED);
//    control_state_.r_gripper_.on_ = false;
//    control_state_.l_gripper_.on_ = false;
//    control_state_.posture_r_ = false;
//    control_state_.posture_l_ = false;
//  }

  bool controller_changed = (r_cart != last_r_cart) || (l_cart != last_l_cart);
  last_r_cart = r_cart;
  last_l_cart = l_cart;

  if(controller_changed)
  {
    //if(tuck_handle_) menu_arms_.setVisible(tuck_handle_, !(r_cart || l_cart) );
    //init_mesh_markers = true;
    init_control_markers = true;
    //init_all_markers = true;
    //initAllMarkers();
  }

  ROS_DEBUG("r_cart: %d  l_cart: %d", r_cart, l_cart);

  // ---------------------------------------------------------------
  // Current state validator stuff

  bool meshes_changed = false;

  bool in_collision_r = !checkStateValidity("right_arm");
  if (in_collision_r_ != in_collision_r) meshes_changed = true;
  in_collision_r_ = in_collision_r;

  bool in_collision_l = !checkStateValidity("left_arm");
  if (in_collision_l_ != in_collision_l) meshes_changed = true;
  in_collision_l_ = in_collision_l;

  init_mesh_markers = meshes_changed;
  //if (meshes_changed) initMeshMarkers();

  // ---------------------------------------------------------------
  // Base client stuff
  bool move_base_active = base_client_.hasGoal();
  if( move_base_active != last_move_base_active ) init_control_markers = true;
  last_move_base_active = move_base_active;


  // ---------------------------------------------------------------
  // Call a re-draw to the necessary markers.
  if(init_all_markers) initAllMarkers();
  else
  {
    if(init_control_markers) initControlMarkers();
    if(init_mesh_markers) initMeshMarkers();
  }
}

//! Finds the state of a given joint in the robot state
double PR2MarkerControl::getJointPosition( std::string name, const arm_navigation_msgs::RobotState& robot_state)
{
  for (size_t i=0; i<robot_state.joint_state.name.size(); i++)
  {
    if (robot_state.joint_state.name[i] == name)
    {
      ROS_ASSERT(robot_state.joint_state.position.size() > i);
      return robot_state.joint_state.position[i];
    }
  }
  ROS_ERROR_STREAM("Joint " << name << " not found in robot state");
  return 0.0;
}

//! Re-initialize only the control markers (like the gripper-dragging frame)
void PR2MarkerControl::initControlMarkers()
{
  ros::Time now = ros::Time::now();
  ROS_INFO("Re-initializing all control markers!");
  arm_navigation_msgs::RobotState robot_state;
  //wait for the service to initialize, for how long it takes
  mechanism_.get_robot_state_client_.client(ros::Duration(-1));
  mechanism_.getRobotState(robot_state);
  
  control_state_.print();
  // Right gripper control
  if(!control_state_.dual_grippers_.on_ && control_state_.r_gripper_.on_ && use_right_arm_ &&
     (interface_number_ == 0 || interface_number_ == 1) )
  {
    tfl_.waitForTransform("r_wrist_roll_link","base_link", now, ros::Duration(2.0));
    tf::StampedTransform stamped;
    geometry_msgs::TransformStamped ts;
    geometry_msgs::PoseStamped ps;
    tfl_.lookupTransform("base_link", "r_wrist_roll_link", now, stamped);
    offsetPS(stamped, pose_offset_[0]);
    tf::transformStampedTFToMsg(stamped, ts);
    ps = msg::createPoseStampedMsg(ts);
    ps.header.stamp = ros::Time(0);   // Time(0) is needed, else the markers seem to end up in random places
    if ( control_state_.planar_only_ ) {
      ROS_INFO("making planar control");
      server_.insert(makePlanarMarker( "r_gripper_control", ps, 0.25, true), 
                     boost::bind( &PR2MarkerControl::updateGripper, this, _1, 0));
    } else {
      server_.insert(make6DofMarker( "r_gripper_control", ps, 0.25, control_state_.r_gripper_.torso_frame_,
				     control_state_.r_gripper_.view_facing_),
		     boost::bind( &PR2MarkerControl::updateGripper, this, _1, 0));
    }
    menu_grippers_.apply(server_, "r_gripper_control");
  }
  else
  {
    server_.erase("r_gripper_control");
  }

  // Left gripper control
  if(!control_state_.dual_grippers_.on_ && control_state_.l_gripper_.on_ && use_left_arm_ &&
     (interface_number_ == 0 || interface_number_ == 1))
  {
    tfl_.waitForTransform("base_link", "odom_combined", now, ros::Duration(2.0));
    tf::StampedTransform tfbaseLink_expressedIn_odomCombined;
    tfl_.lookupTransform("odom_combined", "base_link", now, tfbaseLink_expressedIn_odomCombined);
    tf::Transform odomCombined2_expressedIn_odomCombined = tf::Transform(tfbaseLink_expressedIn_odomCombined.getRotation());
    tf::StampedTransform outST(odomCombined2_expressedIn_odomCombined, now, "odom_combined", "base_aligned_odom_combined"); 
    {
      boost::mutex::scoped_lock lock(alignedOdomMutex_);
      alignedOdom_ = outST;
      alignedOdomValid_ = true;
      tfb_.sendTransform(alignedOdom_);
      tfl_.setTransform(alignedOdom_);
    }
    
    /*
    tfl_.waitForTransform("l_wrist_roll_link", manipulator_base_frame_, now, ros::Duration(2.0));
    tfl_.waitForTransform("base_link", manipulator_base_frame_, now, ros::Duration(2.0));
    tf::StampedTransform tfbaseLink_expressedIn_odomCombined, tfgripper_expressedIn_odomCombined;
    tfl_.lookupTransform(manipulator_base_frame_, "l_wrist_roll_link", now, tfgripper_expressedIn_odomCombined);
    tfl_.lookupTransform(manipulator_base_frame_, "base_link", now, tfbaseLink_expressedIn_odomCombined);
    tf::Transform gripper_expressedIn_odomCombined = tf::Transform(tfgripper_expressedIn_odomCombined.getRotation(), tfgripper_expressedIn_odomCombined.getOrigin());
    tf::Transform odomCombined2_expressedIn_odomCombined = tf::Transform(tfbaseLink_expressedIn_odomCombined.getRotation());
    tf::Transform gripper_expressedIn_odomCombined2 = gripper_expressedIn_odomCombined * odomCombined2_expressedIn_odomCombined.inverse();
    
    geometry_msgs::PoseStamped psGripper_expressedIn_odomCombined2;
    tf::poseTFToMsg(gripper_expressedIn_odomCombined2, psGripper_expressedIn_odomCombined2.pose);
    psGripper_expressedIn_odomCombined2.header.stamp = ros::Time(0);
    
    server_.insert(make6DofMarker( "l_gripper_control", psGripper_expressedIn_odomCombined2, 
				   0.25, control_state_.l_gripper_.torso_frame_,
                                   control_state_.l_gripper_.view_facing_),
                   boost::bind( &PR2MarkerControl::updateGripper, this, _1, 1 ));
    menu_grippers_.apply(server_, "l_gripper_control");
    */
    
    
    tfl_.waitForTransform("l_wrist_roll_link", manipulator_base_frame_, now, ros::Duration(2.0));
    tf::StampedTransform stamped;
    geometry_msgs::TransformStamped ts;
    geometry_msgs::PoseStamped ps;
    tfl_.lookupTransform(manipulator_base_frame_, "l_wrist_roll_link", now, stamped);
    offsetPS(stamped, pose_offset_[1]);
    tf::transformStampedTFToMsg(stamped, ts);
    ps = msg::createPoseStampedMsg(ts);
    ps.header.stamp = ros::Time(0); // Time(0) is needed, else the markers seem to end up in random places
    server_.insert(make6DofMarker( "l_gripper_control", ps, 0.25, control_state_.l_gripper_.torso_frame_,
                                   control_state_.l_gripper_.view_facing_),
                   boost::bind( &PR2MarkerControl::updateGripper, this, _1, 1 ));
    menu_grippers_.apply(server_, "l_gripper_control");
    
    ROS_DEBUG_STREAM("init marker in frame " << ps.header.frame_id << "(" 
		    << ps.pose.position.x << ", " 
		    << ps.pose.position.y << ", " 
		    << ps.pose.position.z << ")" );
 
  }
  else
  {
    server_.erase("l_gripper_control");
  }

  // Dual-gripper control
  if(control_state_.dual_grippers_.on_)
  {
//    // TODO should this update with gripper poses, or be more of a fixed thing?
//    // If it auto updates it might lead to stability issues...
//    tfl_.waitForTransform("r_gripper_tool_frame","base_link", now, ros::Duration(2.0));
//    tfl_.waitForTransform("l_gripper_tool_frame","base_link", now, ros::Duration(2.0));
//    tf::StampedTransform stamped;
//    geometry_msgs::TransformStamped ts;
//    geometry_msgs::PoseStamped ps;
//    tfl_.lookupTransform("base_link", "r_wrist_roll_link", now, stamped);
//    offsetPS(stamped, pose_offset_[1]);
//    tf::transformStampedTFToMsg(stamped, ts);
//    ps = msg::createPoseStampedMsg(ts);

    tf::Transform dual_grippers_frame_tf;
    tf::poseMsgToTF(dual_grippers_frame_.pose, dual_grippers_frame_tf);
    tf::Transform offset_frame = dual_grippers_frame_tf * dual_pose_offset_.inverse();
    geometry_msgs::Pose offset_pose;
    tf::poseTFToMsg(offset_frame, offset_pose);
    geometry_msgs::PoseStamped offset_ps = object_manipulator::msg::createPoseStampedMsg(offset_pose,
                                                                                  dual_grippers_frame_.header.frame_id,
                                                                                  ros::Time(0));
    server_.insert(make6DofMarker( "dual_gripper_control", offset_ps, 0.25, 
                                   control_state_.dual_grippers_.torso_frame_, false),
                   boost::bind( &PR2MarkerControl::updateDualGripper, this, _1 ));
    menu_dual_grippers_.apply(server_, "dual_gripper_control");
  }
  else
  {
    server_.erase("dual_gripper_control");
  }


  if(control_state_.posture_r_ && use_right_arm_ &&
     (interface_number_ <= 2))
  {
      tf::StampedTransform stamped;
      geometry_msgs::TransformStamped ts;
      geometry_msgs::PoseStamped ps;
      tfl_.waitForTransform("r_shoulder_pan_link","base_link", now, ros::Duration(2.0));
      tfl_.lookupTransform("base_link", "r_shoulder_pan_link", now, stamped);
      tf::transformStampedTFToMsg(stamped, ts);
      float angle = getJointPosition("r_upper_arm_roll_joint", robot_state);
      ts.transform.rotation = msg::createQuaternionMsg( tf::Quaternion( tf::Vector3(1,0,0), angle) );
      ps = msg::createPoseStampedMsg(ts);
      ps.header.stamp = ros::Time(0);
      server_.insert(makePostureMarker( "r_posture_control", ps, 0.55, false, false),
                 boost::bind( &PR2MarkerControl::updatePosture, this, _1, 0 ));
  }
  else
  {
      server_.erase("r_posture_control");
  }

  if(control_state_.posture_l_ && use_left_arm_ &&
     (interface_number_ <= 2))
  {
      tf::StampedTransform stamped;
      geometry_msgs::TransformStamped ts;
      geometry_msgs::PoseStamped ps;
      tfl_.waitForTransform("l_shoulder_pan_link",manipulator_base_frame_, now, ros::Duration(2.0));
      tfl_.lookupTransform(manipulator_base_frame_, "l_shoulder_pan_link", now, stamped);
      tf::transformStampedTFToMsg(stamped, ts);
      float angle = getJointPosition("l_upper_arm_roll_joint", robot_state);
      ts.transform.rotation = msg::createQuaternionMsg( tf::Quaternion( tf::Vector3(1,0,0), angle) );
      ps = msg::createPoseStampedMsg(ts);
      ps.header.stamp = ros::Time(0);
      server_.insert(makePostureMarker( "l_posture_control", ps, 0.55, false, false),
                 boost::bind( &PR2MarkerControl::updatePosture, this, _1, 1 ));
  }
  else
  {
    server_.erase("l_posture_control");
  }

  if(control_state_.head_on_ && control_state_.init_head_goal_)
  {
    control_state_.init_head_goal_ = false;
    //geometry_msgs::PoseStamped ps = msg::createPoseStampedMsg( head_goal_pose_, "base_link", now);
    head_goal_pose_.header.stamp = ros::Time(0);
    //ps.header.stamp = ros::Time(0);
    server_.insert(makeHeadGoalMarker( "head_point_goal", head_goal_pose_, 0.1),
                   boost::bind( &PR2MarkerControl::updateHeadGoal, this, _1, 0 ));
  }
  if(!control_state_.head_on_)
  {
    server_.erase("head_point_goal");
  }

  if (interface_number_ == 0)
  {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.x = -0.35;
    ps.pose.position.z = 1.1;
    ps.pose.orientation.w = 1;
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time(0);
    server_.insert(makeElevatorMarker( "torso_control", ps, 0.25, false),
                   boost::bind( &PR2MarkerControl::updateTorso, this, _1 ));
    menu_torso_.apply(server_, "torso_control");
  }

  if(control_state_.base_on_ && interface_number_ == 0) // base control (!)
  {
    geometry_msgs::PoseStamped ps;
    ps.pose.orientation.w = 1;
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time(0);
    server_.insert(makeBaseMarker( "base_control", ps, 0.75, false),
                   boost::bind( &PR2MarkerControl::updateBase, this, _1 ));
  }
  else
  {
    server_.erase("base_control");
  }

  // The giant ball around the robot when it is driving somewhere.
  if( base_client_.hasGoal() )
  {
    geometry_msgs::PoseStamped ps;
    ps.pose.position.z = 1.0;
    ps.pose.orientation.w = 1;
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time(0);

    std_msgs::ColorRGBA color = object_manipulator::msg::createColorMsg(1.0, 0.8, 0.8, 0.2);
    server_.insert(makeButtonSphere( "move_base_e_stop", ps, 2.0, false, false, color ));
    server_.setCallback("move_base_e_stop", boost::bind( &PR2MarkerControl::cancelBaseMovement, this),
                          visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN );
  
    // TODO this should absolutely be read from the robot_description parameter
    std::vector< std::string > mesh_paths, mesh_frames;
    mesh_paths.push_back("package://pr2_description/meshes/base_v0/base.dae");
    mesh_frames.push_back("base_link");
    mesh_paths.push_back("package://pr2_description/meshes/torso_v0/torso_lift.dae");
    mesh_frames.push_back("torso_lift_link");
    mesh_paths.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_pan.dae");
    mesh_frames.push_back("r_shoulder_pan_link");
    mesh_paths.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_pan.dae");
    mesh_frames.push_back("l_shoulder_pan_link");
    mesh_paths.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae");
    mesh_frames.push_back("r_shoulder_lift_link");
    mesh_paths.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_lift.dae");
    mesh_frames.push_back("l_shoulder_lift_link");
    mesh_paths.push_back("package://pr2_description/meshes/upper_arm_v0/upper_arm.dae");
    mesh_frames.push_back("r_upper_arm_link");
    mesh_paths.push_back("package://pr2_description/meshes/upper_arm_v0/upper_arm.dae");
    mesh_frames.push_back("l_upper_arm_link");
    mesh_paths.push_back("package://pr2_description/meshes/forearm_v0/forearm.dae");
    mesh_frames.push_back("r_forearm_link");
    mesh_paths.push_back("package://pr2_description/meshes/forearm_v0/forearm.dae");
    mesh_frames.push_back("l_forearm_link");

    // Right Gripper
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/gripper_palm.dae");
    mesh_frames.push_back("r_gripper_palm_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_r.stl");
    mesh_frames.push_back("r_gripper_r_finger_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_l.stl");
    mesh_frames.push_back("r_gripper_l_finger_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_r.stl");
    mesh_frames.push_back("r_gripper_r_finger_tip_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_l.stl");
    mesh_frames.push_back("r_gripper_l_finger_tip_link");


    // Left Gripper
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/gripper_palm.dae");
    mesh_frames.push_back("l_gripper_palm_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_r.stl");
    mesh_frames.push_back("l_gripper_r_finger_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/upper_finger_l.stl");
    mesh_frames.push_back("l_gripper_l_finger_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_r.stl");
    mesh_frames.push_back("l_gripper_r_finger_tip_link");
    mesh_paths.push_back("package://pr2_description/meshes/gripper_v0/finger_tip_l.stl");
    mesh_frames.push_back("l_gripper_l_finger_tip_link");

    // Head
    mesh_paths.push_back("package://pr2_description/meshes/head_v0/head_pan.dae");
    mesh_frames.push_back("head_pan_link");
    mesh_paths.push_back("package://pr2_description/meshes/head_v0/head_tilt.dae");
    mesh_frames.push_back("head_tilt_link");


    if(mesh_paths.size() != mesh_frames.size()) ROS_ERROR("The number of mesh paths and mash frame_ids is not equal!");

    std::vector< geometry_msgs::PoseStamped > mesh_poses;
    for(size_t i = 0; i < mesh_paths.size(); i++)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time(0);
        ps.header.frame_id = mesh_frames[i];
        ps.pose.orientation.w = 1;
        tfl_.waitForTransform("base_link", ps.header.frame_id, ps.header.stamp,ros::Duration(3.0));
        tfl_.transformPose("base_link", ps, ps);
        mesh_poses.push_back(ps);
    }

    server_.insert(makePosedMultiMeshMarker( "move_base_goal", base_goal_pose_, mesh_poses, mesh_paths, 1.0, true));

  }
  else
  {
    server_.erase("move_base_e_stop");
    server_.erase("move_base_goal");
  }

}

//! Re-initialize only the mesh markers
void PR2MarkerControl::initMeshMarkers()
{
  std::vector<visualization_msgs::InteractiveMarker> mesh_markers;
  addMeshMarkersFromRobotModel(mesh_markers);

  for ( unsigned i=0; i<mesh_markers.size(); i++)
  {
    server_.insert(mesh_markers[i]);
  }

  ros::Time now = ros::Time(0);

  // For making the interactive marker meshes slightly bigger than the default robot_model meshes
  double scale_factor = 1.02;
  double mesh_offset  = -0.002;
  //double palm_scale_factor = 1.15;
  //double palm_mesh_offset  = -0.017;
  geometry_msgs::Quaternion q_identity;
  geometry_msgs::Quaternion q_rotateX180;
  q_rotateX180.w = 0;
  q_rotateX180.x = 1;

  // All interactive markers will be set to time(0), so they update automatically in rviz.
  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time(0);

  menu_head_.apply(server_, "head_tilt_link");

  if(use_right_arm_)
  {
    server_.setCallback("r_upper_arm_link", boost::bind( &PR2MarkerControl::upperArmButtonCB, this, _1, 0),
                        visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK );
    menu_arms_.apply(server_, "r_upper_arm_link");

    server_.setCallback("r_gripper_palm_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("r_gripper_l_finger_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("r_gripper_l_finger_tip_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("r_gripper_r_finger_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("r_gripper_r_finger_tip_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );

    menu_gripper_close_.apply(server_, "r_gripper_palm_link");
    menu_gripper_close_.apply(server_, "r_gripper_l_finger_link");
    menu_gripper_close_.apply(server_, "r_gripper_l_finger_tip_link");
    menu_gripper_close_.apply(server_, "r_gripper_r_finger_link");
    menu_gripper_close_.apply(server_, "r_gripper_r_finger_tip_link");
  }

  if(use_left_arm_)
  {
    server_.setCallback("l_upper_arm_link", boost::bind( &PR2MarkerControl::upperArmButtonCB, this, _1, 1),
                        visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK );
    menu_arms_.apply(server_, "l_upper_arm_link");

    server_.setCallback("l_gripper_palm_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("l_gripper_l_finger_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("l_gripper_l_finger_tip_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("l_gripper_r_finger_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );
    server_.setCallback("l_gripper_r_finger_tip_link", boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "toggle") );

    menu_gripper_close_.apply(server_, "l_gripper_palm_link");
    menu_gripper_close_.apply(server_, "l_gripper_l_finger_link");
    menu_gripper_close_.apply(server_, "l_gripper_l_finger_tip_link");
    menu_gripper_close_.apply(server_, "l_gripper_r_finger_link");
    menu_gripper_close_.apply(server_, "l_gripper_r_finger_tip_link");
  }

  if (interface_number_ == 0) // base link not allowed in studies
  {
    server_.setCallback("base_link", boost::bind( &PR2MarkerControl::baseButtonCB, this, _1),
                        visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK );
    menu_base_.apply(server_, "base_link");
  }

  if(control_state_.projector_on_)
  {
    server_.insert(makeProjectorMarker( "projector_control", ps, 1.0),
                   boost::bind( &PR2MarkerControl::projectorMenuCB, this, _1 ));
  }
  else
  {
    server_.erase("projector_control");
  }

  // The "reset" box over the PR2's head.
  ps = msg::createPoseStampedMsg(msg::createPointMsg(0,0,0.85), msg::createQuaternionMsg(0,0,0,1),
                                 "torso_lift_link", ros::Time(0));
  server_.insert(makeButtonBox( "PR2 Marker Control Reset", ps, 0.05, false, true),
                 boost::bind( &PR2MarkerControl::initAllMarkers, this, true ),
                 visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
}


void PR2MarkerControl::switchToCartesian()
{
  ROS_DEBUG("Switching to cartesian control.");

  // start out with posture control disabled
  std::vector<double> arm_angles(7);
  for( size_t i = 0; i < arm_angles.size(); i++) arm_angles[i] = 9999;

  mechanism_.get_robot_state_client_.client(ros::Duration(100.0));
  if(use_left_arm_){
      try {
        mechanism_.switchToCartesian("left_arm");
      }
      catch ( std::runtime_error &e )
      {
        ROS_ERROR("Caught exception while switching to cartesian controller: %s", e.what( ));
      }
      mechanism_.sendCartesianPostureCommand("left_arm", arm_angles);
  }

  if(use_right_arm_)
  {
      try {
        mechanism_.switchToCartesian("right_arm");
      }
      catch ( std::runtime_error &e ) {
        ROS_ERROR("Caught exception while switching to cartesian controller: %s", e.what( ));
      }
      mechanism_.sendCartesianPostureCommand("right_arm", arm_angles);
  }

  if(joint_handle_)       menu_arms_.setCheckState(joint_handle_, MenuHandler::UNCHECKED);
  if(jtranspose_handle_)  menu_arms_.setCheckState(jtranspose_handle_, MenuHandler::CHECKED);
}

void PR2MarkerControl::switchToJoint()
{
  ROS_DEBUG("Switching to joint control.");
  if(use_left_arm_)
  {
      try {
        mechanism_.switchToJoint("left_arm");
      }
      catch ( std::runtime_error &e ) {
        ROS_ERROR("Caught exception while switching to cartesian controller: %s", e.what( ));
      }
  }

  if(use_right_arm_)
  {
      try {
        mechanism_.switchToJoint("right_arm");
      }
      catch ( std::runtime_error &e ) {
        ROS_ERROR("Caught exception while switching to cartesian controller: %s", e.what( ));
      }
  }

  if(joint_handle_)       menu_arms_.setCheckState(joint_handle_, MenuHandler::CHECKED);
  if(jtranspose_handle_)  menu_arms_.setCheckState(jtranspose_handle_, MenuHandler::UNCHECKED);
}

bool PR2MarkerControl::checkStateValidity(std::string arm_name)
{
  // Hack to avoid using this for now
  if(!use_state_validator_) return true;

  if (interface_number_ != 0) return true;
  arm_navigation_msgs::GetStateValidity::Request request;
  request.group_name = arm_name;
  arm_navigation_msgs::GetStateValidity::Response response;
  try
  {
    if (!check_state_validity_client_.client().call(request, response))
    {
      ROS_ERROR("Call to get state validity failed");
      return false;
    }
    if (response.error_code.val != response.error_code.SUCCESS) return false;
    return true;
  }
  catch (object_manipulator::ServiceNotFoundException &ex)
  {
    ROS_ERROR("Get state validity service not found");
    return false;
  }
}

void PR2MarkerControl::updateHeadGoal( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int arm_id )
{
  ros::Time now = ros::Time(0);

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( feedback->marker_name << " was clicked on." );
      break;
    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM(    "Marker " << feedback->marker_name
                       << " control " << feedback->control_name
                       << " menu_entry_id " << feedback->menu_entry_id);

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      geometry_msgs::PointStamped ps;
      ps.point = feedback-> pose.position;
      ps.header.frame_id = feedback->header.frame_id;
      ps.header.stamp = now;
      head_goal_pose_.pose = feedback->pose;
      head_goal_pose_.header = feedback->header;
      mechanism_.pointHeadAction(ps, head_pointing_frame_, false);
      break;
  }
}

void PR2MarkerControl::targetPointMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  control_state_.head_on_ ^= true;
  control_state_.init_head_goal_ = true;

  if(control_state_.head_on_)
    menu_head_.setCheckState(head_target_handle_, MenuHandler::CHECKED);
  else
    menu_head_.setCheckState(head_target_handle_, MenuHandler::UNCHECKED);

  initControlMarkers();
}

void PR2MarkerControl::projectorMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(control_state_.projector_on_)
  {
    ROS_INFO("Trying to turn projector OFF");
    int ok = system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node \"{'projector_mode':1, 'narrow_stereo_trig_mode':2}\" ");
    ROS_INFO("Done!");
    if(ok < 0)
      ROS_WARN("Dynamic reconfigure for setting trigger mode OFF failed");
    else
    {
      control_state_.projector_on_ = false;
      menu_head_.setCheckState(projector_handle_, MenuHandler::UNCHECKED);
    }
  }
  else
  {
    ROS_INFO("Trying to turn projector ON");
    int ok = system("rosrun dynamic_reconfigure dynparam set camera_synchronizer_node \"{'projector_mode':3, 'narrow_stereo_trig_mode':5}\" ");
    ROS_INFO("Done!");
    if(ok < 0)
      ROS_WARN("Dynamic reconfigure for setting trigger mode ON failed");
    else
    {
      control_state_.projector_on_ = true;
      menu_head_.setCheckState(projector_handle_, MenuHandler::CHECKED);
    }
  }
  //menu_head_.reApply(server_);
  initMeshMarkers();
}

void PR2MarkerControl::gripperToggleModeCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  //MenuHandler::CheckState state;
  //if(!menu_arms_.getCheckState(handle, state))  return;

  if ( handle == gripper_view_facing_handle_ )
  {
    control_state_.r_gripper_.view_facing_ = true;
    control_state_.l_gripper_.view_facing_ = true;
    assert(menu_grippers_.setCheckState(gripper_view_facing_handle_, MenuHandler::CHECKED));
    assert(menu_grippers_.setCheckState(gripper_6dof_handle_, MenuHandler::UNCHECKED));
  }
  if ( handle == gripper_6dof_handle_ )
  {
    control_state_.r_gripper_.view_facing_ = false;
    control_state_.l_gripper_.view_facing_ = false;
    assert(menu_grippers_.setCheckState(gripper_view_facing_handle_, MenuHandler::UNCHECKED));
    assert(menu_grippers_.setCheckState(gripper_6dof_handle_, MenuHandler::CHECKED));
  }

  initControlMarkers();
}

void PR2MarkerControl::gripperToggleFixedCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  control_state_.r_gripper_.torso_frame_ ^= true;
  control_state_.l_gripper_.torso_frame_ ^= true;

  // TODO right now I'm assuming these will always be in the same state together...
  if(control_state_.r_gripper_.torso_frame_)
    menu_grippers_.setCheckState(gripper_fixed_control_handle_, MenuHandler::CHECKED);
  else
    menu_grippers_.setCheckState(gripper_fixed_control_handle_, MenuHandler::UNCHECKED);

  initControlMarkers();
}

void PR2MarkerControl::dualGripperToggleFixedCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  control_state_.dual_grippers_.torso_frame_ ^= true;

  if(control_state_.dual_grippers_.torso_frame_)
    menu_dual_grippers_.setCheckState(dual_gripper_fixed_control_handle_, MenuHandler::CHECKED);
  else
    menu_dual_grippers_.setCheckState(dual_gripper_fixed_control_handle_, MenuHandler::UNCHECKED);

  initControlMarkers();
}

void PR2MarkerControl::gripperToggleControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  control_state_.r_gripper_.edit_control_ ^= true;
  control_state_.l_gripper_.edit_control_ ^= true;

  // TODO right now I'm assuming these will always be in the same state together...
  if(control_state_.r_gripper_.edit_control_)
    menu_grippers_.setCheckState(gripper_edit_control_handle_, MenuHandler::CHECKED);
  else
    menu_grippers_.setCheckState(gripper_edit_control_handle_, MenuHandler::UNCHECKED);

  initControlMarkers();
}

void PR2MarkerControl::dualGripperToggleControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  control_state_.dual_grippers_.edit_control_ ^= true;

  if(control_state_.dual_grippers_.edit_control_)
    menu_dual_grippers_.setCheckState(dual_gripper_edit_control_handle_, MenuHandler::CHECKED);
  else
    menu_dual_grippers_.setCheckState(dual_gripper_edit_control_handle_, MenuHandler::UNCHECKED);

  ROS_INFO("toggling dual gripper edit control frame, current state is %d", control_state_.dual_grippers_.edit_control_);

  initControlMarkers();
}

void PR2MarkerControl::gripperResetControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  int arm_id = -1;
  if(!feedback->marker_name.compare("r_gripper_control"))
    arm_id = 0;
  else if(!feedback->marker_name.compare("l_gripper_control"))
    arm_id = 1;
  else
  {
    ROS_ERROR("Marker name [%s] not recognized...", feedback->marker_name.c_str());
    return;
  }
  pose_offset_[arm_id].setOrigin(tf::Vector3(0.1, 0, 0));
  pose_offset_[arm_id].setRotation(tf::Quaternion(0, 0, 0, 1));
}

void PR2MarkerControl::dualGripperResetControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  dual_pose_offset_.setOrigin(tf::Vector3(0, 0, 0));
  dual_pose_offset_.setRotation(tf::Quaternion(0, 0, 0, 1));
  initControlMarkers();
}

void PR2MarkerControl::startDualGrippers( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, bool active )
{
  control_state_.dual_grippers_.on_ = active;

  // right and left gripper transforms to base, and right and left transforms to middle frame
  tf::Transform b_T_rw, b_T_lw, b_T_rt, b_T_lt;

  if(active)
  {
    //disable edit control mode in both grippers, in case they were on when we switched
    control_state_.r_gripper_.edit_control_ = false;
    control_state_.l_gripper_.edit_control_ = false;
    menu_grippers_.setCheckState(gripper_edit_control_handle_, MenuHandler::UNCHECKED);

    geometry_msgs::PoseStamped right_tool, left_tool;
    geometry_msgs::PoseStamped right_wrist = mechanism_.getGripperPose("right_arm", feedback->header.frame_id);
    geometry_msgs::PoseStamped left_wrist  = mechanism_.getGripperPose("left_arm", feedback->header.frame_id);
    right_tool = wristToTool(right_wrist);
    left_tool  = wristToTool(left_wrist);
    tf::poseMsgToTF(right_wrist.pose, b_T_rw);
    tf::poseMsgToTF(left_wrist.pose, b_T_lw);
    tf::poseMsgToTF(right_tool.pose, b_T_rt);
    tf::poseMsgToTF(left_tool.pose, b_T_lt);

    // compute midpoint frame using tool frames!
    tf::Vector3 midpoint = b_T_rt.getOrigin() + 0.5*(b_T_lt.getOrigin() - b_T_rt.getOrigin());
    // TODO this orientation is in the wrong frame!
    // Even if we set the correct torso frame initially, I suppose we need to continue
    // updating it if the robot drives around...
    tf::Transform b_T_m = tf::Transform(tf::Quaternion(0,0,0,1), midpoint);

    //compute gripper wrist offsets using wrist frames!
    dual_gripper_offsets_[0] = b_T_m.inverse()*b_T_rw;
    dual_gripper_offsets_[1] = b_T_m.inverse()*b_T_lw;

    //control frame offset from default dual gripper frame
    dual_pose_offset_ = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
    control_state_.dual_grippers_.edit_control_ = false;
    menu_dual_grippers_.setCheckState(dual_gripper_edit_control_handle_, MenuHandler::UNCHECKED);
    control_state_.dual_grippers_.torso_frame_ = true;
    menu_dual_grippers_.setCheckState(dual_gripper_fixed_control_handle_, MenuHandler::CHECKED);

    dual_grippers_frame_ = msg::createPoseStampedMsg(b_T_m, feedback->header.frame_id, ros::Time(0)); // Time(0) prevents disappearing markers bug.
  }

  initControlMarkers();
}

void PR2MarkerControl::commandGripperPose(const geometry_msgs::PoseStamped &ps, int arm_id, bool use_offset)
{
  std::string arm = "right_arm";
  if(arm_id == 1) arm = "left_arm";
  const char *arm_name = arm.c_str();

  tf::Transform control_pose, command_pose;
  tf::poseMsgToTF(ps.pose, control_pose);
  if(use_offset)
    command_pose = control_pose * pose_offset_[arm_id].inverse();
  else
    command_pose = control_pose;
  double dummy_var = 0;
  geometry_msgs::PoseStamped desired_pose = mechanism_.clipDesiredPose(arm_name,
                                                                       msg::createPoseStampedMsg(command_pose, ps.header.frame_id, ros::Time::now()),
                                                                       cartesian_clip_distance_*update_period_,
                                                                       cartesian_clip_angle_*update_period_,
                                                                       dummy_var);

  ROS_DEBUG_STREAM("sending gripper command in frame " << desired_pose.header.frame_id << "(" 
		  << desired_pose.pose.position.x << ", " 
		  << desired_pose.pose.position.y << ", " 
		  << desired_pose.pose.position.z << ")" );

  try {
    mechanism_.sendCartesianPoseCommand( arm_name, desired_pose);
  }
  catch ( std::runtime_error &e )
  {
    ROS_ERROR("Caught exception while sending pose command: %s", e.what( ));
  }
}


void PR2MarkerControl::updateGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int arm_id )
{
  ros::Time now = ros::Time(0);
  std::string arm = "right_arm";
  if(arm_id == 1) arm = "left_arm";
  const char *arm_name = arm.c_str();

  switch ( feedback->event_type )
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      try {
        mechanism_.sendCartesianPoseCommand( arm_name, mechanism_.getGripperPose(arm_name, feedback->header.frame_id));
      }
      catch ( std::runtime_error &e )
      {
        ROS_ERROR("Caught exception while sending pose command: %s", e.what( ));
      }
      break;

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    // TODO still assuming l and r will always be in sync... this is probably BAD...
    if(control_state_.r_gripper_.edit_control_ )
    {
      geometry_msgs::PoseStamped ps = object_manipulator::msg::createPoseStampedMsg(feedback->pose,
                                                                                    feedback->header.frame_id,
                                                                                    ros::Time(0));
      if(!strcmp(arm_name, "right_arm"))
        tfl_.transformPose("r_wrist_roll_link", ps, ps );
      if(!strcmp(arm_name, "left_arm"))
        tfl_.transformPose("l_wrist_roll_link", ps, ps );

      tf::poseMsgToTF(ps.pose, pose_offset_[arm_id]);
    }
    else
    {
      //geometry_msgs::PoseStamped ps = object_manipulator::msg::createPoseStampedMsg(
      //                                              feedback->pose, feedback->header.frame_id, ros::Time(0));
      //tfl_.transformPose("base_link", ps, ps );
//      tf::Transform control_pose;
//      tf::poseMsgToTF(feedback->pose, control_pose);
//      tf::Transform command_pose = control_pose * pose_offset_[arm_id].inverse();

      ROS_DEBUG_STREAM("got updated gripper pose in frame " << feedback->header.frame_id);

      geometry_msgs::PoseStamped ps = object_manipulator::msg::createPoseStampedMsg(
                                                    feedback->pose, feedback->header.frame_id, feedback->header.stamp);
      commandGripperPose(ps, arm_id, true);
    }
    break;
  }
}

void PR2MarkerControl::updateDualGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ros::Time now = ros::Time(0);
  if(!control_state_.dual_grippers_.on_) return;

  //float half_width = 0.15;  // TODO this should be controllable in some way!

  switch ( feedback->event_type )
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      try {
        mechanism_.sendCartesianPoseCommand( "right_arm", mechanism_.getGripperPose("right_arm", feedback->header.frame_id));
        mechanism_.sendCartesianPoseCommand( "left_arm",  mechanism_.getGripperPose("left_arm",  feedback->header.frame_id));
      }
      catch ( std::runtime_error &e )
      {
        ROS_ERROR("Caught exception while sending pose command: %s", e.what( ));
      }
      break;

  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    // TODO still assuming l and r will always be in sync... this is probably BAD...
//    if(control_state_.r_gripper_.edit_control_ )
//    {
//      geometry_msgs::PoseStamped ps = object_manipulator::msg::createPoseStampedMsg(feedback->pose,
//                                                                                    feedback->header.frame_id,
//                                                                                    ros::Time(0));
//      if(!strcmp(arm_name, "right_arm"))
//        tfl_.transformPose("r_wrist_roll_link", ps, ps );
//      if(!strcmp(arm_name, "left_arm"))
//        tfl_.transformPose("l_wrist_roll_link", ps, ps );
//
//      tf::poseMsgToTF(ps.pose, pose_offset_[arm_id]);
//    }
//    else
    {
      if(control_state_.dual_grippers_.edit_control_ )
      {
        geometry_msgs::PoseStamped ps = object_manipulator::msg::createPoseStampedMsg(feedback->pose,
                                                                                      feedback->header.frame_id,
                                                                                      ros::Time(0));
        tfl_.transformPose(dual_grippers_frame_.header.frame_id, ps, ps ); 
        tf::Transform feedback_trans;
        tf::poseMsgToTF(ps.pose, feedback_trans);
        tf::Transform dual_pose_offset_tf;
        tf::Transform dual_grippers_frame_tf;
        tf::poseMsgToTF(dual_grippers_frame_.pose, dual_grippers_frame_tf);
        dual_pose_offset_ = feedback_trans.inverse() * dual_grippers_frame_tf;
      }
      else
      {
        tf::Transform command_pose;

        tf::poseMsgToTF(feedback->pose, command_pose);
        //tf::Transform command_pose = control_pose * pose_offset_[arm_id].inverse();

        double dummy_var = 0;
        tf::Transform offset_command_pose_tf = command_pose * dual_pose_offset_;
        geometry_msgs::Pose offset_command_pose;
        tf::poseTFToMsg(offset_command_pose_tf, offset_command_pose);
        geometry_msgs::PoseStamped offset_command_pose_stamped = msg::createPoseStampedMsg(offset_command_pose, feedback->header.frame_id, now);
        geometry_msgs::PoseStamped desired_pose =
          mechanism_.clipDesiredPose(dual_grippers_frame_,
                                     offset_command_pose_stamped,
                                     0.3*cartesian_clip_distance_*update_period_,
                                     0.3*cartesian_clip_angle_*update_period_,
                                     dummy_var);

        // compute left and right gripper poses
        tf::Transform b_T_m, b_T_r, b_T_l;
        tf::poseMsgToTF(desired_pose.pose, b_T_m);
        b_T_r = b_T_m * dual_gripper_offsets_[0];
        b_T_l = b_T_m * dual_gripper_offsets_[1];
        geometry_msgs::PoseStamped right_pose = msg::createPoseStampedMsg(b_T_r, feedback->header.frame_id, ros::Time(0));
        geometry_msgs::PoseStamped left_pose =  msg::createPoseStampedMsg(b_T_l, feedback->header.frame_id, ros::Time(0));

        //store new dual_grippers_frame_
        dual_grippers_frame_ = desired_pose;

        // send the commands
        try {
          mechanism_.sendCartesianPoseCommand( "right_arm", right_pose);
          mechanism_.sendCartesianPoseCommand( "left_arm",  left_pose);
        }
        catch ( std::runtime_error &e )
        {
          ROS_ERROR("Caught exception while sending pose command: %s", e.what( ));
        }
      }
    }
    break;
  }
}



//! Turns the gripper controls on and off or toggles them
void PR2MarkerControl::gripperButtonCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string action)
{
  if (interface_number_ != 0 && interface_number_ != 1) return;

  std::string arm_name = getArmNameFromMarkerName(feedback->marker_name);

  if(arm_name == "right_arm")
  {
    if(action == "toggle") control_state_.r_gripper_.on_ ^= true;
    else if(action == "turn on") control_state_.r_gripper_.on_ = true;
    else control_state_.r_gripper_.on_ = false;
  }
  if(arm_name == "left_arm")
  {
    if(action == "toggle") control_state_.l_gripper_.on_ ^= true;
    else if(action == "turn on") control_state_.l_gripper_.on_ = true;
    else control_state_.l_gripper_.on_ = false;
  }

  if ( control_state_.r_gripper_.on_ || control_state_.l_gripper_.on_ ||
       control_state_.posture_r_ || control_state_.posture_l_ )
  {
    switchToCartesian();
  }
  else
  {
    switchToJoint();
  }
  initAllMarkers();
}

void PR2MarkerControl::upperArmButtonCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int id)
{
  if (interface_number_ > 2) return;

  if(id == 0)
    control_state_.posture_r_ ^= true;
  if(id == 1)
    control_state_.posture_l_ ^= true;

  //always switch to Cartesian, so that joint trajectories are stopped in a hurry (e-stop)
  switchToCartesian();

  /*
  if ( control_state_.r_gripper_.on_ || control_state_.l_gripper_.on_ ||
       control_state_.posture_r_ || control_state_.posture_l_ )
  {
    switchToCartesian();
  }
  else
  {
    switchToJoint();
    }*/
  initAllMarkers();
}

void PR2MarkerControl::updatePosture( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int arm_id )
{
  std::string arm_name = "right_arm";
  if(arm_id == 1) arm_name = "left_arm";

  std::vector<double> arm_angles(7);
  //mechanism_.getArmAngles(arm_name, arm_angles);

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
      // This value (9999) disables the posture control for the joint.
      for( size_t i = 0; i < arm_angles.size(); i++) arm_angles[i] = 9999;
      mechanism_.sendCartesianPostureCommand(arm_name, arm_angles);
      break;
    }
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      tf::Quaternion q;
      tf::quaternionMsgToTF(feedback->pose.orientation, q);
      float angle = q.getAngle()*q.getAxis().dot(tf::Vector3(1,0,0));

      for( size_t i = 0; i < arm_angles.size(); i++) arm_angles[i] = 9999;
      arm_angles[2] = angle;
      mechanism_.sendCartesianPostureCommand(arm_name, arm_angles);
      break;
    }
  }
}

/*
void PR2MarkerControl::controllerSelectMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;

  if(!menu_arms_.getCheckState(handle, state))  return;
  if ( handle == joint_handle_ )
  {
    switchToJoint();
    control_state_.r_gripper_.on_ = false;
    control_state_.l_gripper_.on_ = false;
  }
  if ( handle == jtranspose_handle_ )
  {
    switchToCartesian();
  }

  initAllMarkers();
  }*/

void PR2MarkerControl::updateTorso( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch ( feedback->event_type )
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    if(!feedback->control_name.compare("up"))
      torso_client_.top();
    else if(!feedback->control_name.compare("down"))
      torso_client_.bottom();
    else
      ROS_ERROR("Marker control unrecognized, this is an error in the client implementation!");
    break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
      arm_navigation_msgs::RobotState robot_state;
      mechanism_.getRobotState(robot_state);
      torso_client_.moveTo( getJointPosition("torso_lift_joint", robot_state) );
      break;
    }
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    break;
  }
}


void PR2MarkerControl::updateBase( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "base_link";

  tf::Vector3 linear  = tf::Vector3(0,0,0);
  tf::Vector3 angular = tf::Vector3(0,0,0);

  switch ( feedback->event_type )
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
  {

    tf::Quaternion q;
    tf::quaternionMsgToTF(feedback->pose.orientation, q);
    tf::Matrix3x3 mat(q);

    if(!feedback->control_name.compare("forward"))  linear = tf::Vector3( 1, 0, 0);
    if(!feedback->control_name.compare("back"))     linear = tf::Vector3(-1, 0, 0);
    if(!feedback->control_name.compare("left"))     linear = tf::Vector3( 0, 1, 0);
    if(!feedback->control_name.compare("right"))    linear = tf::Vector3( 0,-1, 0);
    if(!feedback->control_name.compare("rotate right"))  angular = tf::Vector3(0, 0,-1);
    if(!feedback->control_name.compare("rotate left"))   angular = tf::Vector3(0, 0, 1);

    linear = 0.3*(mat*linear);
    angular = 0.5*angular;

    tf::vector3TFToMsg(linear, ts.twist.linear);
    tf::vector3TFToMsg(angular, ts.twist.angular);

    base_client_.applyTwist(ts);
    break;
  }
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    base_client_.applyTwist(ts);
    break;
  case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
    break;
  }
}

void PR2MarkerControl::torsoMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if(!feedback->control_name.compare("up"))
    torso_client_.top();
  else if(!feedback->control_name.compare("down"))
    torso_client_.bottom();
  else
    ROS_ERROR("Marker control unrecognized, this is an error in the client implementation!");
}


void PR2MarkerControl::gripperClosureCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const float &command)
{
  std::string arm_name = getArmNameFromMarkerName(feedback->marker_name);
  double open = gripper_client_.getGripperOpenGap(arm_name);
  double closed = gripper_client_.getGripperClosedGap(arm_name);
  double value = closed + (open - closed) * command;

  if( arm_name != "" )
  {
    gripper_client_.commandGripperValue(arm_name, value);
  }
  else
    ROS_ERROR("Marker name [%s] not handled!", feedback->marker_name.c_str());

  //if opening the gripper, reset attached objects for that arm
  if (value > closed + (open - closed)/2.)
    mechanism_.detachAllObjectsFromGripper(arm_name);
}

void PR2MarkerControl::requestGripperPose(  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                                            int arm_id)
{
  std::string arm_name;
  if(!feedback->marker_name.compare("r_gripper_control")){
    arm_name = "right_arm";
  }
  else if(!feedback->marker_name.compare("l_gripper_control"))
  {
    arm_name = "left_arm";
  }
  else
  {
    ROS_ERROR("Marker name [%s] not recognized...", feedback->marker_name.c_str());
    return;
  }

  pr2_object_manipulation_msgs::GetGripperPoseGoal goal;
  goal.arm_name = arm_name;
  goal.gripper_opening = 0.08;
  if(arm_name == "right_arm") goal.gripper_pose = mechanism_.getGripperPose(arm_name, "torso_lift_link");
  if(arm_name == "left_arm") goal.gripper_pose = mechanism_.getGripperPose(arm_name, "torso_lift_link");
  gripper_pose_client_.client().sendGoal( goal,
                                          boost::bind(&PR2MarkerControl::processGripperPoseResult, this, _1, _2, arm_name),
                                          actionlib::SimpleActionClient<pr2_object_manipulation_msgs::GetGripperPoseAction>::SimpleActiveCallback(),
                                          //actionlib::SimpleActionClient<pr2_object_manipulation_msgs::GetGripperPoseAction>::SimpleFeedbackCallback());//,
                                          boost::bind(&PR2MarkerControl::processGripperPoseFeedback, this, _1, arm_name) );
  
  control_state_.r_gripper_.on_ = false;
  control_state_.l_gripper_.on_ = false;
  initControlMarkers();
}

void PR2MarkerControl::processGripperPoseFeedback(  const pr2_object_manipulation_msgs::GetGripperPoseFeedbackConstPtr &result,
                                                    const std::string &arm_name)
{
  int arm_id = 0;
  if(arm_name == "left_arm") arm_id = 1;

  commandGripperPose(result->gripper_pose, arm_id, false);

}

void PR2MarkerControl::processGripperPoseResult(  const actionlib::SimpleClientGoalState& state,
                                                  const pr2_object_manipulation_msgs::GetGripperPoseResultConstPtr &result,
                                                  const std::string &arm_name)
{
  ROS_ERROR("We should be setting the state back to 'controls active' here.");
  control_state_.r_gripper_.on_ = true;
  control_state_.l_gripper_.on_ = true;
  initControlMarkers();
}

void PR2MarkerControl::moveArm( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const std::string &position,
              bool planner)
{
  std::string arm_name;
  if( feedback->marker_name == "r_upper_arm_link" || feedback->marker_name == "r_gripper_palm_link" )
    arm_name = "right_arm";
  else if( feedback->marker_name == "l_upper_arm_link" || feedback->marker_name == "l_gripper_palm_link" )
    arm_name = "left_arm";
  else
    ROS_WARN("Marker name [%s] not handled!", feedback->marker_name.c_str());

  std::string move_type = (planner)?("with planner"):("open-loop");
  ROS_INFO("moving %s to %s %s", arm_name.c_str(), position.c_str(), move_type.c_str());
  sys_thread_.reset( new boost::thread( boost::bind(
              &PR2MarkerControl::moveArmThread, this, arm_name, position, true, planner ) ) );
}


void PR2MarkerControl::moveArmThread(std::string arm_name, std::string position, bool collision, bool planner)
{
  if (interface_number_ != 0) gripper_client_.commandGripperValue(arm_name, gripper_client_.getGripperOpenGap(arm_name));
  try
  {
    ROS_DEBUG("Attempting arm motion");
    arm_navigation_msgs::OrderedCollisionOperations ord;
    if (!collision)
    {
      arm_navigation_msgs::CollisionOperation coll;
      coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
      ord.collision_operations.push_back(coll);
    }
    std::vector<arm_navigation_msgs::LinkPadding> pad;
    if (collision)
    {
      pad = object_manipulator::concat( object_manipulator::MechanismInterface::gripperPadding("left_arm", 0.0),
                                        object_manipulator::MechanismInterface::gripperPadding("right_arm", 0.0) );
    }
    if (planner)
    {
      //ompl will crash if move_arm for the right and left arms try to plan at the same time, so lock to make sure we don't send two goals at once
      if(planner_lock_.try_lock())
      {
        ROS_INFO("Locking to attemptMoveArmToGoal for %s", arm_name.c_str());

        //clear any current move arm goals if there are any running before trying to issue our own
        mechanism_.clearMoveArmGoals();

        //trying move_arm twice, resetting map if stuck, timeout of 30 seconds
        mechanism_.attemptMoveArmToGoal(arm_name,
                                        object_manipulator::armConfigurations().position(arm_name, position), ord, pad, 2, true, 30.);
        planner_lock_.unlock();
        ROS_INFO("moveArmThread for %s released lock", arm_name.c_str());
      }
      //if we're already asking move_arm to do something, try to cancel goals and then try again
      else
      {
        ROS_INFO("marker control: Failed to get lock when trying to move %s!  Clearing move arm goals twice", arm_name.c_str());

        //clear goals twice, since we always try twice
        mechanism_.clearMoveArmGoals();
        ros::Duration(0.25).sleep();
        ROS_INFO("marker control: clearing move arm goals a second time");
        mechanism_.clearMoveArmGoals();
        ROS_INFO("marker control: done clearing move arm goals before moving %s", arm_name.c_str());
        if(planner_lock_.try_lock())
        {
          ROS_INFO("Got lock this time, locking to attemptMoveArmToGoal for %s", arm_name.c_str());
          //trying twice, resetting map if stuck, timeout of 30 seconds
          mechanism_.attemptMoveArmToGoal(arm_name,
                                          object_manipulator::armConfigurations().position(arm_name, position), ord, pad, 2, true, 30.);
          planner_lock_.unlock();
          ROS_INFO("released lock after moving %s", arm_name.c_str());
        }
        else
        {
          ROS_INFO("Failed to get lock again when trying to move %s, giving up", arm_name.c_str());
        }
      }
    }
    else
    {
      mechanism_.attemptTrajectory(arm_name, object_manipulator::armConfigurations().trajectory(arm_name, position),
                                   false, 2.0);
    }
  }
  catch (object_manipulator::ServiceNotFoundException &ex)
  {
    ROS_ERROR("a needed service or action server was not found");
  }
  catch (object_manipulator::MoveArmStuckException &ex)
  {
    ROS_ERROR("arm is stuck in a colliding position");
    planner_lock_.unlock();
  }
  catch (object_manipulator::MissingParamException &ex)
  {
    ROS_ERROR("parameters missing; is manipulation pipeline running?");
  }
  catch (object_manipulator::MechanismException &ex)
  {
    ROS_ERROR("Error: %s", ex.what());
    planner_lock_.unlock();
  }
  catch (...)
  {
    ROS_ERROR("an unknown error has occured, please call helpdesk");
    planner_lock_.unlock();
  }

  // Interface 1 (cartesian teleop) should always pop up the control rings.
  if(interface_number_ == 1)
  {
    switchToCartesian();
    control_state_.r_gripper_.on_ = true;
    //control_state_.posture_r_ = true;
    initAllMarkers();
  }
  // Auto-refresh / reset collision map is useful when running with novice users.
  if(interface_number_ != 0){
    snapshot_client_.refresh( nh_.resolveName("snapshot_input_topic", true) );
    std_srvs::Empty srv;
    if (!collider_node_reset_srv_.client().call(srv)) ROS_ERROR("failed to reset collision map!");
  }
}


void PR2MarkerControl::requestNavGoal(const bool &collision_aware)
{
  //ROS_WARN("Starting requestNavGoal");
  // First call the 'get pose' action so that the user is distracted.
  pr2_object_manipulation_msgs::GetNavPoseGoal goal;
  goal.max_distance = collision_aware ? (-1.0): (max_direct_nav_radius_);
  base_pose_client_.client().sendGoal( goal, boost::bind(&PR2MarkerControl::processNavGoal, this, _1, _2, collision_aware));

  if( using_3d_nav_ ) 
  {
    ROS_INFO("Using 3D navigation, not changing global or local planners");
    return;
  }

  // While the user is using the requested action, configure the planner in the background. 
  std::string reconfigure = "rosrun dynamic_reconfigure dynparam set ";
  std::string move_base_config, costmap_config;
  
  if( collision_aware )
  {
    ROS_INFO("Activating 'global' planner.");
    // Don't just use "restore_defaults: true" because it restores whatever was on the param server when the node started."

    // We could dynamically set the footprint of the robot based on the arm pose during these calls!
    move_base_config = reconfigure + move_base_node_name_ + " "
                       + "\"{'base_global_planner': 'navfn/NavfnROS', "
                       + "   'base_local_planner' : 'dwa_local_planner/DWAPlannerROS'}\" ";
    costmap_config = reconfigure + move_base_node_name_ + "/local_costmap \"{ "
                     + "'max_obstacle_height' : 2.0, 'inflation_radius' : 0.55 }\" ";
  }
  else
  {
    ROS_INFO("Activating 'local' planner.");
    move_base_config = reconfigure + move_base_node_name_ +" "
//                       + "\"{'base_global_planner': 'navfn/NavfnROS', "
                       + "\"{'base_global_planner': 'pr2_navigation_controllers/GoalPasser', "
                       + "   'base_local_planner' : 'pr2_navigation_controllers/PoseFollower'}\" ";
    
    costmap_config = reconfigure + move_base_node_name_ + "/local_costmap \"{ "
                     + "'max_obstacle_height' : 0.36 , 'inflation_radius': 0.01 }\" ";
  }

  //ROS_WARN("Setting planners...");
  int ok = system(move_base_config.c_str());
  if(ok < 0)
    ROS_ERROR("Dynamic reconfigure for move_base_node FAILED!");
  //ROS_WARN("Setting costmap configuration...");
  ok = system(costmap_config.c_str());
  if(ok < 0)
    ROS_ERROR("Dynamic reconfigure for move_base_node/local_costmap FAILED!");  
  ROS_INFO("Done!");
  //ROS_WARN("Exiting requestNavGoal");
}

void PR2MarkerControl::processNavGoal(  const actionlib::SimpleClientGoalState& state,
                                        const pr2_object_manipulation_msgs::GetNavPoseResultConstPtr &result,
                                        const bool &collision_aware )
{
  if(state.state_ == state.SUCCEEDED)
  {
    ROS_DEBUG("Got a valid base pose.");
    base_goal_pose_ = result->pose;
    sendLastNavGoal();
  }
  else
  {
    ROS_WARN("Get Base Pose Action did not succeed; state = %d", (int)state.state_);
  }
}

void PR2MarkerControl::requestBasePoseEstimate()
{
  pr2_object_manipulation_msgs::GetNavPoseGoal goal;
  goal.max_distance = -1.0;
  base_pose_client_.client().sendGoal( goal, boost::bind(&PR2MarkerControl::processBasePoseEstimate, this, _1, _2));
}


void PR2MarkerControl::processBasePoseEstimate( const actionlib::SimpleClientGoalState& state,
                                                const pr2_object_manipulation_msgs::GetNavPoseResultConstPtr &result )
{
  if(state.state_ == state.SUCCEEDED)
  {
    ROS_DEBUG("Got a valid estimate for base pose.");
    geometry_msgs::PoseWithCovarianceStamped ps;
    ps.pose.pose = result->pose.pose;
    ps.pose.pose.position.z = 0;  //enforce 0 ground-plane position
    ps.header = result->pose.header;

    //Covariance values taken from what Rviz publishes... no idea if these are optimal though.
    ps.pose.covariance.at(6*0 + 0) = 0.25;
    ps.pose.covariance.at(6*1 + 1) = 0.25;
    ps.pose.covariance.at(6*5 + 5) = 0.06853891945200942;

    base_client_.sendPoseEstimate(ps);
  }
  else
  {
    ROS_WARN("Get Base Pose Action did not succeed; state = %d", (int)state.state_);
  }
}

void PR2MarkerControl::sendLastNavGoal()
{
    if( (ros::Time::now() - base_goal_pose_.header.stamp).toSec() < 60.0 )
    {
        ROS_INFO("Sending last base goal pose");
        base_client_.sendNavGoal( base_goal_pose_ );
    }
    else
    {
        base_goal_pose_ = geometry_msgs::PoseStamped();
        base_goal_pose_.header.frame_id = "base_link";
        base_goal_pose_.pose.orientation.w = 1; 
        ROS_INFO("Last goal was from too long ago; sending identity base goal pose");
        base_client_.sendNavGoal( base_goal_pose_ );
    }
}

void PR2MarkerControl::clearLocalCostmap()
{
    ROS_INFO("Clearing the navigation costmap...");
    base_client_.clearLocalCostmap( );
}

void PR2MarkerControl::inHandObjectRightCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  ROS_INFO("Got an in-hand object cloud for the right hand!");
  object_cloud_right_.updateCloud(*cloud, "in_hand_object_right");
}

void PR2MarkerControl::inHandObjectLeftCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  ROS_INFO("Got an in-hand object cloud for the left hand!");
  object_cloud_left_.updateCloud(*cloud, "in_hand_object_left");
}

void PR2MarkerControl::initMenus()
{
  MenuHandler::EntryHandle handle;

  // - - - - - - - - - - Head Menu - - - - - - - - - - //

  menu_head_.insert("Take Snapshot", boost::bind( &PR2MarkerControl::snapshotCB, this ) );

  if (interface_number_ == 0)
  {
    head_target_handle_ = menu_head_.insert( "Target Point", boost::bind( &PR2MarkerControl::targetPointMenuCB,
                                                                          this, _1 ) );
    menu_head_.setCheckState(head_target_handle_, MenuHandler::UNCHECKED);

    projector_handle_ = menu_head_.insert("Projector", boost::bind( &PR2MarkerControl::projectorMenuCB,
                                                                    this, _1 ) );
    menu_head_.setCheckState(projector_handle_, MenuHandler::UNCHECKED);

    menu_head_.insert( "Move Head To Center", boost::bind( &PR2MarkerControl::centerHeadCB,
							   this ) );
  }


  // - - - - - - - - - - Arm Menu - - - - - - - - - - //
  if (interface_number_ == 0)
  {
    tuck_handle_ = menu_arms_.insert( "Tuck..." );
    menu_arms_.insert(tuck_handle_, "Untuck Both Arms",         boost::bind( &PR2MarkerControl::tuckArmsCB, this, false, false));
    menu_arms_.insert(tuck_handle_, "Untuck left, tuck right",  boost::bind( &PR2MarkerControl::tuckArmsCB, this, false, true ));
    menu_arms_.insert(tuck_handle_, "Tuck left, untuck right",  boost::bind( &PR2MarkerControl::tuckArmsCB, this, true, false ));
    menu_arms_.insert(tuck_handle_, "Tuck Both Arms",           boost::bind( &PR2MarkerControl::tuckArmsCB, this, true, true  ));
  }

  if (interface_number_ == 0)
  {
    handle = menu_arms_.insert( "Safe move to ... " );
    menu_arms_.insert( handle, "above", boost::bind( &PR2MarkerControl::moveArm, this, _1, "above", true  ) );
    menu_arms_.insert( handle, "side",  boost::bind( &PR2MarkerControl::moveArm, this, _1, "side", true  ) );
    menu_arms_.insert( handle, "front", boost::bind( &PR2MarkerControl::moveArm, this, _1, "front", true ) );

    handle = menu_arms_.insert( "Forced move to ... " );
    menu_arms_.insert( handle, "above", boost::bind( &PR2MarkerControl::moveArm, this, _1, "above", false  ) );
    menu_arms_.insert( handle, "side",  boost::bind( &PR2MarkerControl::moveArm, this, _1, "side", false  ) );
    menu_arms_.insert( handle, "front", boost::bind( &PR2MarkerControl::moveArm, this, _1, "front", false ) );

    handle = menu_arms_.insert( "Gripper commands" );
    menu_arms_.insert( handle, "Activate Cartesian control",
		       boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "turn on") );
    menu_arms_.insert( handle, "Turn off Cartesian control",
		       boost::bind( &PR2MarkerControl::gripperButtonCB, this, _1, "turn off") );
    menu_arms_.insert( handle, "Open Gripper",
		       boost::bind( &PR2MarkerControl::gripperClosureCB, this, _1, 1 ) );
    menu_arms_.insert( handle, "Close Gripper",
		       boost::bind( &PR2MarkerControl::gripperClosureCB, this, _1, 0 ) );

    handle = menu_arms_.insert( "Plug..." );
    menu_arms_.insert( handle, "Plug in", boost::bind( &PR2MarkerControl::plugsCB, this, PLUGS_PLUGIN) );
    menu_arms_.insert( handle, "Unplug", boost::bind( &PR2MarkerControl::plugsCB, this, PLUGS_UNPLUG) );
    menu_arms_.insert( handle, "Cancel", boost::bind( &PR2MarkerControl::plugsCB, this, PLUGS_CANCEL) );
  }
  else
  {
    menu_arms_.insert( "Drop object and reset position",
                       boost::bind( &PR2MarkerControl::moveArm, this, _1, "side", true  ) );
  }

  // - - - - - - - - - - Gripper Control Menu - - - - - - - - - - //

  gripper_fixed_control_handle_ = menu_grippers_.insert( "Torso-Fixed Control",
                                                  boost::bind( &PR2MarkerControl::gripperToggleFixedCB, this, _1 ) );
  menu_grippers_.setCheckState(gripper_fixed_control_handle_, MenuHandler::CHECKED);

  if (interface_number_ == 0)
  {
    handle = menu_grippers_.insert( "Marker Mode" );

    gripper_6dof_handle_ = menu_grippers_.insert( handle, "6 DOF",
                                                  boost::bind( &PR2MarkerControl::gripperToggleModeCB, this, _1 ) );
    menu_grippers_.setCheckState(gripper_6dof_handle_, MenuHandler::CHECKED);

    gripper_view_facing_handle_ = menu_grippers_.insert( handle, "View Facing",
                                                  boost::bind( &PR2MarkerControl::gripperToggleModeCB, this, _1 ) );
    menu_grippers_.setCheckState(gripper_view_facing_handle_, MenuHandler::UNCHECKED);

    gripper_edit_control_handle_ = menu_grippers_.insert( "Edit Control Frame",
                                               boost::bind( &PR2MarkerControl::gripperToggleControlCB, this, _1 ) );
    menu_grippers_.setCheckState(gripper_edit_control_handle_, MenuHandler::UNCHECKED);

    gripper_reset_control_handle_ = menu_grippers_.insert( "Reset Control Frame",
                                                boost::bind( &PR2MarkerControl::gripperResetControlCB, this, _1 ) );
    menu_grippers_.insert( "Dual Grippers", boost::bind( &PR2MarkerControl::startDualGrippers, this, _1, true ) );
    menu_grippers_.insert( "Collision-Constraint", boost::bind( &PR2MarkerControl::requestGripperPose, this, _1, 0 ) );
  }

  // - - - - - - - - - - Dual-Gripper Control Menu - - - - - - - - - - //

  if (interface_number_ == 0)
  {
    menu_dual_grippers_.insert( "Exit", boost::bind( &PR2MarkerControl::startDualGrippers, this, _1, false ) );
    dual_gripper_fixed_control_handle_ = menu_dual_grippers_.insert( "Torso-Fixed Control", 
                                              boost::bind( &PR2MarkerControl::dualGripperToggleFixedCB, this, _1 ) );
    menu_dual_grippers_.setCheckState(dual_gripper_edit_control_handle_, MenuHandler::CHECKED);
    dual_gripper_edit_control_handle_ = menu_dual_grippers_.insert( "Edit Control Frame", 
                                              boost::bind( &PR2MarkerControl::dualGripperToggleControlCB, this, _1 ) );
    menu_dual_grippers_.setCheckState(dual_gripper_edit_control_handle_, MenuHandler::UNCHECKED);
    dual_gripper_reset_control_handle_ = menu_dual_grippers_.insert( "Reset Control Frame", 
                                              boost::bind( &PR2MarkerControl::dualGripperResetControlCB, this, _1 ) );
  }

  // - - - - - - - - - - Gripper Closure Menu - - - - - - - - - - //

  if (interface_number_ < 3)
  {
    menu_gripper_close_.insert( "Open Gripper",  boost::bind( &PR2MarkerControl::gripperClosureCB, this, _1, 1 ) );
    menu_gripper_close_.insert( "Close Gripper", boost::bind( &PR2MarkerControl::gripperClosureCB, this, _1, 0 ) );
  }
  else
  {
    menu_gripper_close_.insert( "Drop object and reset position",
        boost::bind( &PR2MarkerControl::moveArm, this, _1, "side", true  ) );
  }

  // - - - - - - - - - - Torso Menu - - - - - - - - - - //
  menu_torso_.insert( "All the way", boost::bind( &PR2MarkerControl::torsoMenuCB, this, _1 ) );

  // - - - - - - - - - - Base Menu - - - - - - - - - - //
  menu_base_.insert( "Navigate to...",      boost::bind( &PR2MarkerControl::requestNavGoal, this, true ) );
  menu_base_.insert( "Directly Move to...", boost::bind( &PR2MarkerControl::requestNavGoal, this, false ) );
  menu_base_.insert( "Resume last Goal", boost::bind( &PR2MarkerControl::sendLastNavGoal, this ) );
  menu_base_.insert( "Clear costmaps", boost::bind( &PR2MarkerControl::clearLocalCostmap, this ) );
  menu_base_.insert( "Set Pose on Map", boost::bind( &PR2MarkerControl::requestBasePoseEstimate, this ) );
}


// Graveyard
