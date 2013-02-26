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

#include <ros/ros.h>
#include <math.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <object_manipulator/tools/mechanism_interface.h>
#include <object_manipulator/tools/msg_helpers.h>

#include <boost/thread.hpp>
#include <tf/transform_listener.h>
#include <boost/thread/recursive_mutex.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <pr2_object_manipulation_msgs/GetNavPoseAction.h>
#include <pr2_object_manipulation_msgs/CameraFocus.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include "eigen_conversions/eigen_msg.h"

#include "eigen3/Eigen/Geometry"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"

#include <interactive_marker_helpers/interactive_marker_helpers.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>


using namespace object_manipulator;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace pr2_object_manipulation_msgs;
using namespace im_helpers;

////////////////////////////////////////////////////////////////////////////////////


/** \brief An action for getting a full robot pose using interactive markers.
  */
class BasePoseAction
{
protected:

  // ****************** class members *********************

  geometry_msgs::PoseStamped base_pose_;
  geometry_msgs::PoseStamped control_offset_;

  bool active_;

  int interface_number_;
  int task_number_;

  bool testing_planned_grasp_;
  int tested_grasp_index_;
  bool testing_current_grasp_;
  double max_direct_move_radius_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_seed_pose_;
  ros::Subscriber sub_seed_point_;

  ros::Timer fast_update_timer_;
  ros::Timer slow_update_timer_;
  InteractiveMarkerServer server_;

  ros::Publisher pub_focus_;

  MenuHandler menu_controls_;
  MenuHandler::EntryHandle accept_handle_;
  MenuHandler::EntryHandle cancel_handle_;
  MenuHandler::EntryHandle focus_handle_;

  tf::TransformListener tfl_;

  object_manipulator::MechanismInterface mechanism_;

  std::string get_pose_name_;
  actionlib::SimpleActionServer<pr2_object_manipulation_msgs::GetNavPoseAction> get_pose_server_;

public:

  BasePoseAction() :
      active_(false),
      max_direct_move_radius_(10.0),
      nh_("/"),
      pnh_("~"),
      server_(ros::names::resolve("pr2_nav_marker_control"), "nav_action", false),
      tfl_(nh_),
      get_pose_name_(ros::this_node::getName()),
      get_pose_server_(nh_, get_pose_name_, false)
  {
    ROS_INFO( "pr2_interactive_base_pose_action is starting up." );

    ros::Duration(1.0).sleep();

    nh_.param<int>("interactive_grasping/interface_number", interface_number_, 0);
    if (!interface_number_) ROS_WARN("No interface number specified for grasping study; using default configuration");
    else ROS_INFO("Using interface number %d for grasping study", interface_number_);
    nh_.param<int>("interactive_grasping/task_number", task_number_, 0);
    if (!task_number_) ROS_WARN("No task number specified for grasping study; using default configuration");
    else ROS_INFO("Using task number %d for grasping study", task_number_);

    //nh_.param<double>("max_direct_move_radius", max_direct_move_radius_, 0.4);

    // Dummy box to avoid empty_server bug...
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "base_link";
    server_.insert(makeButtonBox( "test_box", ps, 0.01, false, false));

    pub_focus_ = nh_.advertise<pr2_object_manipulation_msgs::CameraFocus>("camera_focus", 1);

    fast_update_timer_ =  nh_.createTimer(ros::Duration(0.05), boost::bind( &BasePoseAction::fast_update, this ) );

    sub_seed_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/cloud_click_point", 1, boost::bind(&BasePoseAction::setSeedPoseCallback, this, _1));
    sub_seed_point_ = nh_.subscribe<geometry_msgs::PointStamped>("/rviz/navigate_to", 1, boost::bind(&BasePoseAction::setSeedPointCallback, this, _1));

    // Initialization must happen at the end!
    initMenus();

    //register the goal and feeback callbacks
    get_pose_server_.registerGoalCallback(    boost::bind(&BasePoseAction::goalCB, this));
    get_pose_server_.registerPreemptCallback( boost::bind(&BasePoseAction::preemptCB, this));

    get_pose_server_.start();
  }

  ~BasePoseAction()
  {
  }

  void setSeedPoseCallback(const geometry_msgs::PoseStampedConstPtr &seedPose)
  {
    if(!active_) return;
    ROS_DEBUG("Setting seed.");
    base_pose_ = *seedPose;
    base_pose_.pose.orientation = geometry_msgs::Quaternion();

    updateSeed();
  }
  
  void setSeedPointCallback(const geometry_msgs::PointStampedConstPtr &seedPoint)
  {
    if(!active_) return;
    ROS_DEBUG("Setting seed point.");
    base_pose_.pose.position= seedPoint->point;
    base_pose_.pose.orientation = geometry_msgs::Quaternion();

    updateSeed();
  }

  void updateSeed()
  {
    initMarkers();
  }

  

  //! Remove the markers.
  void setIdle(){
    active_ = false;
    server_.clear();
  }


  //! Callback to accept a new action goal.
  void goalCB()
  {
    active_ = true;
    ROS_INFO("pr2_interactive_nav_action_called!");
    pr2_object_manipulation_msgs::GetNavPoseGoal goal = *get_pose_server_.acceptNewGoal();

    if (goal.starting_pose.pose.orientation.x == 0 &&
        goal.starting_pose.pose.orientation.y == 0 &&
        goal.starting_pose.pose.orientation.z == 0 &&
        goal.starting_pose.pose.orientation.w == 0 )
    {
      ROS_DEBUG("Empty pose passed in; using default");
      base_pose_ = getDefaultPose();
    }
    else
    {
      base_pose_ = goal.starting_pose;
    }

    if (get_pose_server_.isPreemptRequested())
    {
      preemptCB();
      return;
    }
    if(goal.max_distance > 0)
        max_direct_move_radius_ = goal.max_distance;
    else
        max_direct_move_radius_ = 1E6;

    initMarkers();
  }


  //! Callback to allow this action to get preempted by backend.
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", get_pose_name_.c_str());
    //test_pose_client_.client().cancelAllGoals();
    get_pose_server_.setPreempted();
    setIdle();
  }

  /** \brief Update the pose of certain markers.
    */
  void updatePoses()
  {
    server_.setPose("meshes", base_pose_.pose, base_pose_.header);
  }

  geometry_msgs::PoseStamped getDefaultPose()
  {
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time(0);
    ps.header.frame_id = "base_link";
    ps.pose.orientation.w = 1;
    return ps;
  }


// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

  /** \brief Re-initializes all markers.
    */
  void initMarkers()
  {
    initMeshMarkers();
    initControlMarkers();
  }

  void initMeshMarkers()
  {
  }

  void initControlMarkers()
  {
    geometry_msgs::PoseStamped ps = base_pose_;
    ps.header.stamp = ros::Time(0);

    //ROS_ERROR("Populating meshes...");

    // TODO this should absolutely be read from some parameter
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


    if(mesh_paths.size() != mesh_frames.size()) ROS_ERROR("The number of mesh paths and mesh frame_ids is not equal!");

    std::vector< geometry_msgs::PoseStamped > mesh_poses;
    for(size_t i = 0; i < mesh_paths.size(); i++)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.stamp = ros::Time(0);
        ps.header.frame_id = mesh_frames[i];
        ps.pose.orientation.w = 1;
        tfl_.waitForTransform("base_link", ps.header.frame_id, ps.header.stamp,ros::Duration(3.0));
        try
        {
          tfl_.transformPose("base_link", ps, ps);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR("pr2_interactive_nav_action: failed to transform from %s to base_link!  Skipping mesh part", ps.header.frame_id.c_str());
          continue;
        }
        mesh_poses.push_back(ps);
    }

    server_.insert(makePosedMultiMeshMarker( "pose_controls", ps, mesh_poses, mesh_paths, 1.03, false),
                   boost::bind( &BasePoseAction::poseControlCB, this, _1));
    menu_controls_.apply(server_, "pose_controls");
  }


protected:


  //! ROS spin update callback
  void fast_update()
  {
    //updatePoses();
    server_.applyChanges();
  }

  void slow_update()
  {
  }


  //! Return with the gripper pose if the pose is valid, otherwise do nothing
  void acceptCB()
  {
    setIdle();
    pr2_object_manipulation_msgs::GetNavPoseResult result;
    base_pose_.header.stamp = ros::Time::now();
    result.pose = base_pose_;
    get_pose_server_.setSucceeded(result);
  }

  //! Cancel this action call
  void cancelCB()
  {
    get_pose_server_.setAborted();
    setIdle();
  }

  //! Sends a request for the 3D camera to focus on the controls
  void focusCB()
  {
    pr2_object_manipulation_msgs::CameraFocus msg;
    msg.focal_point.point = base_pose_.pose.position;
    msg.focal_point.header = base_pose_.header;
    pub_focus_.publish(msg);
  }

  //! Called when the gripper is clicked; each call cycles through gripper opening values.
  void poseMeshCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
  }

  geometry_msgs::PoseStamped clipToRadius(const geometry_msgs::PoseStamped &input, const float &radius)
  {
      geometry_msgs::PoseStamped ps;
      try
      {
        tfl_.waitForTransform(input.header.frame_id, "base_link", input.header.stamp, ros::Duration(5.0));
        tfl_.transformPose("base_link", input, ps);
        tf::Vector3 pose_center;
        tf::pointMsgToTF(ps.pose.position, pose_center);
        float stored_z = pose_center.z();
        pose_center.setZ(0);
        float length = pose_center.length();
        if(length > radius)
        {
          pose_center = radius / length * pose_center;
          pose_center.setZ(stored_z);
          tf::pointTFToMsg(pose_center, ps.pose.position);
        }
        ps.header.stamp = ros::Time(0);
        if( tfl_.canTransform("map", ps.header.frame_id, ros::Time(0)))
        {
          tfl_.transformPose("map", ps, ps);
        }
        else if( tfl_.canTransform("odom_combined", ps.header.frame_id, ros::Time(0)))
        {
          tfl_.transformPose("odom_combined", ps, ps);
        }
        else if( tfl_.canTransform(input.header.frame_id, ps.header.frame_id, ros::Time(0)))
        {
          tfl_.transformPose(input.header.frame_id, ps, ps);
        }
        else
        {
          ROS_ERROR("TF could not transform from map, odom_combined, or %s to %s!!  clipToRadius failed\n", input.header.frame_id.c_str(), ps.header.frame_id.c_str());
        }
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("pr2_interactive_nav_action: failed to transform from %s to base_link!", input.header.frame_id.c_str());
      }
      return ps;
  }

  //! Callback for pose updates from the controls.
  void poseControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    ros::Time now = ros::Time(0);

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        base_pose_.pose = feedback->pose;
        base_pose_.header = feedback->header;
        ROS_DEBUG_STREAM("MOUSE_UP base_pose before clipping is\n" << base_pose_);
        base_pose_ = clipToRadius(base_pose_, max_direct_move_radius_);
        ROS_DEBUG_STREAM("MOUSE_UP base_pose after clipping is\n" << base_pose_);
        initMarkers();
        break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_DEBUG_STREAM("POSE_UPDATE in frame " << feedback->header.frame_id << std::endl << feedback->pose);
        base_pose_.pose = feedback->pose;
        base_pose_.header = feedback->header;
        updatePoses();
        break;
    }
  }

  //! Initialize the menus for all markers.
  void initMenus()
  {
    accept_handle_ = menu_controls_.insert("Accept", boost::bind( &BasePoseAction::acceptCB, this ) );
    cancel_handle_ = menu_controls_.insert("Cancel", boost::bind( &BasePoseAction::cancelCB, this ) );
    focus_handle_ = menu_controls_.insert("Focus camera", boost::bind( &BasePoseAction::focusCB, this ) );
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_interactive_pose_select_action");
  BasePoseAction base_pose_action;

  ros::Duration(1.0).sleep();

  ros::spin();
}
