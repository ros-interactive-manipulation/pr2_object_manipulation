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
#include <point_cloud_server/StoreCloudAction.h>
#include <object_manipulation_msgs/GraspPlanningAction.h>

#include <pr2_object_manipulation_msgs/TestGripperPoseAction.h>
#include <pr2_object_manipulation_msgs/GetGripperPoseAction.h>
#include <pr2_object_manipulation_msgs/CameraFocus.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include "eigen_conversions/eigen_msg.h"

#include "eigen3/Eigen/Geometry"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"

#include <household_objects_database_msgs/GetModelDescription.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include <interactive_marker_helpers/interactive_marker_helpers.h>

using namespace object_manipulator;
using namespace visualization_msgs;
using namespace interactive_markers;
using namespace pr2_object_manipulation_msgs;
using namespace im_helpers;

typedef pcl::PointXYZ PointT;

////////////////////////////////////////////////////////////////////////////////////


/** \brief An action for getting a gripper pose using interactive markers.
  */
class GripperPoseAction
{
protected:

  // ****************** class members *********************

  std::vector<geometry_msgs::PoseStamped> planner_poses_;
  std::vector<PoseState> planner_states_;
  std::vector<int> planner_grasp_types_;
  int planner_index_;
  geometry_msgs::PoseStamped button_marker_pose_;

  geometry_msgs::PoseStamped gripper_pose_;
  geometry_msgs::PoseStamped control_offset_;
  float gripper_opening_;
  float gripper_angle_;
  pcl::PointCloud<PointT>::Ptr object_cloud_;
  bool active_;
  bool object_model_;
  bool always_call_planner_;
  bool show_invalid_grasps_;
  bool always_find_alternatives_;
  bool gripper_opening_cycling_;

  int interface_number_;
  int task_number_;
  bool testing_planned_grasp_;
  int tested_grasp_index_;
  bool testing_current_grasp_;
  bool testing_alternatives_;

  double alternatives_search_dist_;
  double alternatives_search_dist_resolution_;
  double alternatives_search_angle_;
  double alternatives_search_angle_resolution_;
  double grasp_plan_region_len_x_;
  double grasp_plan_region_len_y_;
  double grasp_plan_region_len_z_;

  PoseState pose_state_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_seed_;
  ros::ServiceClient get_model_mesh_client_;
  ros::Timer spin_timer_;
  ros::Timer slow_sync_timer_;
  InteractiveMarkerServer server_;
  
  ros::Publisher pub_cloud_;
  ros::Publisher pub_focus_;

  MenuHandler menu_gripper_;
  MenuHandler::EntryHandle accept_handle_;
  MenuHandler::EntryHandle cancel_handle_;
  MenuHandler::EntryHandle focus_handle_;
  MenuHandler::EntryHandle alternatives_handle_;

  tf::TransformListener tfl_;

  object_manipulator::MechanismInterface mechanism_;

  object_manipulator::ActionWrapper<pr2_object_manipulation_msgs::TestGripperPoseAction> test_pose_client_;
  object_manipulator::ActionWrapper<object_manipulation_msgs::GraspPlanningAction> grasp_plan_client_;
  object_manipulator::ActionWrapper<point_cloud_server::StoreCloudAction> cloud_server_client_;

  std::string get_pose_name_;
  actionlib::SimpleActionServer<pr2_object_manipulation_msgs::GetGripperPoseAction> get_pose_server_;

public:

  GripperPoseAction() :
      planner_index_(0),
      gripper_angle_(0.541),
      object_cloud_(new pcl::PointCloud<PointT>()),
      active_(false),
      object_model_(false),
      pose_state_(UNTESTED),
      nh_("/"),
      pnh_("~"),
      server_("pr2_marker_control", "server 2", false),
      tfl_(nh_),
      test_pose_client_("test_gripper_pose", true),
      grasp_plan_client_("grasp_plan_action", true),
      cloud_server_client_("point_cloud_server_action", true),
      get_pose_name_(ros::this_node::getName()),
      get_pose_server_(nh_, get_pose_name_, false)
  {
    ROS_INFO( "pr2_interactive_gripper_pose_action is starting up." );

    ros::Duration(1.0).sleep();

    pnh_.param<bool>("always_call_planner", always_call_planner_, false);
    pnh_.param<bool>("always_find_alternatives", always_find_alternatives_, true);
    pnh_.param<bool>("show_invalid_grasps", show_invalid_grasps_, false);
    pnh_.param<bool>("gripper_opening_cycling", gripper_opening_cycling_, false);

    nh_.param<int>("interactive_grasping/interface_number", interface_number_, 0);
    if (!interface_number_) ROS_WARN("No interface number specified for grasping study; using default configuration");
    else ROS_INFO("Using interface number %d for grasping study", interface_number_);
    nh_.param<int>("interactive_grasping/task_number", task_number_, 0);
    if (!task_number_) ROS_WARN("No task number specified for grasping study; using default configuration");
    else ROS_INFO("Using task number %d for grasping study", task_number_);
    pnh_.param<double>("alternatives_search_dist", alternatives_search_dist_, .15);
    pnh_.param<double>("alternatives_search_dist_resolution", alternatives_search_dist_resolution_, .005);
    pnh_.param<double>("alternatives_search_angle", alternatives_search_angle_, M_PI/2);
    pnh_.param<double>("alternatives_search_angle_resolution", alternatives_search_angle_resolution_, M_PI/10);
    pnh_.param<double>("grasp_plan_region_len_x", grasp_plan_region_len_x_, 0.3);
    pnh_.param<double>("grasp_plan_region_len_y", grasp_plan_region_len_y_, 0.3);
    pnh_.param<double>("grasp_plan_region_len_z", grasp_plan_region_len_z_, 0.3);

    if(interface_number_ == 4) always_call_planner_ = true;
    else if(interface_number_ >= 1) always_call_planner_ = false;

    // Dummy box to avoid empty_server bug...
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "/base_link";
    server_.insert(makeButtonBox( "test_box", ps, 0.01, false, false));

    pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_debug", 1);
    pub_focus_ = nh_.advertise<pr2_object_manipulation_msgs::CameraFocus>("camera_focus", 1);

    get_model_mesh_client_ = nh_.serviceClient<household_objects_database_msgs::GetModelMesh>("objects_database_node/get_model_mesh", false);

    spin_timer_ =  nh_.createTimer(ros::Duration(0.05), boost::bind( &GripperPoseAction::spinOnce, this ) );

    sub_seed_ = nh_.subscribe<geometry_msgs::PoseStamped>("cloud_click_point", 1, boost::bind(&GripperPoseAction::setSeed, this, _1));
    
    // Initialization must happen at the end!
    initMenus();
    //initMarkers();
    
    //register the goal and feeback callbacks
    get_pose_server_.registerGoalCallback(    boost::bind(&GripperPoseAction::goalCB, this));
    get_pose_server_.registerPreemptCallback( boost::bind(&GripperPoseAction::preemptCB, this));

    get_pose_server_.start();
  }

  ~GripperPoseAction()
  {
  }

  bool transformGripperPose(const std::string frame_id = "/base_link")
  {
  /*  try{
      tfl_.waitForTransform(frame_id, gripper_pose_.header.frame_id, gripper_pose_.header.stamp, ros::Duration(3.0));
      tfl_.transformPose(frame_id, gripper_pose_, gripper_pose_);
      return true;
    }
    catch(...){
      ROS_ERROR("TF can't transform!");
      return false;
    }
*/
    return true;
  }

  void updateGripperOpening()
  {
    gripper_opening_ = gripper_angle_ * 0.1714;
  }

  void updateGripperAngle()
  {
    gripper_angle_ = gripper_opening_ * 5.834;
  }

  void setSeed(const geometry_msgs::PoseStampedConstPtr &seed)
  {
    if(!active_) return;
    ROS_DEBUG("Setting seed.");
    geometry_msgs::PoseStamped ps = *seed;
    ROS_DEBUG_STREAM("Input seed was \n" << ps);
    tf::Pose pose;
    tf::poseMsgToTF(ps.pose, pose);
    tf::Quaternion q = pose.getRotation();
    tf::Matrix3x3 rot(q);
    tf::Matrix3x3 perm(  0, 0, 1,
                       0, 1, 0,
                      -1, 0, 0);
    (rot*perm).getRotation(q);
    //tf::quaternionTFToMsg(q, ps.pose.orientation);
    pose.setRotation(q);

    if(object_model_) pose = pose*tf::Transform(tf::Quaternion(tf::Vector3(0,1,0), M_PI/2.0), tf::Vector3(0,0,0)).inverse();

    tf::poseTFToMsg(pose, ps.pose);
    //ps.header = seed->header;
    //ROS_INFO_STREAM("Processed seed before wrist offset was \n" << ps);

    gripper_pose_ = toWrist(ps);
    //ROS_INFO_STREAM("Processed seed was \n" << gripper_pose_);
    //transformGripperPose();
    //ROS_DEBUG_STREAM("But after transforming it is \n" << gripper_pose_);
    //ROS_INFO_STREAM("ansforming it is \n" << gripper_pose_);
    //transformGripperPose();

    pose_state_ = UNTESTED;
    eraseAllGraspMarkers();

    testing_planned_grasp_ = false;
    testing_alternatives_ = false;
    testing_current_grasp_ = true;
    if(always_call_planner_)
    {
      ROS_INFO("Always call planner is on");
      graspPlanCB();
    }
    else
    {
      initMarkers();
      testPose(gripper_pose_, gripper_opening_);
    }
    //focus the camera on the new pose
    focusCB();
  }

  //! Remove the markers.
  void setIdle(){
    active_ = false;
    server_.erase("ghosted_gripper");
    server_.erase("gripper_controls");
    server_.erase("object_cloud");
    eraseAllGraspMarkers();
    planner_states_.clear();
    planner_poses_.clear();  
  }


  //! Callback to accept a new action goal.
  void goalCB()
  {
    active_ = true;
    object_model_ = false;
    planner_index_ = 0;
    ROS_INFO("Ghosted gripper called");
    pr2_object_manipulation_msgs::GetGripperPoseGoal goal = *get_pose_server_.acceptNewGoal();
    object_cloud_.reset( new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    control_offset_.pose = object_manipulator::msg::createPoseMsg(tf::Pose(tf::Quaternion::getIdentity(), tf::Vector3(0.15,0,0)));

    //check if we want to call the planner, find alternatives, or show invalid grasps now
    if(interface_number_ == 0) 
    {
      pnh_.param<bool>("always_call_planner", always_call_planner_, false);
      pnh_.param<bool>("always_find_alternatives", always_find_alternatives_, true);
      pnh_.param<bool>("show_invalid_grasps", show_invalid_grasps_, true);
      pnh_.param<bool>("gripper_opening_cycling", gripper_opening_cycling_, false);
      ROS_INFO("always_call_planner: %d, always_find_alt: %d, show_invalid: %d, gripper_opening_cycle: %d", 
               (int)always_call_planner_, (int)always_find_alternatives_, (int)show_invalid_grasps_, (int)gripper_opening_cycling_);
    }

    if( !goal.object.cluster.points.empty() ||
        !goal.object.potential_models.empty() )
    {
      ROS_INFO("Goal object contains %d cluster and %d models.",
               !goal.object.cluster.points.empty(), (int)goal.object.potential_models.size() );
      // Something to draw...
      try
      {
        ROS_INFO("Converting to reference_frame...");
        mechanism_.convertGraspableObjectComponentsToFrame(goal.object, goal.object.reference_frame_id);

        bool use_cloud = goal.object.potential_models.empty();

        // Try to use object model, if there is one
        if(!use_cloud)
        {
          ROS_INFO("Goal contains object model; looking for model mesh...");
          // Connect to databse, get object mesh.
          arm_navigation_msgs::Shape mesh;
          if(!getModelMesh(goal.object.potential_models[0].model_id, mesh))
          {
            ROS_INFO("Unable to get database model, continuing with cluster.");
            use_cloud = true;
          }

          if(!use_cloud)
          {
            object_model_ = true;
            for(unsigned int i = 0; i < mesh.vertices.size(); i++)
            {
              PointT pt;
              pt.x = mesh.vertices[i].x;
              pt.y = mesh.vertices[i].y;
              pt.z = mesh.vertices[i].z;
              cloud->points.push_back(pt);
            }
            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = false;
            cloud->header.frame_id = goal.object.reference_frame_id;

            geometry_msgs::Pose &m = goal.object.potential_models[0].pose.pose;
            Eigen::Affine3f affine = Eigen::Translation3f(m.position.x,
                                                                m.position.y,
                                                                m.position.z) *
                                         Eigen::Quaternionf(m.orientation.w,
                                                            m.orientation.x,
                                                            m.orientation.y,
                                                            m.orientation.z);
            pcl::transformPointCloud(*cloud, *cloud, affine);

            tf::Transform T_o, T_g;
            tf::poseMsgToTF(goal.object.potential_models[0].pose.pose, T_o);
            tf::poseMsgToTF(goal.grasp.grasp_pose, T_g);
            tf::Transform T = T_g.inverse()*T_o;
            tf::poseTFToMsg(T, control_offset_.pose);

          }
        }

        if(use_cloud)
        {
          // Store point cloud

          sensor_msgs::PointCloud2 converted_cloud;
          sensor_msgs::convertPointCloudToPointCloud2 (goal.object.cluster, converted_cloud);

          pcl::fromROSMsg(converted_cloud, *cloud);
        }

        geometry_msgs::Pose &m = goal.grasp.grasp_pose;
        Eigen::Affine3f affine = Eigen::Translation3f(m.position.x,
                                                            m.position.y,
                                                            m.position.z) *
                                     Eigen::Quaternionf(m.orientation.w,
                                                        m.orientation.x,
                                                        m.orientation.y,
                                                        m.orientation.z);
        affine = affine.inverse();
        pcl::transformPointCloud(*cloud, *object_cloud_, affine);
      }
      catch(...){
        ROS_ERROR("%s: Error converting graspable object to reference frame id [%s]!",
                  get_pose_name_.c_str(), goal.object.reference_frame_id.c_str());
      }
    }
    if (goal.gripper_pose.pose.orientation.x == 0 &&
        goal.gripper_pose.pose.orientation.y == 0 &&
        goal.gripper_pose.pose.orientation.z == 0 &&
        goal.gripper_pose.pose.orientation.w == 0 )
    {
      ROS_INFO("Empty pose passed in; using default");
      gripper_pose_ = getDefaultPose(goal.arm_name);
      //ROS_DEBUG_STREAM("Default was \n" << gripper_pose_);
      //transformGripperPose();
      //ROS_DEBUG_STREAM("Default after transform is\n" << gripper_pose_);
      gripper_opening_ = 0.086;
      updateGripperAngle();
    }
    else
    {
      gripper_pose_ = goal.gripper_pose;
      //transformGripperPose();
      gripper_opening_ = goal.gripper_opening;
      updateGripperAngle();
    }
    if (get_pose_server_.isPreemptRequested())
    {
      if(test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::ACTIVE ||
         test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::PENDING )
      {
        test_pose_client_.client().cancelAllGoals();
      }
      get_pose_server_.setPreempted();
      setIdle();
      return;
    }

    pose_state_ = UNTESTED;
    initMarkers();
    pr2_object_manipulation_msgs::TestGripperPoseGoal test_goal;
    test_goal.gripper_poses.push_back(gripper_pose_);
    test_goal.gripper_openings.push_back(gripper_opening_);
    test_pose_client_.client().sendGoal( test_goal, boost::bind(&GripperPoseAction::testGripperResultCallback, this, _1, _2));
  }


  //! Callback to allow this action to get preempted by backend.
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", get_pose_name_.c_str());
    test_pose_client_.client().cancelAllGoals();
    grasp_plan_client_.client().cancelGoal();
    get_pose_server_.setPreempted();
    setIdle();
  }

  //! Translate the control pose to the wrist.
  geometry_msgs::PoseStamped toWrist(const geometry_msgs::PoseStamped &ps)
  {
    geometry_msgs::PoseStamped out;
    out.header = ps.header;
    tf::Transform T, P;
    tf::poseMsgToTF(ps.pose, P);
    tf::poseMsgToTF(control_offset_.pose, T);
    tf::poseTFToMsg( P*T.inverse(), out.pose);
    //out.pose = object_manipulator::msg::applyShift(ps.pose, tf::Vector3(-0.15,0,0));
    return out;
  }

  //! Translate to the control pose.
  geometry_msgs::PoseStamped fromWrist(const geometry_msgs::PoseStamped &ps)
  {
    geometry_msgs::PoseStamped out;
    out.header = ps.header;
    tf::Transform T, P;
    tf::poseMsgToTF(ps.pose, P);
    tf::poseMsgToTF(control_offset_.pose, T);
    tf::poseTFToMsg( P*T, out.pose);
    //out.pose = object_manipulator::msg::applyShift(ps.pose, tf::Vector3(-0.15,0,0));
    return out;
  }


  /** \brief Update the pose of certain markers.
    */
  void updatePoses()
  {
    server_.setPose("ghosted_gripper", gripper_pose_.pose, gripper_pose_.header);
    server_.setPose("object_cloud", gripper_pose_.pose, gripper_pose_.header);
  }

  geometry_msgs::PoseStamped getDefaultPose(std::string arm_name)
  {
    ros::Time now = ros::Time(0);
    tfl_.waitForTransform("r_gripper_tool_frame","/base_link", now, ros::Duration(2.0));
    tf::StampedTransform stamped;
    geometry_msgs::TransformStamped ts;
    geometry_msgs::PoseStamped ps;
    std::string arm_frame;
    if (arm_name == "right_arm") arm_frame="r_wrist_roll_link";
    else if (arm_name == "left_arm") arm_frame="l_wrist_roll_link";
    else 
    {
      arm_frame="r_wrist_roll_link";
      ROS_ERROR("Unknown arm name passed to ghosted gripper");
    }
    tfl_.lookupTransform("/base_link", arm_frame, now, stamped);
    tf::transformStampedTFToMsg(stamped, ts);
    ps = msg::createPoseStampedMsg(ts);
    return ps;
  }


// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

  /** \brief Re-initializes all markers.
    */
  void initMarkers()
  { 
    initGripperMarker();
    initObjectMarker();
    initGripperControl();
  }

  void initGraspMarkers()
  {
    for(unsigned int i = 0; i < planner_poses_.size(); i++)
    {
      // create arrow marker
      std::stringstream ss;
      ss << "grasp_" << i;
      server_.insert(makeGraspMarker(ss.str().c_str(), planner_poses_[i], 1.5, planner_states_[i]));
      server_.setCallback(ss.str(), boost::bind( &GripperPoseAction::selectGrasp, this, _1),
                          visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN );

      // TODO figure out how to delete stuff gracefully
    }
  }

  void eraseAllGraspMarkers()
  {
    for(unsigned int i = 0; i < planner_poses_.size(); i++)
    {
      // delete
      std::stringstream ss;
      ss << "grasp_" << i;
      server_.erase(ss.str().c_str());
    }
    server_.erase("grasp_toggle");
    planner_index_ = 0;
    planner_poses_.resize(0);
    planner_states_.resize(0);
    planner_grasp_types_.resize(0);

  }

  void selectNextGrasp()
  {
    selectGraspIndex( planner_index_+1 );
    /*
    int start_ind = planner_index_+1;
    for(unsigned int i = 0; i < planner_poses_.size(); i++)
    {
      if(start_ind + (int)i >= (int)planner_poses_.size() ) start_ind = -i;
      if(planner_states_[ start_ind + i ] == VALID || show_invalid_grasps_)
      {
        selectGraspIndex( start_ind + i );
        break;
      }
      }*/
  }

  void selectGraspIndex(int index)
  {
    planner_index_ = index;
    if( planner_index_ >= (int)planner_poses_.size() ) planner_index_ = 0;
    gripper_pose_ = planner_poses_[ planner_index_ ];
    pose_state_ = planner_states_[ planner_index_ ];
    ROS_INFO("Selecting grasp %d", planner_index_);
    initMarkers();

    if(pose_state_ == UNTESTED) 
    {
      testing_planned_grasp_ = false;
      testing_alternatives_ = false;
      tested_grasp_index_ = planner_index_;
      testing_current_grasp_ = true;
      testPose(gripper_pose_, gripper_opening_);
    }
  }

  void selectGrasp( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    int index = atoi(feedback->marker_name.substr(6).c_str());
    if( index >= (int)planner_poses_.size() )  return;
    selectGraspIndex(index);
  }

  void initButtonMarker()
  {
    button_marker_pose_.header.stamp = ros::Time(0);
    int num = 0;
    if(planner_poses_.size()) num = planner_index_ + 1;
    /*
    int total = 0;
    for(unsigned int i = 0; i < planner_poses_.size(); i++)
    {
      if(planner_states_[i] != VALID && !show_invalid_grasps_) continue;
      total++;
      if((int)i <= planner_index_) num++;
      }*/
    server_.insert(makeListControl("grasp_toggle", button_marker_pose_, num, planner_poses_.size(), 0.3));
    server_.setCallback("grasp_toggle", boost::bind( &GripperPoseAction::cycleGrasps, this, _1),
                        visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN );
  }

  void initObjectMarker()
  {
    if(object_cloud_->points.size())
    {
      server_.insert(makeCloudMarker( "object_cloud", gripper_pose_, 0.004, 
                                      object_manipulator::msg::createColorMsg(0.2, 0.8, 0.2,1.0) ));
    }
  }

  void initGripperMarker()
  {
    float r,g,b;
    switch(pose_state_)
    {
    case INVALID:
      r = 1.0; g = 0.2; b = 0.2;
      break;
    case VALID:
      r = 0.2; g = 1.0; b = 0.2;
      break;
    default:
      r = 0.5; g = 0.5; b = 0.5;
    }
    std_msgs::ColorRGBA color = object_manipulator::msg::createColorMsg(r,g,b,1.0);

    server_.insert(makeGripperMarker( "ghosted_gripper", gripper_pose_, 1.01, gripper_angle_, false, color),
                   boost::bind( &GripperPoseAction::gripperClickCB, this, _1));
    menu_gripper_.apply(server_, "ghosted_gripper");
  }

  void initGripperControl()
  {
    geometry_msgs::PoseStamped ps = fromWrist(gripper_pose_);
    ps.header.stamp = ros::Time(0);
    server_.insert(make6DofMarker( "gripper_controls", ps, 0.2, false, false),
                   boost::bind( &GripperPoseAction::updateGripper, this, _1));
    menu_gripper_.apply(server_, "gripper_controls");
  }


protected:


  //! ROS spin update callback
  void spinOnce()
  {
    //updatePoses();
    server_.applyChanges();
  }

  void slowSync()
  {
  }

  //! Callback that receives the result of a TestGripperPose action.
  void testGripperResultCallback(const actionlib::SimpleClientGoalState& state,
                                 const pr2_object_manipulation_msgs::TestGripperPoseResultConstPtr &result)
  {
    if (result->valid.empty())
    {
      ROS_ERROR("Test gripper pose returned with empty result list");
      return;
    }
    if(state.state_ == state.SUCCEEDED)
    {
      ROS_INFO("Test pose action returned with result %d", (int)result->valid[0]);

      if (testing_planned_grasp_)
      {
        if (!planner_states_.empty())
        {
          if(planner_states_.size() == result->valid.size())
          {
            int dim_index = 0;
            bool dim_grasp_found = false;
            if(testing_alternatives_) dim_index = planner_grasp_types_[0];
            std::vector<geometry_msgs::PoseStamped> planner_poses_temp;
            std::vector<PoseState> planner_states_temp;

            //fill in planner_states_ with whether the grasps were VALID or INVALID
            for (size_t i=0; i<result->valid.size(); i++)
            {
              PoseState pose_state = INVALID;

              //valid grasp
              if (result->valid[i])
              {
                pose_state = VALID;		    

                //only show the first feasible grasp in each direction if we're testing alternatives
                if (testing_alternatives_){
                  if(planner_grasp_types_[i] == dim_index)
                  {
                    //found the first good grasp along the first direction
                    if(!dim_grasp_found)
                    {
                      dim_grasp_found = true;
                    }

                    //ignoring all other grasps along this direction
                    else pose_state = UNTESTED;
                  }

                  //found the first good grasp along this direction
                  else
                  {
                    dim_index = planner_grasp_types_[i];
                    dim_grasp_found = true;
                  }
                }
              }

              //invalid grasp
              else
              {
                if(testing_alternatives_)
                {
                  //don't show invalid alternatives-grasps, even if show_invalid_grasps is true
                  pose_state = UNTESTED;

                  //moving onto the next direction to search, haven't found a good grasp yet
                  if(planner_grasp_types_[i] != dim_index){
                    dim_index = planner_grasp_types_[i];
                    dim_grasp_found = false;
                  }
                }
              }

              //grasp is valid or we're showing invalid grasps, add it to the list
              if(pose_state == UNTESTED || (pose_state == INVALID && !show_invalid_grasps_)) continue;
              planner_states_temp.push_back(pose_state);
              planner_poses_temp.push_back(planner_poses_[i]);
            }

            //erase all current grasp markers
            eraseAllGraspMarkers();

            //copy the temporary lists into the real ones            
            planner_states_ = planner_states_temp;
            planner_poses_ = planner_poses_temp;
            ROS_INFO("planner_states has %zd entries", planner_states_.size());

            //create the grasp markers
            initGraspMarkers();
          }
          else
          {
            ROS_ERROR("planner_states_ size did not match result->valid size!");
          }
        }
        else
        {
          ROS_ERROR("planner_states_ was empty!");
        }

        //create the button marker for switching grasps
        if(planner_poses_.size() > 1 || testing_alternatives_) initButtonMarker();
      }

      //just testing the one grasp
      if (testing_current_grasp_)
      {
        PoseState pose_state = INVALID;
        if (result->valid[0]) pose_state = VALID;
        pose_state_ = pose_state;
        initGripperMarker();

        //if we want to always find alternatives, do so if the grasp is invalid
        if(always_find_alternatives_ && pose_state == INVALID)
        {
          alternativesCB();
        }
      } 
    }
    else
    {
      ROS_WARN("Test pose action did not succeed; state = %d", (int)state.state_);
    }
  }

  //! Call the grasp planner to find grasps near the seed
  void graspPlanCB()
  {
    eraseAllGraspMarkers();

    point_cloud_server::StoreCloudGoal cloud_goal;
    cloud_goal.action = cloud_goal.GET;
    cloud_goal.name = "interactive_manipulation_snapshot";
    cloud_server_client_.client().sendGoal(cloud_goal);
    if(!cloud_server_client_.client().waitForResult(ros::Duration(3.0)))
    {
      ROS_WARN("Timed-out while waiting for cloud from server!");
      return;
    }

    object_manipulation_msgs::GraspPlanningGoal plan_goal;
    plan_goal.target.region.cloud = cloud_server_client_.client().getResult()->cloud;
    plan_goal.target.region.roi_box_pose = fromWrist(gripper_pose_);
    plan_goal.target.region.roi_box_dims = object_manipulator::msg::createVector3Msg(grasp_plan_region_len_x_, 
                                                                                     grasp_plan_region_len_y_, 
                                                                                     grasp_plan_region_len_z_);
    int cloud_size = plan_goal.target.region.cloud.width * plan_goal.target.region.cloud.height;
    plan_goal.target.reference_frame_id = gripper_pose_.header.frame_id;
    object_manipulation_msgs::Grasp seed;
    seed.grasp_pose = gripper_pose_.pose;
    plan_goal.grasps_to_evaluate.push_back(seed);

    ROS_DEBUG_STREAM("Requesting adjustment on cloud with " << cloud_size << " points, pose \n" << seed);
    //pub_cloud_.publish(plan_goal.target.region.cloud);
    grasp_plan_client_.client().sendGoal( plan_goal, boost::bind(&GripperPoseAction::graspPlanResultCB, this, _1, _2));

    button_marker_pose_ = gripper_pose_;
    button_marker_pose_.header.stamp = ros::Time(0);
    button_marker_pose_.pose.orientation = geometry_msgs::Quaternion();
    button_marker_pose_.pose.orientation.w = 1;
    button_marker_pose_.pose.position.z -= 0.2;
    //initButtonMarker();
    server_.erase("gripper_controls");
    server_.erase("ghosted_gripper");
  }

  //! Callback that receives the result of a TestGripperPose action.
  void graspPlanResultCB(const actionlib::SimpleClientGoalState& state,
                         const object_manipulation_msgs::GraspPlanningResultConstPtr &result)
  {
    planner_index_ = 0;

    //planning succeeded, send the results for feasibility testing
    if(state.state_ == state.SUCCEEDED)
    {
      ROS_INFO("Grasp plan action succeeded.");
      
      int num = result->grasps.size();

      planner_poses_.resize(num + 1);
      planner_states_.resize(num + 1);

      for(int i = 0; i < num; i++)
      {
        planner_poses_[i].pose = result->grasps[i].grasp_pose;
        planner_poses_[i].header = gripper_pose_.header;
        planner_poses_[i].header.stamp = ros::Time(0);
        planner_states_[i] = UNTESTED;
      }
      planner_poses_[num] = gripper_pose_;
      planner_poses_[num].header.stamp = ros::Time(0);
      planner_states_[num] = UNTESTED;
      initGraspMarkers();
      testing_planned_grasp_ = true;
      testing_alternatives_ = false;
      tested_grasp_index_ = 0;
      testing_current_grasp_ = false;
      testPoses(planner_poses_, gripper_opening_);
    }

    //planning failed, just check the original seed pose in gripper_pose_
    else
    {
      ROS_WARN("Grasp plan action did not succeed; state = %d", (int)state.state_);
      planner_poses_.resize(1);
      planner_states_.resize(1);
      planner_poses_[0] = gripper_pose_;
      planner_poses_[0].header.stamp = ros::Time(0);
      planner_states_[0] = UNTESTED;
      gripper_pose_ = planner_poses_[0];
      initMarkers();
      //initButtonMarker();
      pose_state_ = UNTESTED;
      testing_current_grasp_ = true;
      testing_planned_grasp_ = false;
      testing_alternatives_ = false;
      tested_grasp_index_ = 0;
      testPose(gripper_pose_, gripper_opening_);
    }
  }

  //! Cycle through planned grasps and initialize the grasp(n/m) button for switching
  void cycleGrasps(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    selectNextGrasp();
    initButtonMarker();
  }

  //! Return with the gripper pose if the pose is valid, otherwise do nothing
  void acceptCB()
  {
    if( pose_state_ == VALID )
    {
      if(test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::ACTIVE ||
         test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::PENDING )
      {
        test_pose_client_.client().cancelGoal();
      }
      setIdle();
      pr2_object_manipulation_msgs::GetGripperPoseResult result;
      result.gripper_pose = gripper_pose_;
      result.gripper_opening = gripper_opening_;
      get_pose_server_.setSucceeded(result);
    }
  }

  //! Cancel this action call
  void cancelCB()
  {
    if(test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::ACTIVE ||
       test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::PENDING )
    {
      test_pose_client_.client().cancelGoal();
    }
    get_pose_server_.setAborted();
    setIdle();
  }

  //! Sends a request for the 3D camera to focus on the ghosted gripper
  void focusCB()
  {
    pr2_object_manipulation_msgs::CameraFocus msg;
    msg.focal_point.point = gripper_pose_.pose.position;
    msg.focal_point.header = gripper_pose_.header;
    pub_focus_.publish(msg);
  }

  //! Generate poses that are shifted along axis, by amounts from min_shift to max_shift with spacing resolution
  std::vector<geometry_msgs::PoseStamped> generatePosesAlongDir(tf::Pose pose, tf::Vector3 axis, 
				       double min_shift, double max_shift, double resolution, std::string frame_id)
  {
    std::vector<geometry_msgs::PoseStamped> shifted_poses;
    //tf::Quaternion q = pose.getRotation();
    for(double dist=min_shift; dist<=max_shift; dist += resolution)
    {
      if(dist < 1e-6 && dist > -1e-6) continue;
      tf::Pose shifted_pose = pose*tf::Transform(tf::createIdentityQuaternion(), axis*dist);
      geometry_msgs::PoseStamped shifted_pose_stamped;
      tf::poseTFToMsg(shifted_pose, shifted_pose_stamped.pose);
      shifted_pose_stamped.header.frame_id = frame_id;
      if(fabs(min_shift) > fabs(max_shift)){
	shifted_poses.insert(shifted_poses.begin(), shifted_pose_stamped);
      }
      else shifted_poses.push_back(shifted_pose_stamped);
    }
    return shifted_poses;
  }

  //! Generate poses that are rotated about axis, by amounts from min_rot to max_rot with spacing resolution
  std::vector<geometry_msgs::PoseStamped> generateRotatedPoses(tf::Pose pose, tf::Vector3 axis, 
					double min_rot, double max_rot, double resolution, std::string frame_id)
  {
    std::vector<geometry_msgs::PoseStamped> rotated_poses;
    //tf::Quaternion q = pose.getRotation();
    for(double angle=min_rot; angle<=max_rot; angle += resolution)
    { 
      if(angle < 1e-6 && angle > -1e-6) continue;
      tf::Quaternion rot = tf::Quaternion(axis, angle); 
      tf::Pose rotated_pose = pose*tf::Transform(rot, tf::Vector3(0,0,0));
      geometry_msgs::PoseStamped rotated_pose_stamped;
      tf::poseTFToMsg(rotated_pose, rotated_pose_stamped.pose);      
      rotated_pose_stamped.header.frame_id = frame_id;
      if(fabs(min_rot) > fabs(max_rot)){
	rotated_poses.insert(rotated_poses.begin(), rotated_pose_stamped);
      }
      else rotated_poses.push_back(rotated_pose_stamped);
    }
    return rotated_poses;
  }

  //! Find alternatives that are valid grasps
  void alternativesCB()
  {
    if(test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::ACTIVE ||
       test_pose_client_.client().getState() == actionlib::SimpleClientGoalState::PENDING )
    {
      test_pose_client_.client().cancelGoal();
    }
    eraseAllGraspMarkers();

    //generate a bunch of poses to test in all directions around gripper_pose_
    geometry_msgs::PoseStamped fingertips_pose = fromWrist(gripper_pose_);
    tf::Pose pose;
    tf::poseMsgToTF(fingertips_pose.pose, pose);
    std::vector<geometry_msgs::PoseStamped> poses;
    std::string frame_id = gripper_pose_.header.frame_id;
    tf::Vector3 axes[3] = {tf::Vector3(1,0,0), tf::Vector3(0,1,0), tf::Vector3(0,0,1)};
    std::vector<geometry_msgs::PoseStamped> some_poses;
    std::vector<int> some_types;

    //generate shifted poses in all directions
    for(int ind=0; ind<3; ind++)
    {
      some_poses = generatePosesAlongDir(pose, axes[ind], 0, alternatives_search_dist_, alternatives_search_dist_resolution_, frame_id);
      poses.insert(poses.end(), some_poses.begin(), some_poses.end());
      planner_grasp_types_.insert(planner_grasp_types_.end(), some_poses.size(), ind*2);
      some_poses = generatePosesAlongDir(pose, axes[ind], -alternatives_search_dist_, 0, alternatives_search_dist_resolution_, frame_id);
      poses.insert(poses.end(), some_poses.begin(), some_poses.end());
      planner_grasp_types_.insert(planner_grasp_types_.end(), some_poses.size(), ind*2+1);
    }

    //generated rotated poses around all axes
    for(int ind=0; ind<3; ind++)
    {
      some_poses = generateRotatedPoses(pose, axes[ind], 0, alternatives_search_angle_, alternatives_search_angle_resolution_, frame_id);
      poses.insert(poses.end(), some_poses.begin(), some_poses.end());
      planner_grasp_types_.insert(planner_grasp_types_.end(), some_poses.size(), ind*2+6);
      some_poses = generateRotatedPoses(pose, axes[ind], -alternatives_search_angle_, 0, alternatives_search_angle_resolution_, frame_id);
      poses.insert(poses.end(), some_poses.begin(), some_poses.end());
      planner_grasp_types_.insert(planner_grasp_types_.end(), some_poses.size(), ind*2+7);
    }

    //switch back to wrist frame
    for(size_t i=0; i<poses.size(); i++)
    {
      planner_poses_.push_back(toWrist(poses[i]));
      planner_states_.push_back(UNTESTED);
    }		
	       
    //add gripper_pose_
    planner_poses_.push_back(gripper_pose_);
    planner_states_.push_back(UNTESTED);
    planner_grasp_types_.push_back(-1);

    button_marker_pose_ = gripper_pose_;
    button_marker_pose_.header.stamp = ros::Time(0);
    button_marker_pose_.pose.orientation = geometry_msgs::Quaternion();
    button_marker_pose_.pose.orientation.w = 1;
    button_marker_pose_.pose.position.z -= 0.2;

    //test the generated poses
    testing_planned_grasp_ = true;
    testing_alternatives_ = true;
    tested_grasp_index_ = 0;
    testing_current_grasp_ = false;
    //initGraspMarkers();
    testPoses(planner_poses_, gripper_opening_);    
  } 

  //! Called when the gripper is clicked; each call cycles through gripper opening values.
  void gripperClickCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    //disabled for user studies or if the param is set
    if(interface_number_ != 0 || !gripper_opening_cycling_) return;

    ros::Time now = ros::Time(0);

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        //ROS_DEBUG_STREAM( "Marker is being moved, stored pose is invalidated." );
        float max_gripper_angle = 0.541;
        gripper_angle_ -= 0.12;
        if(gripper_angle_ < 0.04)
          gripper_angle_ = max_gripper_angle;
        updateGripperOpening();
        initGripperMarker();
        ROS_DEBUG( "Gripper opening = %.2f, angle = %.2f", gripper_opening_, gripper_angle_);
        break;
    }
  }

  //! Test one grasp pose
  void testPose(geometry_msgs::PoseStamped pose, float opening)
  {
    eraseAllGraspMarkers();
    pr2_object_manipulation_msgs::TestGripperPoseGoal goal;
    goal.gripper_poses.push_back(pose);
    goal.gripper_openings.push_back(opening);
    test_pose_client_.client().sendGoal( goal, boost::bind(&GripperPoseAction::testGripperResultCallback, this, _1, _2));
  }

  //! Test a list of grasp poses
  void testPoses(std::vector<geometry_msgs::PoseStamped> poses, float opening)
  {
    //eraseAllGraspMarkers();
    pr2_object_manipulation_msgs::TestGripperPoseGoal goal;
    for(size_t i=0; i<poses.size(); i++){
      goal.gripper_poses.push_back(poses[i]);
      goal.gripper_openings.push_back(opening);
    }
    test_pose_client_.client().sendGoal( goal, boost::bind(&GripperPoseAction::testGripperResultCallback, this, _1, _2));
  }

  //! Callback for pose updates from the controls.
  void updateGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    ros::Time now = ros::Time(0);

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_DEBUG_STREAM( "Marker is being moved, stored pose is invalidated." );
        test_pose_client_.client().cancelAllGoals();
        pose_state_ = UNTESTED;
        eraseAllGraspMarkers();
        break;
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_DEBUG_STREAM( "Marker was released, storing pose and checking." );
        gripper_pose_.pose = feedback->pose;
        gripper_pose_.header = feedback->header;
        gripper_pose_ = toWrist(gripper_pose_);
        initMarkers();
        testing_planned_grasp_ = false;
        testing_alternatives_ = false;
        testing_current_grasp_ = true;
        testPose(gripper_pose_, gripper_opening_);
        break;
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_DEBUG_STREAM("POSE_UPDATE in frame " << feedback->header.frame_id << std::endl << feedback->pose);
        gripper_pose_.pose = feedback->pose;
        gripper_pose_.header = feedback->header;
        gripper_pose_ = toWrist(gripper_pose_);
        updatePoses();
        break;
    }
  }

  //! Initialize the menus for all markers.
  void initMenus()
  {
    accept_handle_ = menu_gripper_.insert("Accept", boost::bind( &GripperPoseAction::acceptCB, this ) );
    cancel_handle_ = menu_gripper_.insert("Cancel", boost::bind( &GripperPoseAction::cancelCB, this ) );
    focus_handle_ = menu_gripper_.insert("Focus camera", boost::bind( &GripperPoseAction::focusCB, this ) );
    alternatives_handle_ = menu_gripper_.insert("Find alternatives", boost::bind( &GripperPoseAction::alternativesCB, this ) );
    if(interface_number_ == 0  || interface_number_ == 4)
    {
      menu_gripper_.insert("Run Grasp Planner", boost::bind( &GripperPoseAction::graspPlanCB, this ) );
    }
  }

  //! Get the mesh corresponding to a database model_id
  bool getModelMesh( int model_id, arm_navigation_msgs::Shape& mesh )
  {
    household_objects_database_msgs::GetModelMesh mesh_srv;

    mesh_srv.request.model_id = model_id;
    if ( !get_model_mesh_client_.call(mesh_srv) )
    {
      ROS_ERROR("Failed to call get model mesh service");
      return false;
    }

    if (mesh_srv.response.return_code.code != household_objects_database_msgs::DatabaseReturnCode::SUCCESS)
    {
      ROS_ERROR("Model mesh service reports an error (code %d)", mesh_srv.response.return_code.code);
      return false;
    }

    mesh = mesh_srv.response.mesh;
    return true;
  }



  //! Create an interactive marker from a point cloud.
  visualization_msgs::InteractiveMarker makeCloudMarker( const char *name, const geometry_msgs::PoseStamped &stamped,
                                                         float point_size, std_msgs::ColorRGBA color)
  {
    InteractiveMarker int_marker;
    int_marker.name = name;
    int_marker.pose = stamped.pose;
    int_marker.header = stamped.header;

    Marker marker;
    marker.color = color;
    marker.frame_locked = false;

    if(object_cloud_->points.size())
    {
      //int_marker.header = object_cloud_->header;
      marker.scale.x = point_size;
      marker.scale.y = point_size;
      marker.scale.z = point_size;
      marker.type = visualization_msgs::Marker::SPHERE_LIST;

      int num_points = object_cloud_->points.size();
      marker.points.resize( num_points );
      marker.colors.resize( num_points );

      //ROS_INFO_STREAM( "Adding point cluster. #points=" << object_.cluster.points.size() );

      for ( int i=0; i<num_points; i++)
      {
        marker.points[i].x = object_cloud_->points[i].x;
        marker.points[i].y = object_cloud_->points[i].y;
        marker.points[i].z = object_cloud_->points[i].z;
//        marker.colors[i].r = object_cloud_->points[i].r/255.;
//        marker.colors[i].g = object_cloud_->points[i].g/255.;
//        marker.colors[i].b = object_cloud_->points[i].b/255.;
//        marker.colors[i].a = 1.0;
      }
    }
    else
    {

    }

    InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    return int_marker;
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_interactive_pose_select_action");
  GripperPoseAction gripper_pose_action;

  ros::Duration(1.0).sleep();

  ros::spin();
}
