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

#include <arm_navigation_msgs/GetStateValidity.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <object_manipulator/tools/mechanism_interface.h>
#include <object_manipulator/tools/msg_helpers.h>
#include <object_manipulator/tools/arm_configurations.h>
#include <object_manipulator/tools/vector_tools.h>
#include <object_manipulator/tools/service_action_wrappers.h>

#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <boost/thread.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pr2_wrappers/gripper_controller.h>
#include <pr2_wrappers/torso_client.h>
#include <pr2_wrappers/base_client.h>
#include <pr2_wrappers/tuck_arms_client.h>
#include <pr2_wrappers/plugs_client.h>
#include <point_cloud_server/StoreCloudAction.h>

#include <pr2_object_manipulation_msgs/GetNavPoseAction.h>
#include <pr2_object_manipulation_msgs/GetGripperPoseAction.h>

#include <interactive_marker_helpers/interactive_marker_helpers.h>
#include <pr2_marker_control/cloud_handler.h>

#include <boost/thread/thread.hpp>

//! A class for controlling the PR2 using interactive markers.
class PR2MarkerControl
{
public:

  //! Constructor
  PR2MarkerControl();

  //! Destructor
  virtual ~PR2MarkerControl() { switchToJoint();  }

// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

  //! For fast updates, like changes to control and mesh marker poses.
  void fastUpdate();

  //! Used for lower frequency state updates.
  void slowUpdate();

  //! Finds the state of a given joint in the robot state
  double getJointPosition( std::string name, const arm_navigation_msgs::RobotState& robot_state);

   //! Translate the tool pose to the wrist.
  geometry_msgs::PoseStamped toolToWrist(const geometry_msgs::PoseStamped &ps);

  //! Translate the wrist pose to the tool frame.
  geometry_msgs::PoseStamped wristToTool(const geometry_msgs::PoseStamped &ps);

  //! Re-initialize all interactive markers
  void initAllMarkers(bool apply_immediately = false)
  {
    initControlMarkers();
    initMeshMarkers();

    if(apply_immediately)
    {
      ROS_INFO_STREAM( "Re-initializing all markers!" );
      server_.applyChanges();
    }
  }

  //! Re-initialize only the control markers (like the gripper-dragging frame)
  void initControlMarkers();

  //! Re-initialize only the mesh markers
  void initMeshMarkers();

  //! Cancel any base movement.
  void cancelBaseMovement() {
      base_goal_pose_.header.stamp = ros::Time::now();
      base_client_.cancelGoals();
  }


protected:

  //! Activates the cartesian controllers for the grippers.
  void switchToCartesian();

  //! Activates the joint controllers for the grippers
  void switchToJoint();

  //! Checks if the arm pose is currently in collision.
  bool checkStateValidity(std::string arm_name);

  //!
  void gripperToggleModeCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void snapshotCB() { snapshot_client_.refresh( nh_.resolveName("snapshot_input_topic", true) ); }

  void gripperToggleFixedCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void dualGripperToggleFixedCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void gripperToggleControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void dualGripperToggleControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void gripperResetControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void dualGripperResetControlCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void startDualGrippers( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, bool active );


  void updateGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int arm_id );

  void updateDualGripper( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  //! Turns the gripper controls on and off
  void gripperButtonCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, std::string action);

  void upperArmButtonCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int id);


  void baseButtonCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    control_state_.base_on_ ^= true;
    initControlMarkers();
  }

  void refreshPosture( const std::string &arm_name )
  {
    std::vector<double> arm_angles(7);
    for( size_t i = 0; i < arm_angles.size(); i++) arm_angles[i] = 9999;
    mechanism_.sendCartesianPostureCommand(arm_name, arm_angles);
  }

  void tuckArmsCB( bool tuck_left, bool tuck_right) 
  { 
    switchToJoint();
    tuck_arms_client_.tuckArms( tuck_right, tuck_left, false );  
  }

  typedef enum {PLUGS_PLUGIN, PLUGS_UNPLUG, PLUGS_CANCEL} plugs_cmd_t_;
  void plugsCB( plugs_cmd_t_ cmd )
  {
	  switchToJoint();
	  if (cmd!=PLUGS_CANCEL) {
		  plugs_client_.plug_unplug( (cmd==PLUGS_PLUGIN) ? true : false, false );
	  } else {
		  plugs_client_.cancel();
	  }

  }

  void updatePosture( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int arm_id );

  void projectorMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void targetPointMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void centerHeadCB()
  {
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "/base_link";    
    ps.point.x = 1.0;
    mechanism_.pointHeadAction(ps, head_pointing_frame_, false);
  }

  // Moved to graveyard
  //void controllerSelectMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void updateTorso( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void updateBase( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void torsoMenuCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void gripperClosureCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const float &command);

  void plugCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, const bool &plug);

  void updateHeadGoal( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int arm_id );

  void commandGripperPose(const geometry_msgs::PoseStamped &ps, int arm_id, bool use_offset);

  void requestGripperPose(  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, int arm_id);

  void processGripperPoseFeedback(  const pr2_object_manipulation_msgs::GetGripperPoseFeedbackConstPtr &result,
                                    const std::string &arm_name);

  void processGripperPoseResult(  const actionlib::SimpleClientGoalState& state,
                                  const pr2_object_manipulation_msgs::GetGripperPoseResultConstPtr &result,
                                  const std::string &arm_name);

  void moveArm( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
                const std::string &position, bool planner);

  void moveArmThread(std::string arm_name, std::string position, bool collision, bool planner);

  void initMenus();

  void requestNavGoal(  const bool & collision_aware );

  void processNavGoal(  const actionlib::SimpleClientGoalState& state,
                        const pr2_object_manipulation_msgs::GetNavPoseResultConstPtr &result,
                        const bool &collision_aware );

  void requestBasePoseEstimate( );


  void processBasePoseEstimate( const actionlib::SimpleClientGoalState& state,
                                const pr2_object_manipulation_msgs::GetNavPoseResultConstPtr &result );
  
  //! This is useful so we can bind it to a callback. 
  void sendLastNavGoal();
  
  void clearLocalCostmap();

  void inHandObjectRightCallback( const sensor_msgs::PointCloud2ConstPtr &cloud);

  void inHandObjectLeftCallback( const sensor_msgs::PointCloud2ConstPtr &cloud);

protected:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Timer spin_timer_;
  ros::Timer slow_sync_timer_;
  interactive_markers::InteractiveMarkerServer server_;
  //ros::Publisher pub_cloud_snapshot_;

  //! Menu handles for the robot.
  interactive_markers::MenuHandler menu_arms_;
  interactive_markers::MenuHandler menu_head_;
  interactive_markers::MenuHandler menu_torso_;
  interactive_markers::MenuHandler menu_laser_;
  interactive_markers::MenuHandler menu_grippers_;
  interactive_markers::MenuHandler menu_dual_grippers_;
  interactive_markers::MenuHandler menu_gripper_close_;
  interactive_markers::MenuHandler menu_base_;

  //! Menu Entry handles for some important ones.
  interactive_markers::MenuHandler::EntryHandle tuck_handle_;
  interactive_markers::MenuHandler::EntryHandle joint_handle_;
  interactive_markers::MenuHandler::EntryHandle jtranspose_handle_;
  interactive_markers::MenuHandler::EntryHandle posture_handle_;
  interactive_markers::MenuHandler::EntryHandle projector_handle_;
  interactive_markers::MenuHandler::EntryHandle head_target_handle_;
  interactive_markers::MenuHandler::EntryHandle gripper_view_facing_handle_;
  interactive_markers::MenuHandler::EntryHandle gripper_6dof_handle_;
  interactive_markers::MenuHandler::EntryHandle gripper_fixed_control_handle_;
  interactive_markers::MenuHandler::EntryHandle dual_gripper_fixed_control_handle_;
  interactive_markers::MenuHandler::EntryHandle gripper_edit_control_handle_;
  interactive_markers::MenuHandler::EntryHandle dual_gripper_edit_control_handle_;
  interactive_markers::MenuHandler::EntryHandle gripper_reset_control_handle_;
  interactive_markers::MenuHandler::EntryHandle dual_gripper_reset_control_handle_;

  tf::TransformListener tfl_;
  tf::TransformBroadcaster tfb_;

  std::vector<tf::Transform> pose_offset_;
  tf::Transform dual_pose_offset_;
  geometry_msgs::Pose tool_frame_offset_;
  geometry_msgs::PoseStamped head_goal_pose_;

  geometry_msgs::PoseStamped dual_grippers_frame_;
  std::vector<tf::Transform> dual_gripper_offsets_;

  std::string manipulator_base_frame_;

  geometry_msgs::PoseStamped base_goal_pose_;

  //! A struct to store the state of the gripper controls.
  struct GripperState{
    GripperState() : on_(false), view_facing_(false), edit_control_(false), torso_frame_(false) {}

    bool on_;
    bool view_facing_;
    bool edit_control_;
    bool torso_frame_;
  };

  //! A struct to keep the state of the overall robot control.
  struct ControlState{
    ControlState() : posture_r_(false), posture_l_(false), torso_on_(false), head_on_(false),
                     projector_on_(false), init_head_goal_(false), base_on_(false) {}

    void print()
    {
      ROS_DEBUG_NAMED("control_state", "gripper: on[%d|%d][%d], edit[%d|%d][%d], torso[%d|%d]",
                      l_gripper_.on_, r_gripper_.on_, dual_grippers_.on_, l_gripper_.edit_control_, r_gripper_.edit_control_, dual_grippers_.edit_control_, l_gripper_.torso_frame_, r_gripper_.torso_frame_);
      ROS_DEBUG_NAMED("control_state", "posture[%d|%d] torso[%d] base[%d] head[%d] projector[%d]",
                      posture_l_, posture_r_, torso_on_, base_on_, head_on_, projector_on_ );
    }

    bool posture_r_;
    bool posture_l_;
    bool torso_on_;
    bool head_on_;
    bool projector_on_;
    bool init_head_goal_;
    bool base_on_;
    bool planar_only_;
    GripperState dual_grippers_;
    GripperState r_gripper_;
    GripperState l_gripper_;
  };

  ControlState control_state_;
  pr2_wrappers::GripperController gripper_client_;
  pr2_wrappers::TorsoClient torso_client_;
  pr2_wrappers::BaseClient base_client_;
  pr2_wrappers::TuckArmsClient tuck_arms_client_;
  pr2_wrappers::PlugsClient plugs_client_;

  boost::mutex planner_lock_;

  object_manipulator::MechanismInterface mechanism_;

  bool use_state_validator_;
  bool use_right_arm_;
  bool use_left_arm_;
  double gripper_control_linear_deadband_, gripper_control_angular_deadband_;
  double update_period_;
  double cartesian_clip_distance_;
  double cartesian_clip_angle_;

  double max_direct_nav_radius_;

  bool in_collision_r_, in_collision_l_;

  boost::shared_ptr< boost::thread > sys_thread_;

  //! Client for service that checks state validity
  object_manipulator::ServiceWrapper<arm_navigation_msgs::GetStateValidity> check_state_validity_client_;
  object_manipulator::ServiceWrapper<std_srvs::Empty> collider_node_reset_srv_;
  CloudHandler snapshot_client_;
  CloudHandler object_cloud_left_;
  CloudHandler object_cloud_right_;

  //! Subscribers for in-hand object clouds
  ros::Subscriber object_cloud_left_sub_;
  ros::Subscriber object_cloud_right_sub_;

  //! Client for getting a base pose
  object_manipulator::ActionWrapper<pr2_object_manipulation_msgs::GetNavPoseAction> base_pose_client_;

  //! Client for getting a gripper pose
  object_manipulator::ActionWrapper<pr2_object_manipulation_msgs::GetGripperPoseAction> gripper_pose_client_;

  //! For HRI user study
  int interface_number_;
  //! For HRI user study
  int task_number_;

  std::string head_pointing_frame_;

  //! Name of the move base node
  std::string move_base_node_name_;

  //! Are we using 3D navigation?
  bool using_3d_nav_;

  //stuff for the rmrc controller
  bool alignedOdomValid_;
  boost::thread alignedOdomThread_;
  boost::mutex alignedOdomMutex_;
  tf::StampedTransform alignedOdom_;

  //! What type of gripper we have
  /*! For now, "pr2" or "lcg" */
  std::string l_gripper_type_, r_gripper_type_;

  void publishAlignedOdom() {
    while ( true ) {
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
      {
	boost::mutex::scoped_lock lock(alignedOdomMutex_);
	if ( alignedOdomValid_ ) {
	  alignedOdom_.stamp_ = ros::Time::now();
	  tfb_.sendTransform(alignedOdom_);
	}
      }
    }
  }

};



