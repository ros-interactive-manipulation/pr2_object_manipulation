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
 */

#ifndef INTERACTIVE_MANIPULATION_BACKEND
#define INTERACTIVE_MANIPULATION_BACKEND

#include <boost/thread/mutex.hpp>

#include <stdexcept>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/Empty.h>

#include <object_manipulator/tools/mechanism_interface.h>
#include <object_manipulator/tools/hand_description.h>
#include <object_manipulator/tools/service_action_wrappers.h>
#include <object_manipulator/tools/ik_tester_fast.h>

#include <tabletop_collision_map_processing/collision_map_interface.h>

#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

#include <pr2_create_object_model/ModelObjectInHandAction.h>
#include <pr2_create_object_model/ObjectInHand.h>

#include <pr2_object_manipulation_msgs/GetGripperPoseAction.h>
#include <pr2_object_manipulation_msgs/TestGripperPoseAction.h>

#include "pr2_wrappers/gripper_controller.h"

#include "pr2_object_manipulation_msgs/IMGUIOptions.h"
#include "pr2_object_manipulation_msgs/IMGUIAdvancedOptions.h"
#include "pr2_object_manipulation_msgs/IMGUICommand.h"
#include "pr2_object_manipulation_msgs/IMGUIAction.h"
#include "pr2_object_manipulation_msgs/ImageClick.h"
#include "pr2_object_manipulation_msgs/RunScriptAction.h"

#include "move_base_msgs/MoveBaseAction.h"

namespace pr2_interactive_manipulation {

class InteractiveManipulationBackend
{

public:
  InteractiveManipulationBackend();
  ~InteractiveManipulationBackend();

private:
  
  struct GraspInfo
  {
    static geometry_msgs::Pose verticalGripper()
    {
      geometry_msgs::Pose pose;
      pose.position.x = 0;
      pose.position.y = 0;
      pose.position.z = 0.2;
      pose.orientation.x = 0;
      pose.orientation.y = sqrt(0.5);
      pose.orientation.z = 0;
      pose.orientation.w = sqrt(0.5);
      return pose;
    }
    static geometry_msgs::Pose identityPose()
    {
      geometry_msgs::Pose pose;
      pose.position.x = pose.position.y = pose.position.z = 0.0;
      pose.orientation = identityQuaternion();
      return pose;
    }
    static geometry_msgs::Quaternion identityQuaternion()
    {
      geometry_msgs::Quaternion q;
      q.x = q.y = q.z = 0.0;
      q.w = 1.0;
      return q;
    }

    GraspInfo()
    {
      reset();
    }
    void reset()
    {
      object_manipulation_msgs::GraspableObject foo;
      object_ = foo;
      grasp_.grasp_pose = verticalGripper();
      object_.reference_frame_id = "base_link";
      object_orientation_ = identityQuaternion();
    }
    std::string object_collision_name_;
    object_manipulation_msgs::GraspableObject object_;
    object_manipulation_msgs::Grasp grasp_;
    geometry_msgs::Quaternion object_orientation_;
  };

  GraspInfo* getGraspInfo(std::string arm_name)
  {
    if (arm_name=="right_arm") return &grasp_info_right_;
    return &grasp_info_left_;
  }

  // Administrative
  void actionCallback(const pr2_object_manipulation_msgs::IMGUIGoalConstPtr &goal);
  void imageClickCallback(const pr2_object_manipulation_msgs::ImageClickConstPtr &click);
  void pickupFeedbackCallback(const object_manipulation_msgs::PickupFeedbackConstPtr &feedback);
  bool interruptRequested();
  bool checkInterrupts();
  void setStatusLabel(std::string text);

  // Main functions
  int pickupObject(const pr2_object_manipulation_msgs::IMGUIOptions &options,
                   object_manipulation_msgs::GraspableObject object = object_manipulation_msgs::GraspableObject() );
  int placeObject(const pr2_object_manipulation_msgs::IMGUIOptions &options);
  int plannedMove(const pr2_object_manipulation_msgs::IMGUIOptions &options);
  void collisionReset(int reset_choice, int arm_selection_choice);
  void lookAtTable();
  void armMotion(int arm_selection_choice, int arm_action_choice, 
                 int arm_planner_choice, bool collision, object_manipulation_msgs::ManipulationResult &result);
  void openCloseGripper(pr2_object_manipulation_msgs::IMGUIOptions options);
  int modelObject(pr2_object_manipulation_msgs::IMGUIOptions options);
  int runScriptedAction(std::string action_name, std::string group_name, pr2_object_manipulation_msgs::IMGUIOptions options);

  // Helper functions
  bool processCollisionMapForPickup(const pr2_object_manipulation_msgs::IMGUIOptions &options, object_manipulation_msgs::PickupGoal &goal);
  int callGhostedGripperPickup( std::string arm_name, object_manipulation_msgs::Grasp &grasp );
  int callGhostedGripperMove( std::string arm_name, geometry_msgs::PoseStamped &location );
  int callGhostedGripper(const pr2_object_manipulation_msgs::GetGripperPoseGoal &goal,
                         pr2_object_manipulation_msgs::GetGripperPoseResult &result );
  bool getGrasp(object_manipulation_msgs::Grasp &grasp, std::string arm_name,
                geometry_msgs::PoseStamped grasp_pose, float gripper_opening);

  // Test gripper pose
  void testGripperPoseCallback(const pr2_object_manipulation_msgs::TestGripperPoseGoalConstPtr &goal);
  void testGripperPoseForGraspCallback(const pr2_object_manipulation_msgs::TestGripperPoseGoalConstPtr &goal);
  void testGripperPoseForPlaceCallback(const pr2_object_manipulation_msgs::TestGripperPoseGoalConstPtr &goal);
  void testGripperPoseForMoveCallback(const pr2_object_manipulation_msgs::TestGripperPoseGoalConstPtr &goal);

  // Node handles
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;

  // Action servers
  actionlib::SimpleActionServer<pr2_object_manipulation_msgs::IMGUIAction> *action_server_;
  actionlib::SimpleActionServer<pr2_object_manipulation_msgs::TestGripperPoseAction> *test_gripper_pose_server_;

  // Topic publishers
  ros::Publisher status_pub_;

  // Action clients
  object_manipulator::ActionWrapper<object_manipulation_msgs::PickupAction> pickup_client_;
  object_manipulator::ActionWrapper<object_manipulation_msgs::PlaceAction> place_client_;
  object_manipulator::ActionWrapper<pr2_create_object_model::ModelObjectInHandAction> create_model_client_;
  object_manipulator::ActionWrapper<pr2_object_manipulation_msgs::GetGripperPoseAction> get_gripper_pose_client_;
  object_manipulator::ActionWrapper<pr2_object_manipulation_msgs::RunScriptAction> run_script_client_;
  object_manipulator::ActionWrapper<move_base_msgs::MoveBaseAction> move_base_client_;

  // Service clients
  object_manipulator::ServiceWrapper<std_srvs::Empty> collider_node_reset_srv_;

  // Topic subscribers
  ros::Subscriber image_click_sub_;

  // Topic publishers
  ros::Publisher in_hand_object_right_pub_;
  ros::Publisher in_hand_object_left_pub_;

  // Current action bookkeeping
  enum Action {PICKUP, PLACE, MOVE};
  Action current_action_;
  object_manipulation_msgs::PickupGoal current_pickup_goal_;
  object_manipulation_msgs::PlaceGoal current_place_goal_;
  boost::mutex pipeline_mutex_;
  pr2_object_manipulation_msgs::IMGUIOptions options_;

  // Lower level interfaces
  pr2_wrappers::GripperController gripper_client_;
  tabletop_collision_map_processing::CollisionMapInterface collision_map_interface_;
  object_manipulator::MechanismInterface mech_interface_;
  object_manipulator::IKTesterFast ik_tester_fast_;

  std::string action_name_;
  std::string test_gripper_pose_action_name_;
  std::string interactive_manipulation_status_name_;
  std::string image_click_name_;
  
  // World state
  GraspInfo grasp_info_right_;
  GraspInfo grasp_info_left_;
  sensor_msgs::PointCloud2 object_model_left_;
  sensor_msgs::PointCloud2 object_model_right_;

  tf::TransformListener listener_;

  // Params for moveArmToPoseCartesian
  double cartesian_dist_tol_;
  double cartesian_angle_tol_;
  double cartesian_overshoot_dist_;
  double cartesian_overshoot_angle_;

};

}

#endif
