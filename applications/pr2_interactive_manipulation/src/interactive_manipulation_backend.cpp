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

#include "pr2_interactive_manipulation/interactive_manipulation_backend.h"

#include <std_msgs/String.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <object_manipulation_msgs/Grasp.h>
#include <object_manipulation_msgs/ClusterBoundingBox.h>
#include <object_manipulation_msgs/tools.h>

#include <object_manipulator/tools/vector_tools.h>
#include <object_manipulator/tools/arm_configurations.h>
#include <sensor_msgs/point_cloud_conversion.h>

using object_manipulation_msgs::ManipulationResult;
using object_manipulator::InterruptRequestedException;
using pr2_object_manipulation_msgs::TestGripperPoseAction;
using pr2_object_manipulation_msgs::TestGripperPoseGoal;
using pr2_object_manipulation_msgs::TestGripperPoseGoalConstPtr;
using pr2_object_manipulation_msgs::GetGripperPoseAction;
using pr2_object_manipulation_msgs::GetGripperPoseGoal;
using pr2_object_manipulation_msgs::GetGripperPoseResult;
//using namespace pr2_object_manipulation_msgs; // For IMGUI stuff

namespace pr2_interactive_manipulation {

typedef actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> PickupClient; 	 
	
geometry_msgs::Pose translatePose(geometry_msgs::Pose pose_in, tf::Vector3 translation)
{
  tf::Transform trans_in;
  tf::poseMsgToTF(pose_in, trans_in);
  tf::StampedTransform translate_trans;
  translate_trans.setIdentity();
  translate_trans.setOrigin(translation);
  tf::Transform trans_out = trans_in * translate_trans;
  geometry_msgs::Pose pose_out;
  tf::poseTFToMsg(trans_out, pose_out);
  return pose_out;
}

geometry_msgs::Pose preTranslatePose(geometry_msgs::Pose pose_in, tf::Vector3 translation)
{
  tf::Transform trans_in;
  tf::poseMsgToTF(pose_in, trans_in);
  tf::StampedTransform translate_trans;
  translate_trans.setIdentity();
  translate_trans.setOrigin(translation);
  tf::Transform trans_out = translate_trans * trans_in;
  geometry_msgs::Pose pose_out;
  tf::poseTFToMsg(trans_out, pose_out);
  return pose_out;
}

InteractiveManipulationBackend::InteractiveManipulationBackend() : 
  root_nh_(""),
  priv_nh_("~"),  
  pickup_client_("/object_manipulator/object_manipulator_pickup", true),
  place_client_("/object_manipulator/object_manipulator_place", true),
  create_model_client_("/create_object_model_server/model_object_in_hand_action", true),
  get_gripper_pose_client_("/get_pose_server", true),
  run_script_client_("run_rcommander_action", true),
  move_base_client_("move_base", true),
  collider_node_reset_srv_("/collider_node/reset")
{
  priv_nh_.param<double>("cartesian_dist_tol", cartesian_dist_tol_, .01);
  priv_nh_.param<double>("cartesian_angle_tol", cartesian_angle_tol_, .087);
  priv_nh_.param<double>("cartesian_overshoot_dist", cartesian_overshoot_dist_, .005);
  priv_nh_.param<double>("cartesian_overshoot_angle", cartesian_overshoot_angle_, .087);

  action_name_ = "imgui_action";  
  action_server_ = new actionlib::SimpleActionServer<pr2_object_manipulation_msgs::IMGUIAction>(root_nh_, action_name_,
                                               boost::bind(&InteractiveManipulationBackend::actionCallback, this, _1),
                                                                  false);
  action_server_->start();
  
  test_gripper_pose_action_name_="test_gripper_pose";
  test_gripper_pose_server_ = new actionlib::SimpleActionServer<TestGripperPoseAction>(root_nh_, 
                                                                                       test_gripper_pose_action_name_, 
                                       boost::bind(&InteractiveManipulationBackend::testGripperPoseCallback, this, _1),
                                                                  false);
  test_gripper_pose_server_->start();

  interactive_manipulation_status_name_ = "interactive_manipulation_status";
  status_pub_ = root_nh_.advertise<std_msgs::String>(interactive_manipulation_status_name_, 1, true );
  in_hand_object_right_pub_ = root_nh_.advertise<sensor_msgs::PointCloud2>("/in_hand_object_right", 1);
  in_hand_object_left_pub_ = root_nh_.advertise<sensor_msgs::PointCloud2>("/in_hand_object_left", 1);

  image_click_name_ = "/interactive_manipulation_image_click";
  image_click_sub_ = root_nh_.subscribe(image_click_name_, 1, &InteractiveManipulationBackend::imageClickCallback, this);

  pickup_client_.setInterruptFunction(boost::bind(&InteractiveManipulationBackend::interruptRequested, this));
  place_client_.setInterruptFunction(boost::bind(&InteractiveManipulationBackend::interruptRequested, this));
  create_model_client_.setInterruptFunction(boost::bind(&InteractiveManipulationBackend::interruptRequested, this));
  get_gripper_pose_client_.setInterruptFunction(boost::bind(&InteractiveManipulationBackend::interruptRequested, this));
  
  ROS_INFO("IM Backend ready");
}

InteractiveManipulationBackend::~InteractiveManipulationBackend()
{
  delete action_server_;
  delete test_gripper_pose_server_;
}

void InteractiveManipulationBackend::actionCallback(const pr2_object_manipulation_msgs::IMGUIGoalConstPtr &goal)
{
  //boost::mutex::scoped_try_lock stl(action_mutex_)
  ROS_DEBUG("IM Backend received goal with command %d", goal->command.command);
  pr2_object_manipulation_msgs::IMGUIResult im_result;
  try
  {
    switch (goal->command.command)
    {
    case pr2_object_manipulation_msgs::IMGUICommand::PICKUP:
      if (goal->options.grasp_selection == 0) im_result.result.value = pickupObject(goal->options);
      else im_result.result.value = pickupObject(goal->options, goal->options.selected_object);
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::PLACE:
      im_result.result.value = placeObject(goal->options);
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::PLANNED_MOVE:
      im_result.result.value = plannedMove(goal->options);
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::RESET:
      collisionReset(goal->options.reset_choice, goal->options.arm_selection);
      im_result.result.value = object_manipulation_msgs::ManipulationResult::SUCCESS;
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::MOVE_ARM:
      armMotion(goal->options.arm_selection, goal->options.arm_action_choice, 
                goal->options.arm_planner_choice, goal->options.collision_checked, im_result.result);
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::MOVE_GRIPPER:
      openCloseGripper(goal->options);
      im_result.result.value = object_manipulation_msgs::ManipulationResult::SUCCESS;
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::LOOK_AT_TABLE:
      lookAtTable();
      im_result.result.value = object_manipulation_msgs::ManipulationResult::SUCCESS;
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::MODEL_OBJECT:
      im_result.result.value = modelObject(goal->options);
      break;
    case pr2_object_manipulation_msgs::IMGUICommand::SCRIPTED_ACTION:
      im_result.result.value = runScriptedAction(goal->command.script_name, 
              goal->command.script_group_name, 
              goal->options);
      break;      
    case pr2_object_manipulation_msgs::IMGUICommand::STOP_NAV:
      move_base_client_.client().cancelAllGoals();
      break;
    default:
      ROS_ERROR("IM Backend could not understand command: %d", goal->command.command);
      setStatusLabel("Command not yet implemented");
      im_result.result.value = object_manipulation_msgs::ManipulationResult::ERROR;
    }
    action_server_->setSucceeded(im_result);
    ROS_DEBUG("IM Backend: goal finished");
  }
  catch (InterruptRequestedException &ex)
  {
    //switch to Cartesian controllers to halt any joint trajectories in a hurry
    mech_interface_.switchToCartesian("left_arm");
    mech_interface_.switchToCartesian("right_arm");

    if (pickup_client_.isInitialized()) pickup_client_.client().cancelAllGoals();
    if (place_client_.isInitialized())  place_client_.client().cancelAllGoals();
    if (create_model_client_.isInitialized()) create_model_client_.client().cancelAllGoals();
    if (get_gripper_pose_client_.isInitialized()) get_gripper_pose_client_.client().cancelAllGoals();
    if (run_script_client_.isInitialized()) run_script_client_.client().cancelAllGoals();
    im_result.result.value = object_manipulation_msgs::ManipulationResult::CANCELLED;
    action_server_->setSucceeded(im_result);
    ROS_INFO("IM Backend: cancelled");
    setStatusLabel("cancelled.");
  }
  catch (object_manipulator::ServiceNotFoundException &ex)
  {
    setStatusLabel("a needed service or action server was not found");
    im_result.result.value = im_result.result.ERROR;
    action_server_->setSucceeded(im_result);
  }
}

bool InteractiveManipulationBackend::getGrasp(object_manipulation_msgs::Grasp &grasp, std::string arm_name,
                                              geometry_msgs::PoseStamped grasp_pose, float gripper_opening)
{
  std::vector<std::string> joint_names = object_manipulator::handDescription().handJointNames(arm_name);
  grasp.pre_grasp_posture.name = joint_names;
  grasp.grasp_posture.name = joint_names;  
  double gripper_angle = gripper_opening * 0.5 / 0.0857;
  grasp.pre_grasp_posture.position.resize( joint_names.size(), gripper_angle);
  grasp.grasp_posture.position.resize( joint_names.size(), 0.0);     
  grasp.grasp_posture.effort.resize(joint_names.size(), 50);
  grasp.pre_grasp_posture.effort.resize(joint_names.size(), 100);
  grasp.desired_approach_distance = options_.adv_options.desired_approach / 100.;
  grasp.min_approach_distance = options_.adv_options.min_approach / 100.;
  geometry_msgs::PoseStamped pose_stamped_in = grasp_pose;
  pose_stamped_in.header.stamp = ros::Time(0);
  geometry_msgs::PoseStamped pose_stamped;
  try
  {
    listener_.transformPose("base_link", pose_stamped_in, pose_stamped);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("Failed to transform gripper click grasp to %s frame; exception: %s", 
              grasp_pose.header.frame_id.c_str(), ex.what());
    setStatusLabel("Failed to transform gripper click grasp");
    return false;
  }
  grasp.grasp_pose = pose_stamped.pose;
  grasp.success_probability = 1.0;
  return true;
}

void InteractiveManipulationBackend::setStatusLabel(std::string text)
{
  if (action_server_->isActive()) 
  {
    pr2_object_manipulation_msgs::IMGUIFeedback feedback;
    feedback.status = text;
    action_server_->publishFeedback(feedback);
  }
  std_msgs::String msg;
  msg.data = text;
  status_pub_.publish(msg);
  ROS_INFO_STREAM("IM backend feedback: " << text);
}

bool InteractiveManipulationBackend::interruptRequested()
{
  return ( !priv_nh_.ok() || action_server_->isPreemptRequested() );
}

bool InteractiveManipulationBackend::checkInterrupts()
{
  if ( interruptRequested() ) throw object_manipulator::InterruptRequestedException();
  return true;
}

void populateGraspOptions(const pr2_object_manipulation_msgs::IMGUIOptions &options, 
                          object_manipulation_msgs::PickupGoal &pickup_goal)
{
  if (options.arm_selection == 0) pickup_goal.arm_name = "right_arm";
  else pickup_goal.arm_name = "left_arm";
  if ( options.adv_options.lift_direction_choice != 0)
  {
    if (pickup_goal.arm_name == "right_arm") pickup_goal.lift.direction.header.frame_id = "r_wrist_roll_link";
    else pickup_goal.lift.direction.header.frame_id = "l_wrist_roll_link";
    pickup_goal.lift.direction.vector.x = -1;
    pickup_goal.lift.direction.vector.y = 0;
    pickup_goal.lift.direction.vector.z = 0;
  }
  else
  {
    pickup_goal.lift.direction.header.frame_id = "base_link";
    pickup_goal.lift.direction.vector.x = 0;
    pickup_goal.lift.direction.vector.y = 0;
    pickup_goal.lift.direction.vector.z = 1;
  }
  pickup_goal.lift.desired_distance = options.adv_options.lift_steps * 0.01;
  pickup_goal.lift.min_distance = pickup_goal.lift.desired_distance / 2.0;
  pickup_goal.use_reactive_lift = options.adv_options.reactive_force;
  pickup_goal.use_reactive_execution = options.adv_options.reactive_grasping;
  pickup_goal.movable_obstacles = options.movable_obstacles;
  pickup_goal.max_contact_force = options.adv_options.max_contact_force;

  //using unsafe_grasp_execution
  if (!options.collision_checked)
  {
    ROS_WARN("setting ignore_collisions to true");
    pickup_goal.ignore_collisions = true;
  }
}

bool InteractiveManipulationBackend::processCollisionMapForPickup(const pr2_object_manipulation_msgs::IMGUIOptions &options,
                                                                  object_manipulation_msgs::PickupGoal &goal)
{
  if (options.collision_checked)
  {
    // make sure collision services are available
    setStatusLabel("waiting for collision map services...");
    ros::Time start_time = ros::Time::now();
    while (!collision_map_interface_.connectionsEstablished(ros::Duration(1.0)) && checkInterrupts()) 
    {
      if (ros::Time::now() - start_time >= ros::Duration(5.0))
      {
        setStatusLabel("collision map services not found");
        return false;
      }
    }
    if (!goal.target.potential_models.empty())
    {
      //if we have a potential recognized model, we will use its mesh
      try
      {
        //collision_map_interface_.processCollisionGeometryForObjectAsBoundingBox(goal.target.potential_models[0],
        //                                                           goal.collision_object_name);
	ROS_WARN("Adding full target object mesh to collision map - needed for push-grasping, but slow otherwise...");
        collision_map_interface_.processCollisionGeometryForObject(goal.target.potential_models[0],
                                                                   goal.collision_object_name);
      }
      catch (tabletop_collision_map_processing::CollisionMapException &ex)
      {
        setStatusLabel("failed to add mesh to collision map");
        return false;
      }
    }
    else if ( !goal.target.cluster.points.empty() )
    {
      //if we have a point cloud, we will use its bounding box as the collision object
      object_manipulation_msgs::ClusterBoundingBox bbox;
      try
      {
        collision_map_interface_.getClusterBoundingBox(goal.target.cluster, bbox.pose_stamped, bbox.dimensions);
        collision_map_interface_.processCollisionGeometryForBoundingBox(bbox, goal.collision_object_name);
      }
      catch (tabletop_collision_map_processing::CollisionMapException &ex)
      {
        ROS_ERROR_STREAM("Exception: " << ex.what());
        setStatusLabel("failed to compute object bounding box");
        return false;
      }
    }
    else
    {
      //if not, then the gripper can hit anything during approach and lift
      goal.collision_support_surface_name = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
      goal.allow_gripper_support_collision = true;
    }
    //also set small padding on fingertip links
    goal.additional_link_padding = object_manipulator::concat( 
                                            object_manipulator::MechanismInterface::gripperPadding("left_arm", 0.00),
                                            object_manipulator::MechanismInterface::gripperPadding("right_arm", 0.00) );
  }
  else
  {
    //if collisions are disabled, anything can hit anything
    arm_navigation_msgs::CollisionOperation coll;
    coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
    goal.additional_collision_operations.collision_operations.push_back(coll);
  }
  return true;
}

void InteractiveManipulationBackend::testGripperPoseCallback(const TestGripperPoseGoalConstPtr &goal)
{
  switch (current_action_)
  {
  case PICKUP: testGripperPoseForGraspCallback(goal); break;
  case PLACE: testGripperPoseForPlaceCallback(goal); break;
  case MOVE: testGripperPoseForMoveCallback(goal); break;
  default: ROS_ERROR("Unknown current action in testGripperPoseCallback");
  }
}

void InteractiveManipulationBackend::testGripperPoseForGraspCallback(const TestGripperPoseGoalConstPtr &goal)
{
  boost::mutex::scoped_lock lock(pipeline_mutex_);
  ROS_INFO("Request for grasp feedback received");
  pr2_object_manipulation_msgs::TestGripperPoseResult result;
  result.valid.resize(goal->gripper_poses.size(), false);

  if (goal->gripper_poses.size() != goal->gripper_openings.size())
  {
    ROS_ERROR("Size mismatch in TestGripperPose goal");
    test_gripper_pose_server_->setSucceeded(result);
  }

  current_pickup_goal_.desired_grasps.clear();
  current_pickup_goal_.only_perform_feasibility_test = true;
  for (size_t i=0; i<goal->gripper_poses.size(); i++)
  {
    object_manipulation_msgs::Grasp grasp;  
    if (!getGrasp(grasp, current_pickup_goal_.arm_name, goal->gripper_poses[i], goal->gripper_openings[i]))
    {
      setStatusLabel("Grasp test error (conversion to grasp failed)");
      test_gripper_pose_server_->setSucceeded(result);
      return;
    }
    current_pickup_goal_.desired_grasps.push_back(grasp);
  }

  setStatusLabel("Testing grasps...");
  pickup_client_.client().sendGoal(current_pickup_goal_);
  while (!pickup_client_.waitForResult(ros::Duration(1.0)) && !test_gripper_pose_server_->isPreemptRequested())
  {
    ROS_DEBUG("Waiting for pickup result..");
  }
  if (test_gripper_pose_server_->isPreemptRequested())
  {
    setStatusLabel("Grasp test canceled");
    pickup_client_.client().cancelGoal();
    test_gripper_pose_server_->setAborted(result);
    return;
  }

  object_manipulation_msgs::PickupResult pickup_result = *(pickup_client_.client().getResult());
  if (pickup_result.attempted_grasp_results.size() > result.valid.size())
  {
    ROS_ERROR("Size mismatch in list of tested grasps");
    test_gripper_pose_server_->setSucceeded(result);
  }
  bool one_good = false;
  for (size_t i=0; i<pickup_result.attempted_grasp_results.size(); i++)
  {
    if (pickup_result.attempted_grasp_results[i].result_code == object_manipulation_msgs::GraspResult::SUCCESS)
    {
      result.valid[i] = true;
      one_good = true;
    }
  }
  if (goal->gripper_poses.size() == 1)
  {
    if (pickup_result.attempted_grasp_results.empty()) ROS_ERROR("Empty list of attempted grasps in test");
    else if (result.valid[0]) setStatusLabel("grasp is good");
    else setStatusLabel(object_manipulation_msgs::getGraspResultInfo(pickup_result.attempted_grasp_results[0]));
  }
  else if (one_good) setStatusLabel("at least one good grasp found");
  else setStatusLabel("all grasps failed");

  test_gripper_pose_server_->setSucceeded(result);
}

void InteractiveManipulationBackend::testGripperPoseForPlaceCallback(const TestGripperPoseGoalConstPtr &goal)
{
  boost::mutex::scoped_lock lock(pipeline_mutex_);
  ROS_INFO("Request for place feedback received");
  pr2_object_manipulation_msgs::TestGripperPoseResult result;
  result.valid.resize(goal->gripper_poses.size(), false);
 
  current_place_goal_.place_locations.clear();
  current_place_goal_.only_perform_feasibility_test = true;
  for (size_t i=0; i<goal->gripper_poses.size(); i++)
  {
    current_place_goal_.place_locations.push_back(goal->gripper_poses[i]);
  }

  setStatusLabel("Testing places...");
  place_client_.client().sendGoal(current_place_goal_);
  while (!place_client_.waitForResult(ros::Duration(1.0)) && !test_gripper_pose_server_->isPreemptRequested())
  {
    ROS_DEBUG("Waiting for place results..");
  }
  if (test_gripper_pose_server_->isPreemptRequested())
  {
    setStatusLabel("Place test canceled");
    place_client_.client().cancelGoal();
    test_gripper_pose_server_->setAborted(result);
    return;
  }

  object_manipulation_msgs::PlaceResult place_result = *(place_client_.client().getResult());
  if (place_result.attempted_location_results.size() > result.valid.size())
  {
    ROS_ERROR("Size mismatch in list of tested places");
    test_gripper_pose_server_->setSucceeded(result);
  }
  bool one_good = false;
  for (size_t i=0; i<place_result.attempted_location_results.size(); i++)
  {
    if (place_result.attempted_location_results[i].result_code == object_manipulation_msgs::PlaceLocationResult::SUCCESS)
    {
      result.valid[i] = true;
      one_good = true;
    }
  }
  if (goal->gripper_poses.size() == 1)
  {
    if (place_result.attempted_location_results.empty()) ROS_ERROR("Empty list of attempted locations in test");
    else if (result.valid[0]) setStatusLabel("place is good");
    else setStatusLabel(object_manipulation_msgs::getPlaceLocationResultInfo(
                                                                        place_result.attempted_location_results[0]));
  }
  else if (one_good) setStatusLabel("at least one good place found");
  else setStatusLabel("all places failed");
  test_gripper_pose_server_->setSucceeded(result);
}


void InteractiveManipulationBackend::testGripperPoseForMoveCallback(const TestGripperPoseGoalConstPtr &goal)
{
  ROS_INFO("Request for move feedback received");
  pr2_object_manipulation_msgs::TestGripperPoseResult result;
  result.valid.resize(goal->gripper_poses.size(), false);

  std::string arm_name;
  if (options_.arm_selection == 0) arm_name = "right_arm";
  else arm_name = "left_arm";

  arm_navigation_msgs::OrderedCollisionOperations collision_operations;
  if (!options_.collision_checked)
  {
    //if collisions off, disable anything against anything
    arm_navigation_msgs::CollisionOperation coll;
    coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
    collision_operations.collision_operations.push_back(coll);
  }

  //check if the desired poses have a collision-free IK solution  
  std::vector<arm_navigation_msgs::LinkPadding> link_padding;
  kinematics_msgs::GetConstraintAwarePositionIK::Response ik_response;
  mech_interface_.getPlanningScene(collision_operations, link_padding);

  bool one_good = false;
  std::vector<sensor_msgs::JointState> solutions_arr;
  std::vector<arm_navigation_msgs::ArmNavigationErrorCodes> error_codes;
  ik_tester_fast_.testIKSet(arm_name, goal->gripper_poses, false, solutions_arr, error_codes);
  for (size_t i=0; i<goal->gripper_poses.size(); i++)
  {
    if(error_codes[i].val == error_codes[i].SUCCESS)
    {
      result.valid[i] = true;
      one_good = true;
    }
  }

  /*
  for (size_t i=0; i<goal->gripper_poses.size(); i++)
  {
    geometry_msgs::PoseStamped test_pose = goal->gripper_poses[i];
    test_pose.header.stamp = ros::Time::now();
    //clean up leading slash, for now
    if (!test_pose.header.frame_id.empty() && test_pose.header.frame_id.at(0)=='/')
    {
      test_pose.header.frame_id = test_pose.header.frame_id.substr(1, test_pose.header.frame_id.size()-1);
    }
    if(mech_interface_.getIKForPose(arm_name, test_pose, ik_response, collision_operations, link_padding))
    {
      result.valid[i] = true;
      one_good = true;
    }
    }*/
  if (goal->gripper_poses.size()==1)
  {
    if (result.valid[0]) setStatusLabel("Gripper pose is good");
    else if (options_.collision_checked) setStatusLabel("Gripper pose is out of reach or in collision");
    else setStatusLabel("Gripper pose is out of reach");
  }
  else
  {
    if (one_good) setStatusLabel("At least one gripper pose is good");
    else if (options_.collision_checked) setStatusLabel("All gripper poses are out of reach or in collision");
    else setStatusLabel("All gripper pose are out of reach");
  }
  test_gripper_pose_server_->setSucceeded(result);
}


int InteractiveManipulationBackend::plannedMove(const pr2_object_manipulation_msgs::IMGUIOptions &options)
{
  options_ = options;
  current_action_ = MOVE;

  std::string arm_name;
  if (options.arm_selection == 0) arm_name = "right_arm";
  else arm_name = "left_arm";

  //call ghosted gripper to get move-to location
  geometry_msgs::PoseStamped location;
  
  ROS_INFO("plannedMove called on %s", arm_name.c_str());
  int planning_result = callGhostedGripperMove(arm_name, location);
  if ( planning_result != object_manipulation_msgs::ManipulationResult::SUCCESS ) return planning_result;

  //clean up leading slash, for now
  if (!location.header.frame_id.empty() && location.header.frame_id.at(0)=='/')
  {
    location.header.frame_id = location.header.frame_id.substr(1, location.header.frame_id.size()-1);
  }
  //ask move_arm to get us there
  arm_navigation_msgs::OrderedCollisionOperations collision_operations;
  bool success;
  if (!options_.collision_checked)
  {
    //use Cartesian controllers to just move us straight there, if we're ignoring collisions
    success = mech_interface_.moveArmToPoseCartesian(arm_name, location, ros::Duration(15.0));
    /*
    //if collisions off, disable anything against anything
    arm_navigation_msgs::CollisionOperation coll;
    coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
    collision_operations.collision_operations.push_back(coll);
    */
  }
  else
  {
    std::vector<arm_navigation_msgs::LinkPadding> link_padding;
    success = mech_interface_.moveArmToPose(arm_name, location, collision_operations, link_padding);
  }
  if(success){
    setStatusLabel("planned move completed");
    return object_manipulation_msgs::ManipulationResult::SUCCESS;
  }
  setStatusLabel("planned move failed");
  return object_manipulation_msgs::ManipulationResult::FAILED;
}


int InteractiveManipulationBackend::pickupObject(const pr2_object_manipulation_msgs::IMGUIOptions &options,
                                                 object_manipulation_msgs::GraspableObject object)
{
  options_ = options;
  ROS_INFO("Graspable object has %d points and %d database models",
           (int)object.cluster.points.size(), (int)object.potential_models.size());
  //convert the graspable object to base link frame. this also sets its reference_frame_id to that
  //so any grasps returned will be in that frame
  try
  {
    mech_interface_.convertGraspableObjectComponentsToFrame(object, "base_link");
  }
  catch(object_manipulator::MechanismException &ex)
  {
    ROS_ERROR_STREAM("Conversion error: " << ex.what());
    setStatusLabel("failed to convert object to desired frame");
    return object_manipulation_msgs::ManipulationResult::ERROR;
  }

  //prepare pickup goal
  object_manipulation_msgs::PickupGoal pickup_goal;
  pickup_goal.target = object;
  if (!processCollisionMapForPickup(options, pickup_goal)) return object_manipulation_msgs::ManipulationResult::ERROR;
  populateGraspOptions(options, pickup_goal);

  //from this point on, if we get a cancellation we must clear the collision object
  try
  {
    //save this as the current pickup goal 
    pipeline_mutex_.lock();
    current_pickup_goal_ = pickup_goal;
    current_action_ = PICKUP;
    pipeline_mutex_.unlock();
    
    //if the object is empty, ask the user for help on desired grasps
    if ( object.cluster.points.empty() && object.potential_models.empty() )
    {
      object_manipulation_msgs::Grasp grasp;
      int success = callGhostedGripperPickup( pickup_goal.arm_name, grasp );
      if ( success != object_manipulation_msgs::ManipulationResult::SUCCESS ) return success;
      grasp.desired_approach_distance = options_.adv_options.desired_approach / 100.;
      grasp.min_approach_distance = options_.adv_options.min_approach / 100.;
      pickup_goal.desired_grasps.push_back(grasp);
    }
    
    //call pickup and wait for operation to finish
    pickup_goal.only_perform_feasibility_test = false;
    setStatusLabel("calling pickup action...");

    //at this point, we can lock the pipeline for good
    boost::mutex::scoped_lock lock(pipeline_mutex_);
    pickup_client_.client().sendGoal(pickup_goal, PickupClient::SimpleDoneCallback(),
                                     PickupClient::SimpleActiveCallback(), 	 
                                     boost::bind(&InteractiveManipulationBackend::pickupFeedbackCallback, this, _1));
    pickup_client_.waitForResult();
    
    //get back pickup result
    object_manipulation_msgs::PickupResult pickup_result = *(pickup_client_.client().getResult());
    
    bool pickup_succeeded = true;
    if (pickup_client_.client().getState() != actionlib::SimpleClientGoalState::SUCCEEDED) pickup_succeeded = false;
    
    if(!pickup_succeeded)
    {
      //if we failed with any result except LIFT_FAILED, in which case we are holding the object,
      //we want to get rid of if from the collision map
      if (pickup_result.manipulation_result.value != pickup_result.manipulation_result.LIFT_FAILED && 
          !pickup_goal.collision_object_name.empty())
      {
        collision_map_interface_.removeCollisionModel(pickup_goal.collision_object_name);      
      }
      if (pickup_result.attempted_grasp_results.empty())
      {
        setStatusLabel("pickup failed.");
        return object_manipulation_msgs::ManipulationResult::ERROR;
      }
      else
      {
        setStatusLabel("could not execute desired grasp");
        //setStatusLabel(object_manipulation_msgs::getGraspResultInfo(pickup_result.attempted_grasp_results[0]));
        
        //reset all attached objects for that arm, in case one was attached before the error
        mech_interface_.detachAllObjectsFromGripper(pickup_goal.arm_name);
      }
      return object_manipulation_msgs::ManipulationResult::UNFEASIBLE;
    }
    
    //save object and grasp info for later
    getGraspInfo(pickup_goal.arm_name)->object_ = object;
    getGraspInfo(pickup_goal.arm_name)->object_collision_name_ = pickup_goal.collision_object_name;
    getGraspInfo(pickup_goal.arm_name)->grasp_ = pickup_result.grasp;
    getGraspInfo(pickup_goal.arm_name)->object_orientation_ = GraspInfo::identityQuaternion();
    
    setStatusLabel("grasp completed");
    return object_manipulation_msgs::ManipulationResult::SUCCESS;
  }
  catch (InterruptRequestedException &ex)
  {
    if (!pickup_goal.collision_object_name.empty())
    {
      collision_map_interface_.removeCollisionModel(pickup_goal.collision_object_name);      
    }
    throw(ex);
  }
}

void InteractiveManipulationBackend::pickupFeedbackCallback( 	 
	                                            const object_manipulation_msgs::PickupFeedbackConstPtr &feedback) 	 
{ 	 
  std::stringstream stat; 	 
  stat << "trying grasp " << feedback->current_grasp+1 << "/" << feedback->total_grasps; 	 
  setStatusLabel(stat.str()); 	 
}

void populatePlaceOptions(object_manipulation_msgs::PlaceGoal &place_goal, const pr2_object_manipulation_msgs::IMGUIOptions &options)
{
  if (options.arm_selection == 0) place_goal.arm_name = "right_arm";
  else place_goal.arm_name = "left_arm";
  place_goal.place_padding = 0.0;
  place_goal.desired_retreat_distance = 0.1;
  place_goal.min_retreat_distance = 0.05;
  if (options.adv_options.lift_direction_choice == 0)
  {
    place_goal.approach.direction.header.frame_id = "base_link";
    place_goal.approach.direction.vector.x = 0;
    place_goal.approach.direction.vector.y = 0;
    place_goal.approach.direction.vector.z = -1;
  }
  else
  {
    if (place_goal.arm_name == "right_arm") place_goal.approach.direction.header.frame_id = "r_wrist_roll_link";
    else place_goal.approach.direction.header.frame_id = "l_wrist_roll_link";
    place_goal.approach.direction.vector.x = 1;
    place_goal.approach.direction.vector.y = 0;
    place_goal.approach.direction.vector.z = 0;
  }
  place_goal.approach.desired_distance = options.adv_options.lift_steps * 0.01;
  place_goal.approach.min_distance = place_goal.approach.desired_distance * 0.5;
  place_goal.use_reactive_place = options.adv_options.reactive_place;
}

int InteractiveManipulationBackend::placeObject(const pr2_object_manipulation_msgs::IMGUIOptions &options)
{
  //prepare place goal
  object_manipulation_msgs::PlaceGoal place_goal;
  //note identity grasp; we will be going to the gripper pose specified by the user
  place_goal.grasp.grasp_pose.orientation.w = 1;
  populatePlaceOptions(place_goal, options);
  if (!options.collision_checked)
  {
    //collisions off, disable anything against anything
    arm_navigation_msgs::CollisionOperation coll;
    coll.object1 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.object2 = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    coll.operation = arm_navigation_msgs::CollisionOperation::DISABLE;
    place_goal.additional_collision_operations.collision_operations.push_back(coll);
  }
  else
  {
    //collisions on, 0 link padding on fingertips
    place_goal.additional_link_padding = 
      object_manipulator::concat( object_manipulator::MechanismInterface::gripperPadding("left_arm", 0.0),
                                  object_manipulator::MechanismInterface::gripperPadding("right_arm", 0.0) );
    //anything can be hit during move from pre-place to place
    place_goal.collision_support_surface_name = arm_navigation_msgs::CollisionOperation::COLLISION_SET_ALL;
    place_goal.allow_gripper_support_collision = true;
    //collision name of the currently grasped object
    place_goal.collision_object_name = getGraspInfo(place_goal.arm_name)->object_collision_name_;
  }

  //save this as the current place goal
  pipeline_mutex_.lock();
  current_place_goal_ = place_goal;
  current_action_ = PLACE;
  pipeline_mutex_.unlock();

  //call gripper click to get place location
  geometry_msgs::PoseStamped location;
  int planning_result = callGhostedGripperMove(place_goal.arm_name, location);
  if ( planning_result != object_manipulation_msgs::ManipulationResult::SUCCESS ) return planning_result;
  place_goal.place_locations.push_back(location);

  //debug output
  geometry_msgs::Pose grasp_pose = place_goal.grasp.grasp_pose;
  ROS_INFO("Placing object %s on support %s using grasp: %f %f %f; %f %f %f %f", 
           place_goal.collision_object_name.c_str(), place_goal.collision_support_surface_name.c_str(),
           grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
           grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);

  //send the goal
  setStatusLabel("calling place action.");
  //at this point, we can lock the pipeline for good
  boost::mutex::scoped_lock lock(pipeline_mutex_);
  place_client_.client().sendGoal(place_goal);
  place_client_.waitForResult();
      
  //get the result
  object_manipulation_msgs::PlaceResult result = *(place_client_.client().getResult());
  if (place_client_.client().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    if (result.attempted_location_results.empty())
    {
      setStatusLabel("a serious error has occured, please call the WG helpdesk");
      return object_manipulation_msgs::ManipulationResult::ERROR;
    }
    else
    {
      setStatusLabel(object_manipulation_msgs::getPlaceLocationResultInfo(result.attempted_location_results[0]));
    }
    return object_manipulation_msgs::ManipulationResult::UNFEASIBLE;
  }
  getGraspInfo(place_goal.arm_name)->reset();
  setStatusLabel("place completed");
  return object_manipulation_msgs::ManipulationResult::SUCCESS;
}

void InteractiveManipulationBackend::collisionReset(int reset_choice, int arm_selection_choice)
{
  try
  {
    std_srvs::Empty srv;
    switch (reset_choice)
    {
    case 0:
      //reset map and all collision objects
      collision_map_interface_.resetCollisionModels();
      collision_map_interface_.resetAttachedModels();
      getGraspInfo("right_arm")->reset();
      getGraspInfo("left_arm")->reset();
      if (!collider_node_reset_srv_.client().call(srv)) setStatusLabel("failed to call map reset client");
      else setStatusLabel("collision map and all objects reset");
      break;
    case 1:
      //reset all unattached collision objects
      collision_map_interface_.resetCollisionModels();
      setStatusLabel("collision models reset");
      break;
    case 2:
      //reset all attached objects
      collision_map_interface_.resetAttachedModels();
      getGraspInfo("right_arm")->reset();
      getGraspInfo("left_arm")->reset();
      setStatusLabel("attached models reset");
      break;
    case 3:
      //reset only attached objects for the selected arm
      if(arm_selection_choice == 0){
        mech_interface_.detachAllObjectsFromGripper("right_arm");
        getGraspInfo("right_arm")->reset();
        setStatusLabel("reset models attached to right arm");
      }
      else{
        mech_interface_.detachAllObjectsFromGripper("left_arm");
        getGraspInfo("left_arm")->reset();
        setStatusLabel("reset models attached to left arm");
      }
      break;
    case 4:
      {
      //reset collision map
      if (!collider_node_reset_srv_.client().call(srv)) setStatusLabel("failed to call reset client");
      else setStatusLabel("collision map reset");
      break;
      }
    default:
      setStatusLabel("could not understand collision reset request");
    }
  }
  catch (tabletop_collision_map_processing::CollisionMapException &ex) 
  {
    setStatusLabel("failed to perform requested collision operation");
  }    
}

void InteractiveManipulationBackend::armMotion(int arm_selection_choice, int arm_action_choice, 
                                               int arm_planner_choice, bool collision,
                                               ManipulationResult &result)
{
  std::string arm_name;
  if ( arm_selection_choice == 0) arm_name = "right_arm";
  else arm_name = "left_arm";

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

    switch(arm_action_choice)
    {
    case 0: //side
      {
        if (arm_planner_choice == 0)
        {
          boost::mutex::scoped_lock lock(pipeline_mutex_);
          setStatusLabel("moving arm to side using motion planner");
          mech_interface_.clearMoveArmGoals();
          if ( !mech_interface_.attemptMoveArmToGoal(arm_name, 
                                    object_manipulator::armConfigurations().position(arm_name, "side"), ord, pad) )
          {
            result.value = result.FAILED;
            setStatusLabel("failed to move arm to side (possible collisions)");
          }
          else
          {
            setStatusLabel("arm moved to side");
            result.value = result.SUCCESS;
          }
        }
        else
        {
          setStatusLabel("moving arm to side open-loop");
          mech_interface_.attemptTrajectory(arm_name, 
                                 object_manipulator::armConfigurations().trajectory(arm_name, "side"), false, 3.0);
          setStatusLabel("arm moved to side");
          result.value = result.SUCCESS;
        }
      }
      break;
    case 1: //front
      {
        if (arm_planner_choice == 0)
        {
          boost::mutex::scoped_lock lock(pipeline_mutex_);
          mech_interface_.clearMoveArmGoals();
          setStatusLabel("moving arm to front using motion planner");
          if ( !mech_interface_.attemptMoveArmToGoal(arm_name, object_manipulator::armConfigurations().position
                                                     (arm_name, "front"), ord, pad) )
          {
            setStatusLabel("failed to move arm to front (possible collisions)");
            result.value = result.FAILED;
          }
          else
          {
            setStatusLabel("arm moved to front");
            result.value = result.SUCCESS;
          }
        }
        else
        {
          setStatusLabel("moving arm to front open-loop");
          mech_interface_.attemptTrajectory(arm_name, 
                                 object_manipulator::armConfigurations().trajectory(arm_name, "front"), false, 3.0);
          setStatusLabel("arm moved to front");
          result.value = result.SUCCESS;
        }
      }
      break;
    case 2: //side handoff
      {
        if (arm_planner_choice == 0)
        {
          boost::mutex::scoped_lock lock(pipeline_mutex_);
          mech_interface_.clearMoveArmGoals();
          setStatusLabel("moving arm to handoff using motion planner");
          if ( !mech_interface_.attemptMoveArmToGoal(arm_name, 
                                  object_manipulator::armConfigurations().position(arm_name, "handoff"), ord, pad) )
          {
            setStatusLabel("failed to move arm to handoff (possible collisions)");
            result.value = result.FAILED;
          }
          else
          {
            setStatusLabel("arm moved to handoff");
            result.value = result.SUCCESS;
          }
        }
        else
        {
          setStatusLabel("moving arm to handoff open-loop");
          mech_interface_.attemptTrajectory(arm_name, 
                                object_manipulator::armConfigurations().trajectory(arm_name, "handoff"), false, 3.0);
          setStatusLabel("arm moved to handoff");
          result.value = result.SUCCESS;
        }
      }
    default:
      setStatusLabel("unknown operation requested");
    }
  }
  catch (object_manipulator::ServiceNotFoundException &ex)
  {
    setStatusLabel("a needed service or action server was not found");
    result.value = result.ERROR;
  }
  catch (object_manipulator::MoveArmStuckException &ex)
  {
    setStatusLabel("arm is stuck in a colliding position");
    result.value = result.ERROR;
  }
  catch (object_manipulator::MissingParamException &ex)
  {
    setStatusLabel("parameters missing; is manipulation pipeline running?");
    result.value = result.ERROR;
  }
  catch (object_manipulator::MechanismException &ex)
  {
    setStatusLabel("an error has occured, please call helpdesk");
    result.value = result.ERROR;
  }
  catch (...)
  {
    setStatusLabel("an unknown error has occured, please call helpdesk");
    result.value = result.ERROR;
  }
}

void InteractiveManipulationBackend::openCloseGripper(pr2_object_manipulation_msgs::IMGUIOptions options)
{
  std::string arm_name;
  if (options.arm_selection == 0) arm_name = "right_arm";
  else arm_name = "left_arm";
  double value = gripper_client_.getGripperClosedGap(arm_name) + 
    (gripper_client_.getGripperOpenGap(arm_name) - gripper_client_.getGripperClosedGap(arm_name)) * 
    (double)(options.gripper_slider_position)/100.0;
  bool result = gripper_client_.commandGripperValue(arm_name, value);

  //if opening the gripper, reset attached objects for that arm and reset the grasp info
  if (value > gripper_client_.getGripperClosedGap(arm_name) + 
      (gripper_client_.getGripperOpenGap(arm_name) - gripper_client_.getGripperClosedGap(arm_name))/2.)
  {
    mech_interface_.detachAllObjectsFromGripper(arm_name);
    getGraspInfo(arm_name)->reset();
  }
  if (!result) setStatusLabel("failed to command gripper position");
  else setStatusLabel("sent gripper position command");
}

int InteractiveManipulationBackend::modelObject(pr2_object_manipulation_msgs::IMGUIOptions options)
{
  std::string arm_name;
  if (options.arm_selection == 0) arm_name = "right_arm";
  else arm_name = "left_arm";

  //wait for collision map services
  setStatusLabel("waiting for collision map services...");
  ros::Time start_time = ros::Time::now();
  while (!collision_map_interface_.connectionsEstablished(ros::Duration(1.0)) && checkInterrupts() ) 
  {
    if (ros::Time::now() - start_time >= ros::Duration(5.0))
    {
      setStatusLabel("collision map services not found");
      return object_manipulation_msgs::ManipulationResult::ERROR;
    }
  }

  //create and send the goal
  bool keep_level = false;
  bool snap_object_where_it_is = true;
  ROS_INFO("Modeling object in %s", arm_name.c_str());
  pr2_create_object_model::ModelObjectInHandGoal goal;
  goal.arm_name = arm_name;
  goal.keep_level = keep_level;
  if(!snap_object_where_it_is)
  {
    goal.clear_move.header.frame_id = "base_link";
    goal.clear_move.vector.z = .5;
    goal.rotate_pose.header.frame_id = "torso_lift_link";
    if(!keep_level)
    {
      goal.rotate_pose.pose.orientation.x = -0.5;
      if(arm_name == "right_arm")
      {
        goal.rotate_pose.pose.orientation.y = -0.5;
        goal.rotate_pose.pose.orientation.z = 0.5;
      }
      else
      {
        goal.rotate_pose.pose.orientation.y = 0.5;
        goal.rotate_pose.pose.orientation.z = -0.5;
      }
      goal.rotate_pose.pose.orientation.w = 0.5;
    }
    else
    {
      goal.rotate_pose = mech_interface_.getGripperPose(arm_name, "torso_lift_link");
    }
    goal.rotate_pose.pose.position.x = .65;
    goal.rotate_pose.pose.position.z = .3;

    goal.rotate_object = true;
  }
  goal.add_to_collision_map = true;
  create_model_client_.client().sendGoal(goal);
  
  setStatusLabel("calling model object in hand action...");
  create_model_client_.waitForResult();

  if (create_model_client_.client().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    setStatusLabel("modeling object in hand failed");
    return object_manipulation_msgs::ManipulationResult::ERROR;
  }
  else
  {
    pr2_create_object_model::ModelObjectInHandResult result = *create_model_client_.client().getResult();
    if(!result.cluster.data.empty())
    {
      setStatusLabel("modeling object in hand completed");
      ROS_INFO("modeled object with collision name %s",
               result.collision_name.c_str());
      object_manipulation_msgs::GraspableObject object;
      sensor_msgs::convertPointCloud2ToPointCloud( result.cluster, object.cluster );
      object.reference_frame_id = result.cluster.header.frame_id;
      if (object.reference_frame_id != "r_wrist_roll_link" && 
          object.reference_frame_id != "l_wrist_roll_link" )
      {
        ROS_ERROR_STREAM("Object model expected in gripper frame and received in frame " << object.reference_frame_id);
        setStatusLabel("unexpected frame for modeled object");
        return object_manipulation_msgs::ManipulationResult::ERROR;
      }
    
      std::string arm_name;
      //broadcast the resulting cloud to the interactive marker
      if ( options.arm_selection == 0)
      {
        arm_name = "right_arm";
        in_hand_object_right_pub_.publish(result.cluster);
      }
      else
      {
        arm_name = "left_arm";
        in_hand_object_left_pub_.publish(result.cluster);
      }
      getGraspInfo(arm_name)->object_ = object;
      //grasp is identity since object is in gripper frame
      getGraspInfo(arm_name)->grasp_.grasp_pose = GraspInfo::identityPose();
      getGraspInfo(arm_name)->object_collision_name_ = create_model_client_.client().getResult()->collision_name;
      return object_manipulation_msgs::ManipulationResult::SUCCESS;
    }
    else
    {
      setStatusLabel("modeling object in hand failed");
      return object_manipulation_msgs::ManipulationResult::ERROR;
    }
  }
}


int InteractiveManipulationBackend::runScriptedAction(std::string action_name, std::string group_name, 
                                                      pr2_object_manipulation_msgs::IMGUIOptions options)
{
    setStatusLabel("Please select a point in the point cloud.");
    pr2_object_manipulation_msgs::RunScriptGoal goal;
    goal.action_name = action_name;
    goal.group_name = group_name;

    run_script_client_.client().sendGoal(goal);
    run_script_client_.waitForResult();

    pr2_object_manipulation_msgs::RunScriptResult result = *(run_script_client_.client().getResult());
    if (run_script_client_.client().getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        setStatusLabel(result.result);
    else 
        setStatusLabel("Action failed.");
    return object_manipulation_msgs::ManipulationResult::SUCCESS;
}



int InteractiveManipulationBackend::callGhostedGripper( const GetGripperPoseGoal &goal,  GetGripperPoseResult &result)
{
  get_gripper_pose_client_.client().sendGoal(goal);
  setStatusLabel("calling ghosted gripper click...");
  get_gripper_pose_client_.waitForResult();
  if (get_gripper_pose_client_.client().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    setStatusLabel("user has canceled");
    return object_manipulation_msgs::ManipulationResult::CANCELLED;
  }
  result = *get_gripper_pose_client_.client().getResult();
  return object_manipulation_msgs::ManipulationResult::SUCCESS;  
}

int InteractiveManipulationBackend::callGhostedGripperPickup( std::string arm_name, 
                                                              object_manipulation_msgs::Grasp &grasp )
{
  GetGripperPoseResult result;
  GetGripperPoseGoal goal;
  goal.arm_name = arm_name;
  int success = callGhostedGripper(goal, result);
  if (success != object_manipulation_msgs::ManipulationResult::SUCCESS) return success;
  if (!getGrasp(grasp, arm_name, result.gripper_pose, result.gripper_opening))
  {
    ROS_DEBUG("getGrasp returned ERROR");
    return object_manipulation_msgs::ManipulationResult::ERROR;
  }
  return object_manipulation_msgs::ManipulationResult::SUCCESS;
}

int InteractiveManipulationBackend::callGhostedGripperMove( std::string arm_name, 
                                                            geometry_msgs::PoseStamped &location )
{
  GetGripperPoseResult result;
  GetGripperPoseGoal goal;
  goal.arm_name = arm_name;
  goal.object = getGraspInfo(arm_name)->object_;
  goal.grasp = getGraspInfo(arm_name)->grasp_;
  int success = callGhostedGripper(goal, result);
  if (success != object_manipulation_msgs::ManipulationResult::SUCCESS) return success;
  location = result.gripper_pose;
  return object_manipulation_msgs::ManipulationResult::SUCCESS;
}

void InteractiveManipulationBackend::imageClickCallback(const pr2_object_manipulation_msgs::ImageClickConstPtr &click)
{
  geometry_msgs::PointStamped target;
  target.header.frame_id = click->ray.header.frame_id;
  target.point.x = click->ray.origin.x + click->ray.direction.x;
  target.point.y = click->ray.origin.y + click->ray.direction.y;
  target.point.z = click->ray.origin.z + click->ray.direction.z;
  //should get the right pointing frame, maybe from a param or maybe from the message
  try
  {
    if ( !mech_interface_.pointHeadAction( target, click->camera_frame_id ) ) 
    {
      setStatusLabel( "head movement failed");
    }
    else setStatusLabel( "head movement completed");
  }
  catch (object_manipulator::ServiceNotFoundException &ex)
  {
    setStatusLabel("a needed service or action server was not found");
  }
}

void InteractiveManipulationBackend::lookAtTable()
{
  geometry_msgs::PointStamped target;
  target.point.x = 1;
  target.point.y = 0;
  target.point.z = 0;
  target.header.frame_id = "base_link";
  setStatusLabel( "moving head" );
  try
  {
    if ( !mech_interface_.pointHeadAction( target, "/narrow_stereo_optical_frame" ) ) 
    {
      setStatusLabel( "head movement failed");
    }
    else setStatusLabel( "head movement completed");
  }
  catch (object_manipulator::ServiceNotFoundException &ex)
  {
    setStatusLabel("a needed service or action server was not found");
  }
}


}

