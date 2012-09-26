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

// Author(s): Kaijen Hsiao

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <pcl/io/io.h>
#include "tabletop_object_detector/SegmentObjectInHand.h"
#include "pr2_create_object_model/ModelObjectInHandAction.h"
#include "pr2_create_object_model/ObjectInHand.h"
#include "object_manipulator/tools/mechanism_interface.h"
#include "arm_navigation_msgs/LinkPadding.h"
#include "arm_navigation_msgs/OrderedCollisionOperations.h"
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <object_manipulation_msgs/ClusterBoundingBox.h>

#include <tabletop_collision_map_processing/collision_map_interface.h>

using pr2_create_object_model::ObjectInHand;

class InHandObjectModeler
{
 
private:

  //! Node handle in the root namespace
  ros::NodeHandle root_nh_;
  
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;

  //! Action server for the model_object_in_hand_action
  actionlib::SimpleActionServer<pr2_create_object_model::ModelObjectInHandAction> model_object_action_server_;

  //! Result and feedback for the action
  pr2_create_object_model::ModelObjectInHandResult model_object_result_;
  pr2_create_object_model::ModelObjectInHandFeedback model_object_feedback_;

  //! Publisher for the current object in hand point cloud
  ros::Publisher pub_cloud;
  ros::Publisher pub_pc2;

  //! Name of the segment_object_in_hand service
  std::string segment_object_service_name_;

  //! Mechanism interface for moving the arms/head
  object_manipulator::MechanismInterface mech_interface_;

  //! Collision interface for adding the object to the collision map, if requested
  tabletop_collision_map_processing::CollisionMapInterface collision_map_interface_;

  //! Which stereo camera we're using (true = narrow_stereo, false = kinect)
  bool stereo_;

public:
  
  InHandObjectModeler(std::string name) : 
    root_nh_(""),
    priv_nh_("~"),
    model_object_action_server_(root_nh_, name+"/model_object_in_hand_action", 
				boost::bind(&InHandObjectModeler::executeCB, this, _1), false)
  {
    segment_object_service_name_ = "/segment_object_in_hand_srv";

    while ( !ros::service::waitForService(segment_object_service_name_, ros::Duration().fromSec(3.0)) && priv_nh_.ok() )
    {
      ROS_INFO("Waiting for service %s...", segment_object_service_name_.c_str());
    }
    if (!priv_nh_.ok()) exit(0);

    pub_cloud = priv_nh_.advertise<ObjectInHand>("modeled_object_in_hand", 1);
    pub_pc2 = priv_nh_.advertise<sensor_msgs::PointCloud2>("modeled_object_in_hand_cloud", 1);

    priv_nh_.param<bool>("stereo", stereo_, true);

    model_object_action_server_.start();
    ROS_INFO("started action server");
  }

  //! Model object in hand action callback
  void executeCB(const pr2_create_object_model::ModelObjectInHandGoalConstPtr &goal)
  {
    sensor_msgs::PointCloud2 object_in_hand;
    sensor_msgs::PointCloud2 cluster;

    //Set the min_disparity high so we can see stuff closer to the camera
    if(stereo_)
    {
      int res = system("rosrun dynamic_reconfigure dynparam set_from_parameters /narrow_stereo_textured/narrow_stereo_textured_proc _min_disparity:=30");
    }    
    bool result;

    //broadcast the phase we're in
    model_object_feedback_.phase = model_object_feedback_.BEFORE_MOVE;
    model_object_action_server_.publishFeedback(model_object_feedback_);

    //not supposed to move the arm--just grab any points we see in the hand and return them
    if(goal->clear_move.vector.x == 0 && goal->clear_move.vector.y == 0 && goal->clear_move.vector.z == 0){
      result = callSegmentObjectInHand(goal->arm_name, cluster);
      if(result != 0) object_in_hand = cluster;
    }
    else
    {
      //move the arm up to get it away from stuff
      moveArmAway(goal->arm_name, goal->clear_move);

      //snap the current points in the hand for motion planning
      pointHead(goal->arm_name);
      ros::Duration(0.5).sleep();
      result = callSegmentObjectInHand(goal->arm_name, cluster);
      if(result != 0) object_in_hand = cluster;
      
      //if desired, move the arm to where the camera can see it
      if(goal->rotate_pose.pose.position.x != 0 || goal->rotate_pose.pose.position.y != 0 
         || goal->rotate_pose.pose.position.z != 0)
      {
        moveArmIntoView(goal->arm_name, object_in_hand, goal->rotate_pose);
      }

      //if desired, rotate the object, gathering clusters as we go 
      if(goal->rotate_object)
      { 
        //clear the old cluster first
        object_in_hand.data.clear();
        object_in_hand.width = 0;

        //accumulate a new object model while rotating
        if(goal->keep_level) turnArmInwards(goal->arm_name, object_in_hand);
        else rotateObject(goal->arm_name, object_in_hand);
      }

      //don't rotate, just snap the object where it is
      else
      {
        result = callSegmentObjectInHand(goal->arm_name, cluster);
        if(result != 0) object_in_hand = cluster;
      }
    }

    //broadcast the phase we're in
    model_object_feedback_.phase = model_object_feedback_.DONE;
    model_object_action_server_.publishFeedback(model_object_feedback_);

    //return the resulting object cluster
    model_object_result_.cluster = object_in_hand;
    std::cout << "object_in_hand width: " << object_in_hand.width << "\n";
    
    //if desired, attach the bounding box of the resulting cluster to the gripper in the collision map
    std::string collision_name;
    if (goal->add_to_collision_map)
    {
      attachObject(goal->arm_name, object_in_hand, collision_name);
    }

    //publish the final result
    ObjectInHand oih;
    oih.cluster = object_in_hand;
    oih.arm_name = goal->arm_name;
    oih.collision_name = collision_name;
    model_object_result_.collision_name = collision_name;
    ROS_INFO("frame_id: %s, final num points: %d, collision name: %s", 
	     object_in_hand.header.frame_id.c_str(), (int)object_in_hand.width, collision_name.c_str());
    pub_cloud.publish(oih);
    pub_pc2.publish(oih.cluster);

    //Set the min_disparity back
    if(stereo_)
    {
      int res = system("rosrun dynamic_reconfigure dynparam set_from_parameters /narrow_stereo_textured/narrow_stereo_textured_proc _min_disparity:=0");
    }
    model_object_action_server_.setSucceeded(model_object_result_);
  }

  //! Add the object's bounding box to the collision map and attach it to the gripper
  bool attachObject(std::string arm_name, sensor_msgs::PointCloud2 object_in_hand, std::string &collision_name)
  {
    if(object_in_hand.width == 0){
      ROS_ERROR("object_in_hand was empty!");
      return false;
    }

    try
    {
      ros::Time start_time = ros::Time::now();
      while (!collision_map_interface_.connectionsEstablished(ros::Duration(1.0))) 
      {
        if (!priv_nh_.ok() || ros::Time::now() - start_time >= ros::Duration(5.0)) 
        {
          ROS_ERROR("Create object model: failed to connect to collision map interface!");
          collision_name.clear();
          return false;
        }
      }

      //remove any current object in the hand from the collision map
      mech_interface_.detachAllObjectsFromGripper(arm_name);

      //convert the object cluster to PointCloud1
      //sensor_msgs::PointCloud cluster1;
      //sensor_msgs::convertPointCloud2ToPointCloud(object_in_hand, cluster1);
      //collision_map_interface_.processCollisionGeometryForCluster(cluster1, collision_name);
            
      //compute the cluster bounding box and add it to collision map
      object_manipulation_msgs::ClusterBoundingBox bbox;
      collision_map_interface_.getClusterBoundingBox3D(object_in_hand, bbox.pose_stamped, bbox.dimensions);
      //collision_map_interface_.getClusterBoundingBox(cluster1, bbox.pose_stamped, bbox.dimensions);
      collision_map_interface_.processCollisionGeometryForBoundingBox(bbox, collision_name);
      
      //give the cluster collision object time to instantiate
      ros::Duration(1.0).sleep();

      //attach the box to the gripper
      mech_interface_.attachObjectToGripper(arm_name, collision_name);
    }
    catch(...)
    {
      ROS_ERROR("Error while trying to add object to collision map");
      collision_name.clear();
      return false;
    }
    return true;
  }


  //! Call the segment_object_in_hand_srv service
  bool callSegmentObjectInHand(std::string arm_name, sensor_msgs::PointCloud2 &object_in_hand)
  {
    tabletop_object_detector::SegmentObjectInHand segmentation_srv;
    if (arm_name.compare("right_arm") == 0)
      segmentation_srv.request.wrist_frame = std::string("r_wrist_roll_link");
    else
      segmentation_srv.request.wrist_frame = std::string("l_wrist_roll_link");
    if (!ros::service::call(segment_object_service_name_, segmentation_srv))
    {
      ROS_ERROR("Call to segmentation service failed");
      return 0;
    }
    object_in_hand = segmentation_srv.response.cluster;
    return 1;
  }

  //! Point the head at what's in the gripper
  void pointHead(std::string arm_name)
  {
    geometry_msgs::PointStamped desired_head_point;
    if (arm_name.compare("right_arm") == 0)
      desired_head_point.header.frame_id = "r_wrist_roll_link";
    else
      desired_head_point.header.frame_id = "l_wrist_roll_link";
    desired_head_point.point.x = .2;
    if(stereo_) mech_interface_.pointHeadAction(desired_head_point, "/narrow_stereo_optical_frame");
    else mech_interface_.pointHeadAction(desired_head_point, "/openni_rgb_optical_frame");
    /* else
    {
      geometry_msgs::PoseStamped gripper_pose = mech_interface_.getGripperPose(arm_name, std::string("torso_lift_link"));
      desired_head_point.header.frame_id = "torso_lift_link";
      desired_head_point.point.x = gripper_pose.pose.position.x;
      desired_head_point.point.y = gripper_pose.pose.position.y;
      desired_head_point.point.z = gripper_pose.pose.position.z;
      mech_interface_.pointHeadAction(desired_head_point, "/openni_rgb_optical_frame");
    }*/
    ROS_INFO("pointing head");
  }

  //! Move the arm to a fixed position in front of the head (keep old orientation)
  bool moveArmIntoView(std::string arm_name, sensor_msgs::PointCloud2 object_in_hand, geometry_msgs::PoseStamped desired_pose)
  {
    /*
    geometry_msgs::PoseStamped desired_pose = mech_interface_.getGripperPose(arm_name, std::string("torso_lift_link"));
    desired_pose.pose.position.x = .7;
    desired_pose.pose.position.y = 0;
    desired_pose.pose.position.z = 0.40;
    */

    //broadcast the phase we're in
    model_object_feedback_.phase = model_object_feedback_.MOVE_TO_ROTATE_POSE;
    model_object_action_server_.publishFeedback(model_object_feedback_);

    //if there are any points in object_in_hand, attach them to the gripper
    std::string collision_name;
    bool result = false;
    attachObject(arm_name, object_in_hand, collision_name);
    
    // //try to get to the desired pose using move_arm
    // const arm_navigation_msgs::OrderedCollisionOperations col_opers;
    // std::vector<arm_navigation_msgs::LinkPadding> link_padding;
    // bool result = mech_interface_.moveArmToPose(arm_name, desired_pose, col_opers, link_padding);

    //detach the object from the gripper
    mech_interface_.detachAllObjectsFromGripper(arm_name);

    //use Cartesian controllers to go straight there in the event of failure (and raise the elbow)
    if(!result)
    {
      geometry_msgs::PoseStamped current_pose = mech_interface_.getGripperPose(arm_name, desired_pose.header.frame_id);
      double pos_dist, angle;
      mech_interface_.poseDists(current_pose.pose, desired_pose.pose, pos_dist, angle);
      double time = pos_dist / .10;
      double angle_time = angle / .785;
      if (angle_time > time) time = angle_time;
      const double left_elbow_up_angles[7] = {0.79,0,1.6,9999,9999,9999,9999};
      const double right_elbow_up_angles[7] = {-0.79,0,-1.6,9999,9999,9999,9999};
      std::vector<double> left_elbow_up_angles_vect(left_elbow_up_angles, left_elbow_up_angles+7); 
      std::vector<double> right_elbow_up_angles_vect(right_elbow_up_angles, right_elbow_up_angles+7);
      if(arm_name == "left_arm") mech_interface_.sendCartesianPostureCommand(arm_name, left_elbow_up_angles_vect);
      else mech_interface_.sendCartesianPostureCommand(arm_name, right_elbow_up_angles_vect);
      mech_interface_.moveArmToPoseCartesian(arm_name, desired_pose, ros::Duration(time + 2.0), .015, .09, .04, .3, .1);
    }
    ROS_INFO("moving arm in front of head");

    //point the head at the gripper and wait for it to get there
    pointHead(arm_name);
    ros::Duration(0.5).sleep();

    return 1;
  }

  //! Rotate the last arm joint, not trying to keep the object level
  bool rotateObject(std::string arm_name, sensor_msgs::PointCloud2 &object_in_hand)
  {
    sensor_msgs::PointCloud2 cluster;
    ROS_INFO("inside rotateObject");
    
    //get the current arm angles
    std::vector<double> current_arm_angles;
    mech_interface_.getArmAngles(arm_name, current_arm_angles);

    //rotate the gripper by turning the last joint
    double max_angle_step = M_PI/2;
    int num_steps = 2*M_PI/max_angle_step;
    double step_size = 2*M_PI/num_steps;
    for(int view_ind=0; view_ind<num_steps; view_ind++)
    {
      //ask the arm to move to the new joint angles (except for the first step, which is the current pose)
      if(view_ind != 0)
      {
        std::vector< std::vector<double> > trajectory;
        trajectory.push_back(current_arm_angles);
        current_arm_angles[6] += step_size;
        trajectory.push_back(current_arm_angles);
        mech_interface_.attemptTrajectory(arm_name, trajectory, false, 0.75);
      }

      //point the head at the object in the gripper
      pointHead(arm_name);
      //ros::Duration(1.0).sleep();

      //broadcast the step we're on
      model_object_feedback_.phase = model_object_feedback_.ROTATING;
      model_object_feedback_.rotate_ind = view_ind;
      model_object_action_server_.publishFeedback(model_object_feedback_);      
      
      //get the cluster of points in the hand
      bool result = callSegmentObjectInHand(arm_name, cluster);
      if(result != 0)
      {
        //combine resulting point cloud into object model
        if(object_in_hand.width == 0) object_in_hand = cluster;
        else
        {
          result = pcl::concatenatePointCloud(object_in_hand, cluster, object_in_hand);
          object_in_hand.header.stamp = ros::Time::now();
        }
        
        //publish the new result
        ROS_INFO("frame_id: %s, num points: %d", object_in_hand.header.frame_id.c_str(), (int)object_in_hand.width);
        //pub_pc2.publish(object_in_hand);
      }
    }
    return 1;
  }

  //! Turn the gripper so it's facing towards the robot, keeping the gripper level
  bool turnArmInwards(std::string arm_name, sensor_msgs::PointCloud2 &object_in_hand)
  {

    geometry_msgs::PoseStamped current_pose = mech_interface_.getGripperPose(arm_name, "torso_lift_link");
    Eigen::Affine3d current_trans;
    tf::poseMsgToEigen(current_pose.pose, current_trans);
    Eigen::Affine3d desired_trans;

    int num_steps = 0;
    double rot_angle;
    std::vector<geometry_msgs::PoseStamped> desired_poses;
    Eigen::Vector3d z(0,0,1);

    //need to keep the object level--compute desired poses that try to turn the gripper inwards
    //find the angle that rotates the current gripper x-axis to the torso_lift_link x-z plane, pointed in the -x direction
    //but make the gripper swing towards the center 
    //(for the right arm, -PI/4 moves +3PI/4, left arm reversed)
    Eigen::Vector3d x_axis = current_trans.rotation().col(0);
    rot_angle = atan2(x_axis[1], -x_axis[0]);
    //std::cout << "rot_angle before:" << rot_angle << std::endl;
    if(arm_name.compare("right_arm") == 0 && rot_angle < -M_PI/4) rot_angle += 2*M_PI;
    if(arm_name.compare("left_arm") == 0 && rot_angle > M_PI/4) rot_angle -= 2*M_PI; 
    //std::cout << "x_axis:\n" << x_axis << std::endl;
    std::cout << "rot_angle after:" << rot_angle << std::endl;

    //compute the number of steps based on the max angle step to move before trying to snap a point cloud
    double max_angle_step = M_PI/2;
    num_steps = ceil(fabs(rot_angle) / max_angle_step) + 1;
    //std::cout << "num_steps:" << num_steps << std::endl;

    //compute the sequence of desired poses at which to snap point clouds by interpolating
    Eigen::Affine3d step_trans;	
    geometry_msgs::PoseStamped desired_pose;
    for(int view_ind=0; view_ind<num_steps; view_ind++)	 
    {
      double frac;
      if(num_steps == 1) frac = 1;
      else frac = view_ind / float(num_steps-1);
      //std::cout << "frac:" << frac << std::endl;
      Eigen::AngleAxis<double> step_rotation(rot_angle*frac, z);
      step_trans = step_rotation * current_trans;
      for(int i=0; i<3; i++) step_trans(i,3) = current_trans(i,3);
      tf::poseEigenToMsg(step_trans, desired_pose.pose);      
      desired_pose.header.frame_id = "torso_lift_link";
      desired_pose.header.stamp = ros::Time::now();
      desired_poses.push_back(desired_pose);
      //std::cout << "desired pose:" << desired_pose << std::endl;
    }
    Eigen::Affine3d towards_trans = step_trans;
    //now go PI towards the center if we haven't gone that far already
    if(fabs(rot_angle) < M_PI)
    {
      double additional_angle = -M_PI;
      if(arm_name.compare("left_arm") == 0) additional_angle = M_PI;
      int num_add_steps = ceil(fabs(additional_angle) / max_angle_step) + 1;
      for(double step=0; step<num_add_steps; step++)
      {
        double frac = step / float(num_add_steps-1);
        Eigen::AngleAxis<double> step_rotation(additional_angle*frac, z);
        step_trans = step_rotation * towards_trans;
        for(int i=0; i<3; i++) step_trans(i,3) = towards_trans(i,3);
        tf::poseEigenToMsg(step_trans, desired_pose.pose);      
        desired_pose.header.frame_id = "torso_lift_link";
        desired_pose.header.stamp = ros::Time::now();
        desired_poses.push_back(desired_pose);
      } 
    }

    //Gather points from multiple viewpoints while turning the object
    bool result;
    sensor_msgs::PointCloud2 cluster;
    for(size_t view_ind=0; view_ind<desired_poses.size(); view_ind++){
      
      //move the arm to the next viewpoint
      //std::cout << "moving to desired pose:" << desired_poses[view_ind] << std::endl;
      mech_interface_.moveArmToPoseCartesian(arm_name, desired_poses[view_ind], 
                                             ros::Duration(1.0), .015, .09, .02, .16, .1);
      
      //point the head
      pointHead(arm_name);
      //ros::Duration(1.0).sleep();

      //broadcast the step we're on
      model_object_feedback_.phase = model_object_feedback_.ROTATING;
      model_object_feedback_.rotate_ind = view_ind;
      model_object_action_server_.publishFeedback(model_object_feedback_);
      
      //get the cluster of points in the hand
      result = callSegmentObjectInHand(arm_name, cluster);
      if(result != 0)
      {
        //combine resulting point cloud into object model
        if(object_in_hand.width == 0) object_in_hand = cluster;
        else
        {
          result = pcl::concatenatePointCloud(object_in_hand, cluster, object_in_hand);
          ROS_INFO("concatenate result: %d", result);
          object_in_hand.header.stamp = ros::Time::now();
        }
        
        //publish the new result
        ROS_INFO("frame_id: %s, num points: %d", object_in_hand.header.frame_id.c_str(), (int)object_in_hand.width);
        //pub_pc2.publish(object_in_hand);
      }
    }
    
    return 1;
  }


  //! Move the arm to get it away from stuff
  bool moveArmAway(std::string arm_name, geometry_msgs::Vector3Stamped clear_move)
  {
    //broadcast the phase we're in
    model_object_feedback_.phase = model_object_feedback_.CLEAR_MOVE;
    model_object_action_server_.publishFeedback(model_object_feedback_);

    mech_interface_.translateGripperCartesian(arm_name, clear_move, ros::Duration(5.0), .015, .09, .02, .16, .1);
    return 1;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "create_object_model_server");
  InHandObjectModeler node(ros::this_node::getName());
  ROS_INFO("create object model action server ready");
  ros::spin();
  return 0;
}
