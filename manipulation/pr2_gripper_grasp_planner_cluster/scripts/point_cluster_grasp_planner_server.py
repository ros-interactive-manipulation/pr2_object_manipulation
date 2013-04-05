#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Kaijen Hsiao

## @package pr2_gripper_grasp_planner_cluster_server
# Server for the point_cluster_grasp_planner

import roslib
roslib.load_manifest('pr2_gripper_grasp_planner_cluster')
import rospy
import actionlib
from manipulation_msgs.srv import GraspPlanning, GraspPlanningResponse
from manipulation_msgs.msg import Grasp, GraspPlanningAction, GraspPlanningErrorCode, GraspPlanningResult, GripperTranslation
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsResponse
import pr2_gripper_grasp_planner_cluster.point_cluster_grasp_planner as grasp_planner_cluster
from sensor_msgs.msg import JointState
import random
import pdb
from object_manipulator.convert_functions import *
import time

##class for the point cluster grasp planner service
class PointClusterGraspPlannerServer:

    def __init__(self):
        
        self.pcgp = grasp_planner_cluster.PointClusterGraspPlanner()

        #advertise service and action for planning grasps
        rospy.Service('plan_point_cluster_grasp', GraspPlanning, self.plan_point_cluster_grasp_callback)
        self._as = actionlib.SimpleActionServer('plan_point_cluster_grasp', GraspPlanningAction, 
                                                execute_cb = self.plan_point_cluster_grasp_execute_cb)

        #advertise service for evaluating grasps
        rospy.Service('evaluate_point_cluster_grasps', GraspPlanning, self.evaluate_point_cluster_grasps_callback) 

        #advertise service for changing params
        rospy.Service('set_point_cluster_grasp_params', SetPointClusterGraspParams, self.set_point_cluster_grasp_params_callback)

        #param to randomize grasps (generally a bad idea, unless you actually want bad grasps.)
        self.randomize_grasps = rospy.get_param("~randomize_grasps", 0)
        rospy.loginfo("randomize_grasps:"+str(self.randomize_grasps))
        random.seed()

        #start the action server
        self._as.start()
        
        rospy.loginfo("point cluster grasp planner service/action is ready")


    ##service callback for changing params
    def set_point_cluster_grasp_params_callback(self, req):
        self.pcgp.height_good_for_side_grasps = req.height_good_for_side_grasps
        self.pcgp.gripper_opening = req.gripper_opening
        self.pcgp.side_step = req.side_step
        self.pcgp.palm_step = req.palm_step
        self.pcgp.overhead_grasps_only = req.overhead_grasps_only
        self.pcgp.side_grasps_only = req.side_grasps_only
        self.pcgp.include_high_point_grasps = req.include_high_point_grasps
        self.pcgp.pregrasp_just_outside_box = req.pregrasp_just_outside_box
        self.pcgp.backoff_depth_steps = req.backoff_depth_steps
        if self.pcgp.backoff_depth_steps < 1:
            self.pcgp.backoff_depth_steps = 1
        self.pcgp.disable_grasp_neighbor_check = req.disable_grasp_neighbor_check
        self.randomize_grasps = req.randomize_grasps
        resp = SetPointClusterGraspParamsResponse()
        return resp


    ##service callback for the evaluate_point_cluster_grasps service
    def evaluate_point_cluster_grasps_callback(self, req):
        #rospy.loginfo("evaluating grasps for a point cluster")
        
        #find the cluster bounding box and relevant frames, and transform the cluster
        if len(req.target.cluster.points) > 0:
            self.pcgp.init_cluster_grasper(req.target.cluster)
            cluster_frame = req.target.cluster.header.frame_id
        else:
            self.pcgp.init_cluster_grasper(req.target.region.cloud)
            cluster_frame = req.target.region.cloud.header.frame_id

        #evaluate the grasps on the cluster
        probs = self.pcgp.evaluate_point_cluster_grasps(req.grasps_to_evaluate, cluster_frame)

        #return the same grasps with the qualities added
        for (grasp, prob) in zip(req.grasps_to_evaluate, probs):
            grasp.success_probability = prob

        #fill in the response
        resp = GraspPlanningResponse()
        resp.error_code.value = 0
        resp.grasps = req.grasps_to_evaluate
        
        return resp


    def create_joint_state(self, hand_joints, position, effort):
        js = JointState()
        js.name = hand_joints
        js.position = position
        js.effort = effort
        return js


    ##action callback for the plan_point_cluster_grasp action
    def plan_point_cluster_grasp_execute_cb(self, goal):
        rospy.loginfo("planning grasps for a point cluster")
        result = GraspPlanningResult()
        (grasps, error_code) = self.plan_point_cluster_grasps(goal.target, goal.arm_name)
        result.grasps = grasps
        result.error_code = error_code
        self._as.set_succeeded(result)


    ##service callback for the plan_point_cluster_grasp service
    def plan_point_cluster_grasp_callback(self, req):
        rospy.loginfo("planning grasps for a point cluster")
        resp = GraspPlanningResponse()        
        (grasps, error_code) = self.plan_point_cluster_grasps(req.target, req.arm_name)
        resp.grasps = grasps
        resp.error_code = error_code
        return resp


    def plan_point_cluster_grasps(self, target, arm_name):
        error_code = GraspPlanningErrorCode()
        grasps = []

        #get the hand joint names from the param server (loaded from yaml config file)
        hand_description = rospy.get_param('hand_description')
        pregrasp_joint_angles_dict = rospy.get_param('~pregrasp_joint_angles')
        grasp_joint_angles_dict = rospy.get_param('~grasp_joint_angles')
        pregrasp_joint_efforts_dict = rospy.get_param('~pregrasp_joint_efforts')
        grasp_joint_efforts_dict = rospy.get_param('~grasp_joint_efforts')
        if not arm_name:
            arm_name = hand_description.keys()[0]
            rospy.logerr("point cluster grasp planner: missing arm_name in request!  Using "+arm_name)
        try:
            hand_joints = hand_description[arm_name]["hand_joints"]
        except KeyError:
            arm_name = hand_description.keys()[0]
            rospy.logerr("arm_name "+arm_name+" not found!  Using joint names from "+arm_name)
            try:
                hand_joints = hand_description[arm_name]["hand_joints"]
            except KeyError:
                rospy.logerr("no hand joints found for %s!"%arm_name)
                return ([], error_code.OTHER_ERROR)
        try:
            hand_frame = hand_description[arm_name]["hand_frame"]
            hand_approach_direction = hand_description[arm_name]["hand_approach_direction"]
        except KeyError:
            rospy.logerr("couldn't find hand_frame or hand_approach_direction!")
            return ([], error_code.OTHER_ERROR)
        pregrasp_joint_angles = pregrasp_joint_angles_dict[arm_name]
        grasp_joint_angles = grasp_joint_angles_dict[arm_name]
        pregrasp_joint_efforts = pregrasp_joint_efforts_dict[arm_name]
        grasp_joint_efforts = grasp_joint_efforts_dict[arm_name]

        #hand_joints = rospy.get_param('/hand_description/'+arm_name+'/hand_joints')
        rospy.loginfo("hand_joints:"+str(hand_joints))

        #find the cluster bounding box and relevant frames, and transform the cluster
        init_start_time = time.time()
        if len(target.cluster.points) > 0:
            self.pcgp.init_cluster_grasper(target.cluster)
            cluster_frame = target.cluster.header.frame_id
        else:
            self.pcgp.init_cluster_grasper(target.region.cloud)
            cluster_frame = target.region.cloud.header.frame_id
            if len(cluster_frame) == 0:
                rospy.logerr("region.cloud.header.frame_id was empty!")
                error_code.value = error_code.OTHER_ERROR
                return (grasps, error_code)
        init_end_time = time.time()
        #print "init time: %.3f"%(init_end_time - init_start_time)

        #plan grasps for the cluster (returned in the cluster frame)
        grasp_plan_start_time = time.time()
        (pregrasp_poses, grasp_poses, gripper_openings, qualities, pregrasp_dists) = self.pcgp.plan_point_cluster_grasps()
        grasp_plan_end_time = time.time()
        #print "total grasp planning time: %.3f"%(grasp_plan_end_time - grasp_plan_start_time)

        #add error code to service
        error_code.value = error_code.SUCCESS
        grasp_list = []
        if pregrasp_poses == None:
            error_code.value = error_code.OTHER_ERROR
            return (grasps, error_code)

        #fill in the list of grasps
        for (grasp_pose, quality, pregrasp_dist) in zip(grasp_poses, qualities, pregrasp_dists):
            pre_grasp_joint_state = self.create_joint_state(hand_joints, pregrasp_joint_angles, pregrasp_joint_efforts)
            grasp_joint_state = self.create_joint_state(hand_joints, grasp_joint_angles, grasp_joint_efforts)

            #if the cluster isn't in the same frame as the graspable object reference frame,
            #transform the grasps to be in the reference frame
            if cluster_frame == target.reference_frame_id:
                transformed_grasp_pose = stamp_pose(grasp_pose, cluster_frame)
            else:
                transformed_grasp_pose = change_pose_stamped_frame(self.pcgp.tf_listener, 
                                         stamp_pose(grasp_pose, cluster_frame), 
                                         target.reference_frame_id)
            if self.pcgp.pregrasp_just_outside_box:
                min_approach_distance = pregrasp_dist
            else:
                min_approach_distance = max(pregrasp_dist-.05, .05)
            approach = GripperTranslation(create_vector3_stamped(hand_approach_direction, hand_frame), pregrasp_dist, min_approach_distance)
            retreat = GripperTranslation(create_vector3_stamped([-1.*x for x in hand_approach_direction], hand_frame), pregrasp_dist, min_approach_distance)
            grasp_list.append(Grasp(id="id", pre_grasp_posture=pre_grasp_joint_state, grasp_posture=grasp_joint_state, 
                                    grasp_pose=transformed_grasp_pose, grasp_quality=quality, 
                                    approach=approach, retreat=retreat, max_contact_force=-1))

        #if requested, randomize the first few grasps
        if self.randomize_grasps:
            first_grasps = grasp_list[:30]
            random.shuffle(first_grasps)
            shuffled_grasp_list = first_grasps + grasp_list[30:]
            grasps = shuffled_grasp_list
        else:
            grasps = grasp_list

        return (grasps, error_code)


if __name__ == '__main__':
    rospy.init_node('point_cluster_grasp_planner', anonymous=False)

    pcgps = PointClusterGraspPlannerServer()
    
    rospy.loginfo("point cluster grasp planner is ready for queries")
    rospy.spin()
