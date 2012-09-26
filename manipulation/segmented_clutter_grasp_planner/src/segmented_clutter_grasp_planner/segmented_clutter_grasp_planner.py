#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2011, Willow Garage, Inc.
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

## @package segmented_clutter_grasp_planner
# Finds clusters in clutter, then runs the cluster planner to plan grasps on regions of interest in PointCloud2s

from __future__ import division
import roslib
roslib.load_manifest('segmented_clutter_grasp_planner')
import rospy
import actionlib
import tf
import time

from sensor_msgs.msg import CameraInfo, JointState, RegionOfInterest, PointCloud2
from object_manipulation_msgs.msg import Grasp
from object_manipulator.convert_functions import *
from object_manipulator.image_region_functions import *
from object_manipulation_msgs.srv import GraspPlanning, GraspPlanningResponse, GraspPlanningRequest
from object_manipulation_msgs.msg import GraspPlanningAction, GraspPlanningResult, GraspPlanningErrorCode
from pr2_gripper_grasp_planner_cluster.srv import SetPointClusterGraspParams, SetPointClusterGraspParamsRequest
from object_manipulation_msgs.msg import FindContainerAction, FindContainerGoal
from arm_navigation_msgs.srv import SetPlanningSceneDiff, SetPlanningSceneDiffRequest

class ClusteredClutterGraspPlanner(object):

    def __init__(self, tf_listener = None, advertise_service = False, advertise_action = True):

        #init a TF transform listener
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        #point cloud processing action
        self._container_client = actionlib.SimpleActionClient('find_container_action', FindContainerAction)
        rospy.loginfo("Waiting for find_container_action...")
        self._container_client.wait_for_server()

        #cluster grasp planning service
        service_name = "plan_point_cluster_grasp"
        rospy.loginfo("waiting for plan_point_cluster_grasp service")
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found")
        self._cluster_grasp_srv = rospy.ServiceProxy(service_name, GraspPlanning)

        #cluster grasp planning params
        service_name = "set_point_cluster_grasp_params"
        rospy.loginfo("waiting for set_point_cluster_grasp_params service")
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found")
        self._cluster_grasp_params_srv = rospy.ServiceProxy(service_name, SetPointClusterGraspParams)

        #planning scene setting service
        planning_scene_srv_name = "environment_server/set_planning_scene_diff"
        rospy.loginfo("waiting for %s service"%planning_scene_srv_name)
        rospy.wait_for_service(planning_scene_srv_name)
        self._planning_scene_srv = rospy.ServiceProxy(planning_scene_srv_name, SetPlanningSceneDiff)

        #what approximate directions should the grasps be pointing into (as with a container)?
        self._opening_dir_list = rospy.get_param("~opening_dir_list", [[0,0,1],[-1,0,0]])

        #inserted for HRI studies--if the task number is set, change the opening_dir_list
        task_number = rospy.get_param("interactive_grasping/task_number", 0)
        if task_number == 3:            
            rospy.loginfo("Task number is 3, doing front grasps only.")
            self._opening_dir_list = [[-1,0,0]]
        elif task_number > 0:
            rospy.loginfo("Task number is 1 or 2, doing overhead grasps only.")
            self._opening_dir_list = [[0,0,1]]           

        #whether to make the pregrasp just outside the cluster bounding box, or let it be the default (10 cm)
        self._pregrasp_just_outside_box = rospy.get_param("~pregrasp_just_outside_box", False)

        #name of the cluster planner service topic
        self._cluster_planner_name = rospy.get_param("cluster_planner_name", "point_cluster_grasp_planner")

        #advertise the grasp planning service
        if advertise_service:
            rospy.Service('plan_segmented_clutter_grasps', GraspPlanning, self.plan_segmented_clutter_grasps_callback)
            rospy.loginfo("segmented clutter grasp planner is ready for queries")

        #advertise the grasp planning action
        if advertise_action:
            self._as = actionlib.SimpleActionServer(rospy.get_name(), GraspPlanningAction, execute_cb = self.segmented_clutter_grasps_execute_cb)
            self._as.start()
            rospy.loginfo(rospy.get_name()+" is ready for queries")
            

    ##service callback for the plan_segmented_clutter_grasps service
    def plan_segmented_clutter_grasps_callback(self, req):
        rospy.loginfo("planning grasps for a point cloud")
        resp = GraspPlanningResponse()        
        (grasps, error_code) = self.plan_grasps_for_target(req.target, req.arm_name)
        resp.grasps = grasps
        resp.error_code = error_code
        return resp


    ##action callback for the segmented_clutter_grasps_action 
    def segmented_clutter_grasps_execute_cb(self, goal):
        result = GraspPlanningResult()
        (grasps, error_code) = self.plan_grasps_for_target(goal.target, goal.arm_name)
        result.grasps = grasps
        result.error_code = error_code
        self._as.set_succeeded(result)


    ##return grasps for either a service or action request
    def plan_grasps_for_target(self, target, arm_name):

        #what approximate directions should the grasps be pointing into (as with a container)?
        self._opening_dir_list = rospy.get_param("~opening_dir_list", [[0,0,1],[-1,0,0]])

        #plan grasps for the point cloud region (returned in the roi_box_pose frame)
        grasps = []
        for opening_dir in self._opening_dir_list:
            some_grasps = self.find_grasps(target.region.cloud, target.region.roi_box_pose, target.region.roi_box_dims, arm_name, target.reference_frame_id, opening_dir)
            grasps.extend(some_grasps)

        error_code = GraspPlanningErrorCode()
        error_code.value = error_code.SUCCESS
        if grasps == None:
            error_code.value = error_code.OTHER_ERROR
            return ([], error_code)

        #if grasps aren't in the reference_frame_id, transform them
        #if target.region.roi_box_pose.header.frame_id == target.reference_frame_id:
        #    transformed_grasps = grasps
        #else:
        #    for grasp in grasps:
        #        transformed_grasp_pose = change_pose_stamped_frame(self.tf_listener, 
        #                                 stamp_pose(grasp.grasp_pose, target.region.roi_box_pose.header.frame_id), 
        #                                 target.reference_frame_id).pose
        #        grasp.grasp_pose = transformed_grasp_pose

        return (grasps, error_code)


    ##call the find_container_action to remove parallel-to-container-side points and cluster the contents 
    def process_point_cloud(self, point_cloud, box_pose, box_dims, opening_dir):

        # x: 0 - 0.5 y: -0.3 to -0.7  z: 0.22 - 0.82
        container_goal = FindContainerGoal()
        container_goal.cloud = point_cloud
        container_goal.box_pose = box_pose
        container_goal.box_dims = box_dims
        set_xyz(container_goal.opening_dir, opening_dir)

        success = self._container_client.send_goal_and_wait(container_goal, rospy.Duration(10.0))

        if not success:
          rospy.logerr("Timed out while processing find_container_action")
          return []

        result = self._container_client.get_result()
        return result.clusters            


    #call plan_point_cluster_grasp to get candidate grasps for a cluster
    def call_plan_point_cluster_grasp(self, cluster, arm_name, reference_frame_id, side = False):

        req = GraspPlanningRequest()
        req.target.reference_frame_id = reference_frame_id
        req.target.region.cloud = cluster
        req.arm_name = arm_name   #not that cluster planner cares right now which arm
        try:
            res = self._cluster_grasp_srv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling plan_point_cluster_grasp: %s"%e)  
            return 0
        if res.error_code.value != 0:
            return []
        return res.grasps


    #call the set_point_cluster_grasp_params service with appropriate params
    def call_set_point_cluster_grasp_params(self, side = False, pregrasp_just_outside_box = False):
        
        req = SetPointClusterGraspParamsRequest()
        req.height_good_for_side_grasps = 0.02
        req.gripper_opening = 0.083
        req.side_step = 0.04
        req.palm_step = 0.02
        if side:
            req.side_grasps_only = True
            req.overhead_grasps_only = False
        else:
            req.side_grasps_only = False
            req.overhead_grasps_only = True
        req.include_high_point_grasps = False
        req.pregrasp_just_outside_box = pregrasp_just_outside_box
        req.backoff_depth_steps = 1
        req.disable_grasp_neighbor_check = True
        try:
            res = self._cluster_grasp_params_srv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling set_cluster_grasp_params: %s"%e)  
            return 0
        return 1


    #find grasps on a point cloud inside a bounding box
    def find_grasps(self, point_cloud, box_pose, box_dims, arm_name, reference_frame_id, opening_dir):
        
        start_time = time.time()

        #remove container-parallel points and cluster the contents
        clusters = self.process_point_cloud(point_cloud, box_pose, box_dims, opening_dir)
        if len(clusters) == 0:
            return []

        #whether to make the pregrasp just outside the cluster bounding box, or let it be the default (10 cm)
        self._pregrasp_just_outside_box = rospy.get_param("~pregrasp_just_outside_box", False)

        #set the params for the point cluster grasp planner
        side_opening = False
        if opening_dir[2] == 0:
            side_opening = True
        self.call_set_point_cluster_grasp_params(side_opening, self._pregrasp_just_outside_box)

        #compute grasps on all the clusters in turn
        grasp_plan_start_time = time.time()
        grasps = []
        rospy.loginfo("computing grasps on %d clusters"%len(clusters))
        for (ind, cluster) in enumerate(clusters):
            cluster_grasps = self.call_plan_point_cluster_grasp(cluster, arm_name, reference_frame_id, side_opening)
            grasps.extend(cluster_grasps)
            rospy.loginfo("%d grasps from cluster %d"%(len(cluster_grasps), ind))
        grasp_plan_end_time = time.time()
        rospy.loginfo("total cluster planning time elapsed: %.2f"%(grasp_plan_end_time-grasp_plan_start_time))

        #sort the grasps by quality
        sorted(grasps, key=lambda t:t.success_probability, reverse=True)

        #filter out grasps pointing the wrong way
        filtered_grasps = [] 
        print "%d grasps before filtering"%len(grasps)
        start_angles = [0]*7
        for grasp in grasps:

            #if side, filter out ones that are pointing the wrong way
            if side_opening:                
                grasp_mat = pose_to_mat(grasp.grasp_pose)

                ### TODO: compare to opening dir
                if grasp_mat[0,0] < 0.7:
                    continue

            filtered_grasps.append(grasp)

        end_time = time.time()
        print "total grasp planning time elapsed: %.2f"%(end_time - start_time)
        print "returning with %d grasps"%len(filtered_grasps)
        return filtered_grasps


if __name__ == '__main__':
    rospy.init_node('segmented_clutter_grasp_planner_server', anonymous=False)

    ccgps = ClusteredClutterGraspPlanner(advertise_service = False, advertise_action = True)
    rospy.spin()
