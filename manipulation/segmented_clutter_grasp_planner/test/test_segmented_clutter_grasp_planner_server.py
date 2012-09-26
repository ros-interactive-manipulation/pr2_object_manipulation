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
# Test script for the segmented_clutter_grasp_planner_server

from __future__ import division
import roslib
roslib.load_manifest('segmented_clutter_grasp_planner')
import rospy
from object_manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
import object_manipulator.draw_functions as draw_functions
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Vector3


#call plan_point_cluster_grasp to get candidate grasps for a cluster
def call_plan_segmented_clutter_grasps(point_cloud, box_pose, box_dims, arm_name):
    
    req = GraspPlanningRequest()
    req.arm_name = arm_name
    req.target.reference_frame_id = box_pose.header.frame_id
    req.target.region.cloud = point_cloud
    req.target.region.roi_box_pose = box_pose
    req.target.region.roi_box_dims = box_dims
    
    service_name = "plan_segmented_clutter_grasps"
    rospy.loginfo("waiting for %s"%service_name)
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, GraspPlanning)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling plan_segmented_clutter_grasps: %s"%e)  
        return 0
    if res.error_code.value != 0:
        return []
    
    return res.grasps


if __name__ == "__main__":
    #initialize the node, tf listener and broadcaster, and rviz drawing helper class
    rospy.init_node('test_point_cluster_grasp_planner', anonymous=True)
    draw_functions = draw_functions.DrawFunctions('grasp_markers')

    #set the region to be grocery bag, shelf, or table
    mode = 'table'
    if mode == 'shelf':
        rospy.set_param("segmented_clutter_grasp_planner_server/side_opening", True)
    else:
        rospy.set_param("segmented_clutter_grasp_planner_server/side_opening", False)

    #test asking the pregrasp to be just above the object's bounding box
    rospy.set_param("segmented_clutter_grasp_planner_server/pregrasp_just_outside_box", True)

    #using the right arm
    arm_name = "right_arm"

    #keep going until the user says to quit (or until Ctrl-C is pressed)
    while(not rospy.is_shutdown()):

        cloud_topic = "/camera/rgb/points"
        #cloud_topic = "/narrow_stereo_textured/points2"

        #grab a point cloud from cloud_topic
        print "waiting for point cloud"
        point_cloud = rospy.wait_for_message(cloud_topic, PointCloud2)
        print "point cloud received"

        #set the bounding region 
        box_pose = PoseStamped()
        box_dims = Vector3()
        box_pose.header.frame_id = "/base_link";

        # (grocery bag to the front right side of robot) x: 0 to 0.5 y: -0.3 to -0.7  z: 0.22 to 1
        if mode == 'grocery bag':
            box_pose.pose.position.x = 0.25
            box_pose.pose.position.y = -0.5
            box_pose.pose.position.z = 0.53
            box_dims.x = 0.5
            box_dims.y = 0.4
            box_dims.z = 0.6

        # (right-arm reachable table in front of robot) x: .41 to .84 y: -0.38 to +0.18  z: 0.68 to 1.0
        elif mode == 'table':
            box_pose.pose.position.x = (.41+.84)/2.
            box_pose.pose.position.y = (-.38+.18)/2.
            box_pose.pose.position.z = (.68+1.0)/2.
            box_dims.x = .84-.41
            box_dims.y = .18+.38;
            box_dims.z = 1.0-.68;

        # (right-arm reachable shelf in front of robot) x: .42 to .84 y: -0.17 to +0.07  z: 0.70 to 1.247
        else:
            box_pose.pose.position.x = (.42+.84)/2.
            box_pose.pose.position.y = (-.17+.07)/2.
            box_pose.pose.position.z = (.70+1.247)/2.
            box_dims.x = .84-.42
            box_dims.y = .17+.07
            box_dims.z = 1.247-.70

        #plan grasps on the point cloud in that area
        grasps = call_plan_segmented_clutter_grasps(point_cloud, box_pose, box_dims, arm_name)

        #draw the resulting grasps (all at once, or one at a time)
        print "number of grasps returned:", len(grasps)
        grasp_poses = [grasp.grasp_pose for grasp in grasps]
        draw_functions.draw_grasps(grasp_poses, box_pose.header.frame_id, pause = 0)
        print "done drawing grasps, press enter to continue"
        raw_input()

        #clear out the grasps drawn
        draw_functions.clear_grasps(num = 1000)

        print "Run again? Enter to run, q to exit"
        c = raw_input()
        if c == 'q':
            break


    
