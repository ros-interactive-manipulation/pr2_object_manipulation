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

## @package pr2_gripper_grasp_planner_cluster
#Finds candidate grasps for point cloud clusters by finding the principal directions of the 
#cloud and searching for orthogonal grasps (and a few extra grasps of high points)

from __future__ import division
import roslib
roslib.load_manifest('pr2_gripper_grasp_planner_cluster')
import rospy
import scipy
import pdb
import random
import math
import scipy.linalg
from geometry_msgs.msg import PoseStamped, Point, Pose, Vector3
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from interpolated_ik_motion_planner import ik_utilities
import tf.transformations
from object_manipulator.convert_functions import *
import object_manipulator.draw_functions as draw_functions
import object_manipulator.cluster_bounding_box_finder as cluster_bounding_box_finder
import time

'''
some dimensions:
from r_wrist_roll_link to r_gripper_l_finger_tip_link: .154, .059  (-.059 to r_finger_tip_link)
from fingertip tip link to tip of fingertip is 3.1 cm
from fingertip tip link to front of fingertip is 1.5 cm
'''

##Class for doing grasp planning on point clusters
class PointClusterGraspPlanner:

    def __init__(self, tf_listener = None, tf_broadcaster = None): 

        #init a TF transform listener
        if tf_listener == None:
            self.tf_listener = tf.TransformListener()
        else:
            self.tf_listener = tf_listener

        #init a TF transform broadcaster for the object frame
        if tf_broadcaster == None:
            self.tf_broadcaster = tf.TransformBroadcaster()
        else:
            self.tf_broadcaster = tf_broadcaster

        #init a FindClusterBoundingBox object
        self.cbbf = cluster_bounding_box_finder.ClusterBoundingBoxFinder(self.tf_listener, self.tf_broadcaster)

        #draw_functions object for drawing stuff in rviz
        self.draw_functions = draw_functions.DrawFunctions('point_cluster_grasp_planner_markers')

        #publisher for displaying gripper boxes in rviz
        #self.marker_pub = rospy.Publisher('point_cluster_grasp_planner_markers', Marker)

        #keep track of the ids used for various markers
        self.point_id_start = 1000
        self.collision_point_id_start = 5000
        self.gripper_box_id_start = 1

        #average number of points per sq. cm of bounding box surface
        self._points_per_sq_cm = 5.

        #params for the gripper model/grasp search

        #the axis-aligned model of the gripper for checking for collisions and points in the hand
        self.gripper_boxes = rospy.get_param("~gripper_boxes", None)
        self.space_boxes = rospy.get_param("~space_boxes", None)
        if self.gripper_boxes == None or self.space_boxes == None:
            (self.gripper_boxes, self.space_boxes) = self.gripper_model()

        #the actual desired wrist frame of the robot (4x4, column-order) in the gripper model frame
        actual_wrist_frame_in_model_frame = rospy.get_param("~actual_wrist_frame_in_model_frame", None)
        if actual_wrist_frame_in_model_frame == None:
            self.actual_wrist_frame_in_model_frame = scipy.identity(4)
            rospy.logerr("no actual_wrist_frame_in_model_frame!  Using identity!")
        else:
            self.actual_wrist_frame_in_model_frame = scipy.matrix(actual_wrist_frame_in_model_frame).transpose()
        self.model_frame_in_actual_wrist_frame = self.actual_wrist_frame_in_model_frame**-1

        #default pregrasp dist away from grasp (overridden if pregrasp_just_outside_box is True)
        self.pregrasp_dist = rospy.get_param("~default_pregrasp_dist", .10)
        
        #minimum number of points to be declared a good grasp
        self.min_good_grasp_points = rospy.get_param("min_good_grasp_points", 15)

        #distance from the wrist frame to the center of the fingertips (along the gripper x-axis)
        self._wrist_to_fingertip_center_dist = rospy.get_param("~wrist_to_fingertip_center_dist", .185)

        #distance from the wrist frame to a safe distance barely past the surface of the palm (along the gripper x-axis)
        self._wrist_to_palm_dist = rospy.get_param("~wrist_to_palm_dist", .143)

        #bounding box "fits in hand" if the relevant dimension is less than this
        self.gripper_opening = rospy.get_param("~gripper_opening", .083)                  

        #only want side grasps if the bounding box height is greater than this
        self.height_good_for_side_grasps = rospy.get_param("~height_good_for_side_grasps", .05)       

        #start the side grasp search at this height or at the center of the object, whichever is higher
        self.side_grasp_start_height = rospy.get_param("~side_grasp_start_height", .04)

        #how far to move the gripper to the side with each step when searching for grasps
        self.side_step = rospy.get_param("~side_step", .02)     
        
        #how far to move the palm inward with each step when searching for grasps
        self.palm_step = rospy.get_param("~palm_step", .005)   

        #set this to true to limit the planner to overhead grasps
        self.overhead_grasps_only = rospy.get_param("~overhead_grasps_only", False)

        #set this to true to limit the planner to side grasps
        self.side_grasps_only = rospy.get_param("~side_grasps_only", False)

        #set this to false to omit random high-point grasps
        self.include_high_point_grasps = rospy.get_param("~include_high_point_grasps", True)

        #set this to true to make the pregrasps be just outside the bounding box instead of self.pregrasp_dist away from the grasp
        self.pregrasp_just_outside_box = rospy.get_param("~pregrasp_just_outside_box", False)

        #how many backing-off-from-the-deepest-possible-grasp grasps to keep for each good grasp found
        self.backoff_depth_steps = rospy.get_param("~backoff_depth_steps", 5)
        if self.backoff_depth_steps < 1:
            self.backoff_depth_steps = 1

        #don't check the neighbors for each grasp (reduces grasps checked, but makes for worse rankings)
        self.disable_grasp_neighbor_check = rospy.get_param("~disable_grasp_neighbor_check", False)

        #for outputting feature weights
        self._output_features = 0
        if self._output_features:
            self._outfile = file("feature_weights.txt", 'w')

        #debug drawing modes
        self.debug = 0
        self.draw_gripper = 0


    ##pretty-print list to string
    def pplist(self, list):
        return ' '.join(['%5.3f'%x for x in list])


    ##draw the box model of the gripper, placed at pose_mat (4x4 scipy matrix) in frame (defaults to the object frame)
    def draw_gripper_model(self, pose_mat, frame = 'object_frame', pause_after_broadcast = 0):

        #broadcast the gripper frame to tf
        (pos, quat) = mat_to_pos_and_quat(pose_mat)
        (object_frame_pos, object_frame_quat) = mat_to_pos_and_quat(self.object_to_cluster_frame)
        if frame == 'object_frame':
            self.tf_broadcaster.sendTransform(object_frame_pos, object_frame_quat, rospy.Time.now(), "object_frame", self.cluster_frame) 
        now = rospy.Time.now()
        self.tf_broadcaster.sendTransform(pos, quat, now, 'wrist_frame', frame)
        if pause_after_broadcast:
            time.sleep(.1)
        self.tf_listener.waitForTransform('wrist_frame', frame, now, rospy.Duration(2.0))

        box_id = 0
        for i in range(len(self.gripper_boxes)):
            color = [0,1,0]
            self.draw_functions.draw_rviz_box(scipy.matrix(scipy.identity(4)), ranges = self.gripper_boxes[i], frame = 'wrist_frame', ns = "gripper_model", \
                                                  id = i, color = color, duration = 30, opaque = .75)
        box_id += len(self.gripper_boxes)
        for i in range(len(self.space_boxes)):
            for j in range(len(self.space_boxes[i])):
                color = [0,.2*i,1]            
                self.draw_functions.draw_rviz_box(scipy.matrix(scipy.identity(4)), ranges = self.space_boxes[i][j], frame = 'wrist_frame', ns = "gripper_model", \
                                                 id = box_id, color=color, duration = 30, opaque = .75)
                box_id += 1

        #if self.debug:
        #    self.keypause()


    ##return the xyz bounding box ranges for the parts of the gripper model placed at 3-list pos and 3x3 scipy matrix rot
    #uses a really simple model of the fully-open gripper as three boxes (2 fingers and a palm), with a box for the empty space in-between 
    #gripper_boxes is a list of boxes, each of which is a 2-list (min, max) of 3-lists (x,y,z) of corner coords 
    #space boxes is a list of lists of box ranges; there must be points contained in each box-list for the grasp to be valid (points are summed within lists) 
    #origin of gripper is at the ?_wrist_roll_link 
    def gripper_model(self):
        #palm_box = [[.04, -.0755, -.026], [.128, .0755, .026]]
        #left_finger_box = [[.128, .0405, -.0115], [.185, .0755, .0115]]
        #right_finger_box = [[.128, -.0755, -.0115], [.185, -.0405, .0115]]
        palm_box = [[.04, -.0755, -.026], [.138, .0755, .026]]
        left_finger_box = [[.138, .0425, -.0115], [.185, .0755, .0115]]
        right_finger_box = [[.138, -.0755, -.0115], [.185, -.0425, .0115]]
        #space_box = [[.128, -.0405, -.0115], [.185, .0405, .0115]]
        space_box = [[.138, -.0405, -.005], [.180, .0405, .005]]
        gripper_boxes = [palm_box, left_finger_box, right_finger_box]
        space_boxes = [[space_box,],]

        return gripper_boxes, space_boxes


    ##return a count of points (4xn scipy mat) contained in a bounding box (ranges is a 2-list (min, max) of 3-lists (x,y,z))
    def find_points_in_bounding_box(self, wrist_frame_points, ranges, return_points = 0):

        #find the number of  points that fit within the box ranges in the wrist frame
        gt = scipy.array(wrist_frame_points[0:3,:]).T > scipy.array(ranges[0])
        lt = scipy.array(wrist_frame_points[0:3,:]).T < scipy.array(ranges[1])
        fits_in_box_bool = scipy.logical_and(gt, lt)
        fits_in_box = scipy.all(fits_in_box_bool, axis=1)

        if return_points:
            points_that_fit = wrist_frame_points[:, fits_in_box]
            return (fits_in_box.sum(), points_that_fit)

        #print "points in bounding box:", fits_in_box.sum()

        return fits_in_box.sum()

#         fits_in_box = [pt for pt in wrist_frame_points.T if \
#                            pt[0,0] > ranges[0][0] and pt[0,0] < ranges[1][0] and \
#                            pt[0,1] > ranges[0][1] and pt[0,1] < ranges[1][1] and \
#                            pt[0,2] > ranges[0][2] and pt[0,2] < ranges[1][2]]
#         return len(fits_in_box)
                                  

    ##transform ranges for a bounding box using transform 
    def transform_ranges(self, transform, ranges):
        corners = [scipy.matrix([ranges[xind][0], ranges[yind][1], ranges[zind][2], 1]).T for \
                       (xind, yind, zind) in scipy.ndindex(2,2,2)] 
        corners = scipy.concatenate(corners, axis=1)
        transformed_corners = transform*corners
        transformed_corners_ranges = [transformed_corners.min(axis=1)[0:3].T.tolist()[0], transformed_corners.max(axis=1)[0:3].T.tolist()[0]]
        return transformed_corners_ranges
        
        
    ##check if a box goes through the x-y plane
    #ranges is [[xmin, ymin, zmin], [xmax, ymax, zmax]] 
    #pose is a 4x4 scipy matrix
    def check_box_plane_collisions(self, pose, ranges):

        #transform the box corners into the object frame (where the table is the x-y plane) and check that z is above the table
        transformed_corners = self.transform_ranges(pose, ranges)
        for transformed_corner in transformed_corners:
            if transformed_corner[2] < -0.005:
#                 if self.debug:
#                     self.draw_functions.draw_rviz_points(scipy.matrix(transformed_corner).T, frame = 'object_frame', size = .02, ns = "box_plane_collision_points", id=0, color = [1,1,0])
#                     print "transformed_corner:", transformed_corner
#                     #self.keypause()
                return 1
        return 0

    
    ##check for collisions and points within the gripper when placed at a given pose (4x4 scipy mat)
    #returns -1 for colliding with points, -2 for colliding with the table, 
    #and the number of points inside the gripper otherwise
    #table is assumed to be the xy plane
    def check_gripper_pose(self, points, pose):

        if self.debug:
            #self.draw_functions.clear_rviz_points(ns = 'wrist_frame_points', id = 0)
            for i in range(3):
                self.draw_functions.clear_rviz_points(ns = 'box_point_collision_points', id = i)
            self.draw_functions.clear_rviz_points(ns = 'inside_gripper_points', id = 0)

        #draw the gripper model we're currently checking in rviz and broadcast the current 'wrist_frame' transform
        if self.draw_gripper:
            self.draw_gripper_model(pose)

#         #check if any of the boxes at that pose go through the table (assumed to be the xy plane)
#         for i in range(len(self.gripper_boxes)):
#             if self.check_box_plane_collisions(pose, self.gripper_boxes[i]):
#                 if self.debug:
#                     print "box-table collision"
#                 #self.keypause()
#                 return -2

        #transform the points to the wrist frame
        world_to_wrist = pose**-1
        transformed_points = world_to_wrist * points
        #self.draw_functions.draw_rviz_points(transformed_points, frame = 'wrist_frame', size = .005, ns = 'wrist_frame_points', id = 0, color = [.5, .5, .5], opaque = 1)
        #self.keypause()

        #figure out whether the boxes collide with any of the points
        num_collision_points = 0
        for i in range(len(self.gripper_boxes)):
            if self.debug:
                (num_points, points) = self.find_points_in_bounding_box(transformed_points, self.gripper_boxes[i], 1)
            else:
                num_points = self.find_points_in_bounding_box(transformed_points, self.gripper_boxes[i], 0)
            if num_points != 0:
                if self.debug:
                    print "box-point collision"
                    self.draw_functions.draw_rviz_points(points, frame = 'wrist_frame', size = .0075, ns = 'box_point_collision_points', id = i, \
                                                           duration = 20., color = [1,0,0], opaque = 1) 
                    #self.keypause()
                num_collision_points += num_points
                #return -1
            
        #no collisions, count the number of points in each gripper space and return the min of all the gripper-space-box lists
        debug_points = None
        min_space_points = 1e6
        current_list_space_points = 0
        for i in range(len(self.space_boxes)):
            for j in range(len(self.space_boxes[i])):
                if self.debug:
                    (num_space_points, points) = self.find_points_in_bounding_box(transformed_points, self.space_boxes[i][j], 1)    
                    if debug_points == None:
                        debug_points = points
                    else:
                        debug_points = scipy.hstack((debug_points, points))
                else:
                    num_space_points = self.find_points_in_bounding_box(transformed_points, self.space_boxes[i][j], 0)
                current_list_space_points += num_space_points
            if current_list_space_points < min_space_points:
                min_space_points = current_list_space_points
        if min_space_points and self.debug:
            print "min_space_points:", min_space_points
            if debug_points != None:
                self.draw_functions.draw_rviz_points(debug_points, frame = 'wrist_frame', size = .0075, ns = 'inside_gripper_points', id = 0., \
                                                     duration = 20., color = [0,0,1], opaque = 1)
        if self.debug:
            self.keypause()

        return (min_space_points, num_collision_points)


    ##given a cluster and a grasp pose (4x4 scipy mat in object frame), guess at the probability of success for the grasp
    def evaluate_arbitrary_grasp(self, points, pose):

        #print ppmat(pose)

        #how close to orthogonal is the pose?
        orthogonality = self.orthogonal_measure(pose)

        #find the distances from the object box center to the fingertip center when projected on the gripper x-, y-, and z-axes
        (x_dist, y_dist, z_dist) = self.find_fingertip_object_center_dists(pose)

        #how close to overhead is the pose?  Turn the angle into a value from 0 (sideways or worse) to 1 (exactly overhead).
        overhead_angle = self.overhead_angle(pose)
        overhead_grasp = max(0, 1 - overhead_angle / (math.pi/2))

        #figure out whether all the points fit within the gripper (all points no more than gripper width/2 away from gripper x-z plane)
        fits_in_hand = self.object_fits_in_hand(points, pose)

        #check the grasp's neighbors to see if it's at the edge of the object 
        not_edge = self.check_grasp_neighbors(points, pose)

        #how many points in the gripper space box?  (-1 if gripper is colliding)
        (point_count, collision_points) = self.check_gripper_pose(points, pose)

        #don't like grasps that don't have at least some number of points in the gripper
        if point_count < self.min_good_grasp_points:
            #rospy.loginfo("in collision or not enough points")
            #raw_input()
            #print "not enough points, returning 0"
            return 0

        #get the estimated probability of success
        probability = self.grasp_quality(point_count, x_dist, z_dist, overhead_grasp, fits_in_hand, not_edge, orthogonality, y_dist, collision_point_count = collision_points)

        return probability


    ##turn a quality value into a 'probability' value (fairly arbitrary, not based on real data)
    def quality_to_probability(self, quality):

        #sigmoid around center
        center = 50
        scale = 150. / 3.6  #150 + center -> ~ 95%
        t = (quality - center) / scale
        if t < -50: #avoiding math range error
            return 0
        prob = 1./(1.+math.exp(-t))
        return prob


    ##how far is the pose from being orthogonal to the object frame?
    #returns a measure of angle of the closest rotation that would make the pose orthogonal to the object frame
    def orthogonal_measure(self, pose):
        
	#align the gripper x-axis and then y-axis with the nearest object frame axis
	aligned_pose = pose.copy()
	for axis in (0, 1):
		axis_vect = scipy.array(aligned_pose[0:3, axis].T)[0]
		closest_axis = scipy.fabs(axis_vect).argmax()
		axis_dir = axis_vect[closest_axis] / abs(axis_vect[closest_axis])
		closest_axis_vect = scipy.zeros(3)
		closest_axis_vect[closest_axis] = axis_dir
                dot = min(1, max(0, scipy.dot(closest_axis_vect, axis_vect)))
                angle = math.acos(dot)
                if math.fabs(angle) > 1e-6:
                    rot_vect = scipy.cross(axis_vect, closest_axis_vect)
                    align_rot = scipy.matrix(tf.transformations.rotation_matrix(angle, rot_vect))
                    aligned_pose = align_rot * aligned_pose

	#find the angle difference between the newly-aligned rotation and the original pose
	rel_mat = aligned_pose**-1 * pose
	rel_quat = tf.transformations.quaternion_from_matrix(rel_mat)
        if rel_quat[3] > 1.0:
            angle = 0
        elif rel_quat[3] < -1.0:
            angle = math.pi
	else:
            angle = 2*math.acos(rel_quat[3])

        #return 1 if the pose is exactly orthogonal (angle 0), 0 if the pose is more than pi/6 away
        orthogonality = max(0, 1 - angle / (math.pi/6))
	return orthogonality
    

    ##find the distances from the object box center to the fingertip center when projected on the gripper x-, y-, and z-axes
    def find_fingertip_object_center_dists(self, pose):

        #object bounding box center in gripper frame
        object_center = scipy.matrix([0, 0, self.object_bounding_box_dims[2]/2, 1]).T
        gripper_frame_center = pose**-1 * object_center

        #dist to fingertip center (fingertips past center is even better than at center)
        x_dist = gripper_frame_center[0,0] - self._wrist_to_fingertip_center_dist
        
        #dist to either side of 0 is bad
        y_dist = math.fabs(gripper_frame_center[1,0])
        z_dist = math.fabs(gripper_frame_center[2,0])

        return (x_dist, y_dist, z_dist)
    

    ##figure out whether all the points fit within the gripper for an arbitrary pose (no more than gripper width/2 away from gripper x-z plane)
    def object_fits_in_hand(self, points, pose):

        #transform the points to the wrist frame
        transformed_points = pose**-1 * points

        #check if any of the points' y-coords have a magnitude greater than the gripper width/2
        return not (scipy.fabs(transformed_points[1, :]) > self.gripper_opening).any()


    ##angle of gripper pose x-axis away from directly overhead (global -z)
    def overhead_angle(self, pose):

        z_comp_of_x_axis = max(0, min(1, -pose[2,0]))
        angle = math.acos(z_comp_of_x_axis)
        return angle


    ##check both sides of a grasp to see if they are also viable grasps
    def check_grasp_neighbors(self, points, grasp):

#         print "grasp"
#         self.draw_gripper_model(grasp, pause_after_broadcast = 1)
#         self.draw_gripper_model(grasp, pause_after_broadcast = 1)
#         self.keypause()

        neighbor_dist = .02
        for sign in [-1., 1.]:
            
            #shift the grasp by neighbor_dist in z   
            shift_mat = scipy.matrix(tf.transformations.translation_matrix([0,0,sign*neighbor_dist]))
            shifted_grasp = grasp * shift_mat

#             print "shifted grasp"
#             self.draw_gripper_model(shifted_grasp, pause_after_broadcast = 1)
#             self.draw_gripper_model(shifted_grasp, pause_after_broadcast = 1)

            #test to see if it's any good
            (point_count, collision_points) = self.check_gripper_pose(points, shifted_grasp)
            if point_count < self.min_good_grasp_points or collision_points > 0:
                #print "neighbor was no good"
                #self.keypause()
                return 0
            #print "neighbor good"

        #both sides were good
        #print "both neighbors were good"
        return 1


    ##assign a numerical quality value for this grasp
    def grasp_quality(self, point_count, palm_dist, side_dist, overhead_grasp, fits_in_hand, not_edge, orthogonality = 1, centered_dist = 0, points = None, grasp = None, collision_point_count = 0):

        # if grasp != None:
        #     print "running evaluate_arbitrary_grasp:"
        #     self.evaluate_arbitrary_grasp(points, grasp)
        #     print "original grasp quality params:"

        #rospy.loginfo("pc: %d, x_d: %0.3f, z_d: %0.3f, og: %0.3f, ih: %d, ne: %d, orth: %0.3f, y_d: %0.3f\n"%(point_count, palm_dist, side_dist, overhead_grasp, fits_in_hand, not_edge, orthogonality, centered_dist))

        feature_vector = [point_count, palm_dist, side_dist, overhead_grasp, fits_in_hand, not_edge, orthogonality, centered_dist, collision_point_count]
        if self._output_features:
            self._outfile.write(str(feature_vector)+'\n')

        #adjust the point_count weight depending on the density of points
        point_count_weight = 5./self._points_per_sq_cm

        #hand chosen weights for now (collisions set to be unacceptable)
        weights = [point_count_weight, -10./0.01, -10./0.01, 10., 50., 50., 50., -10./0.01, -1000]  
        quality = sum([w*f for (w,f) in zip(weights, feature_vector)])

        prob = self.quality_to_probability(quality)

        #rospy.loginfo("quality: %0.3f, prob: %0.3f\n"%(quality, prob))

        return prob


    ##find the best grasp from a given start pose along the palm (gripper x) direction, in steps of size palm_step 
    #moving the gripper inward until/as long as there are object points within the gripper, until there are collisions
    #checking for collisions with the table (assumed to be the xy plane)
    #returns the grasp pose if a good grasp is found, None if not
    def find_best_palm_dir_grasp(self, points, start_pose, palm_step, object_bounding_box):
        
        #find a reasonable max distance to move, in case we never find collisions
        #using half the max bounding box dim + 6 cm
        max_dist = max([(upper-lower)/2 for (upper, lower) in zip(object_bounding_box[1], object_bounding_box[0])]) + .06

        good_grasps = []
        good_grasp_point_counts = []
        good_grasp_dists_moved = []
        new_pose = start_pose.copy()
        dist_moved = 0.
        while(1):
            #print "dist_moved:", dist_moved

            #check to see if the gripper pose is good (stop if collisions are found)
            (point_count, collision_points) = self.check_gripper_pose(points, new_pose)
#            if self.debug:
#                time.sleep(.02)
#                self.keypause()
            if point_count < 0 or collision_points > 0:
                break
            
            #add the grasp to the list if it is acceptable
            if point_count > self.min_good_grasp_points:
                good_grasps.append(new_pose.copy())
                good_grasp_point_counts.append(point_count)
                good_grasp_dists_moved.append(dist_moved)

            #direction to move is gripper's x-axis
            new_pose[0:3, 3] += start_pose[0:3, 0] * palm_step
            dist_moved += palm_step

            #moved too far already
            if dist_moved > max_dist:
                break
        
        return (good_grasps[-self.backoff_depth_steps:], good_grasp_point_counts[-self.backoff_depth_steps:], good_grasp_dists_moved[-self.backoff_depth_steps:])


    ##for a given gripper pose and axis (0=x, 1=y, 2=z), both expressed in the points frame, 
    #find grasps at a fixed spacing along that axis in both directions
    #while varying the palm dist (inward by steps of palm_step) as long as there are object points within the gripper, until there are collisions
    #start_pose is a 4x4 scipy mat, object_bounding_box is [[xmin, ymin, zmin], [xmax, ymax, zmax]]
    #min_points is the minimum number of points that need to be inside the hand for the grasp to be okay
    def find_grasps_along_direction(self, points, start_pose, axis, spacing, palm_step, object_bounding_box, max_side_move = None, omit_center = False):
        
        #how far to keep away from the edge of the bounding box
        min_edge_dist = .02

        dist = 0.
        positive_done = 0
        negative_done = 0
        new_pose = start_pose.copy()
        good_grasps = []
 
        while(not positive_done or not negative_done):
            if self.debug:
                print "dist:", dist

            #check positive dist along direction
            new_pose[axis, 3] = dist + start_pose[axis, 3] 
            #print "new_pose:\n", ppmat(new_pose)

            if not positive_done and not (dist == 0 and omit_center):
                (grasps, point_counts, palm_dists_moved) = self.find_best_palm_dir_grasp(points, new_pose, palm_step, object_bounding_box)
                for (grasp, point_count, palm_dist_moved) in zip(grasps, point_counts, palm_dists_moved):
                    good_grasps.append([grasp, point_count, palm_dist_moved, dist])

            #only check the center once
            if dist != 0.:
                #check negative dist along direction
                if not negative_done:
                    new_pose[axis, 3] = -dist + start_pose[axis, 3] 
                    (grasps, point_counts, palm_dists_moved) = self.find_best_palm_dir_grasp(points, new_pose, palm_step, object_bounding_box)
                    for (grasp, point_count, palm_dist_moved) in zip(grasps, point_counts, palm_dists_moved):
                        good_grasps.append([grasp, point_count, palm_dist_moved, dist])

            #increment dist and check to see if we've gone off either end of the cluster (keeping away from the ends)
            dist += spacing
            if max_side_move != None and dist > max_side_move:
                break
            if -dist + start_pose[axis, 3] < object_bounding_box[0][axis] + min_edge_dist:
                negative_done = 1
            if dist + start_pose[axis, 3] > object_bounding_box[1][axis] - min_edge_dist:
                positive_done = 1

        return good_grasps
        

    ##find grasps of high points on the object, pointing in towards the bottom center of the object
    def find_high_point_grasps(self, points, object_bounding_box, object_bounding_box_dims, palm_step, max_grasps_to_return = 40):
        print "starting high_point_grasps"
        
        max_grasps_to_find = int(math.floor(max_grasps_to_return * 1.5))

        #points should already be z-sorted after removing outliers, so we just need to use the points at the end
        num_points = scipy.shape(points)[1]
        num_selected_points = max_grasps_to_find*2
        max_dist_from_top = .02
        step_size = 50

        highest_z = points[2,-1]
        ind_of_lowest_pt = scipy.searchsorted(points[2, :].tolist()[0], highest_z - max_dist_from_top)
        print "ind_of_lowest_pt:", ind_of_lowest_pt

        #use the highest point, and randomly select some more points to try
        selected_point_inds = scipy.random.randint(ind_of_lowest_pt, num_points-2, (1,num_selected_points))
        selected_point_inds[0,0] = num_points-1                                            
        selected_points = points[:, selected_point_inds.tolist()[0]]
        #if self.debug:
        #self.draw_functions.draw_rviz_points(selected_points, frame = 'object_frame', size = .01, ns = 'selected_points', id = 0, duration = 20., color = [1,1,0], opaque = 1.0)

        #check if we can grasp each selected point from above, with the gripper y-axis pointing towards the object z-axis
        grasps_with_features = []
        for point_ind in range(num_selected_points): 
            point = selected_points[:, point_ind]

            #start the wrist out so that the fingertips are at the edge of the bounding box
            wrist_z_pos = object_bounding_box_dims[2] + self._wrist_to_fingertip_center_dist

            #find overhead grasps (gripper x is down (-z), y is toward the z-axis, z is x cross y)
            start_pose = scipy.matrix(scipy.identity(4))
            point_xy_mag = math.sqrt(point[0:2,0].T*point[0:2,0])
            x_axis = scipy.matrix([0.,0.,-1.]).T
            y_axis = scipy.matrix([point[0,0]/point_xy_mag, point[1,0]/point_xy_mag, 0]).T
            z_axis = scipy.matrix(scipy.cross(x_axis.T, y_axis.T).T)
            start_pose[0:3, 0] = x_axis
            start_pose[0:3, 1] = y_axis
            start_pose[0:3, 2] = z_axis
            start_pose[0, 3] = point[0,0]
            start_pose[1, 3] = point[1,0]
            start_pose[2, 3] = wrist_z_pos

            #search along the palm direction for the best grasp
            (grasps, point_counts, palm_dists_moved) = self.find_best_palm_dir_grasp(points, start_pose, palm_step, object_bounding_box)
            #rospy.loginfo("len(grasps):%d"%len(grasps))
            for (grasp, point_count, palm_dist_moved) in zip(grasps, point_counts, palm_dists_moved):
                grasps_with_features.append((grasp, point_count, palm_dist_moved))

            #stop once we have max_grasps_to_find grasps
            if len(grasps_with_features) > max_grasps_to_find:
                break

        #compute the qualities for the found grasps
        grasps_with_qualities = []    
        for (grasp, point_count, palm_dist_moved) in grasps_with_features:

            #figure out whether the grasp is still good when shifted to both sides in z
            not_edge = 1
            if not self.disable_grasp_neighbor_check:
                not_edge = self.check_grasp_neighbors(points, grasp)
            
            #compute the quality
            orthogonality = self.orthogonal_measure(grasp)

            #find the distances from the object box center to the fingertip center when projected on the gripper x-, y-, and z-axes
            (x_dist, y_dist, z_dist) = self.find_fingertip_object_center_dists(grasp)

            #figure out whether all the points fit within the gripper (all points no more than gripper width/2 away from gripper x-z plane)
            fits_in_hand = self.object_fits_in_hand(points, grasp)

            quality = self.grasp_quality(point_count, x_dist, y_dist, overhead_grasp = 1, fits_in_hand = fits_in_hand, not_edge = not_edge, 
                                         orthogonality = orthogonality, centered_dist = z_dist, points = points, grasp = grasp)
            grasps_with_qualities.append([grasp, quality, palm_dist_moved])


        #sort them by quality and return only the best ones
        grasps_with_qualities = sorted(grasps_with_qualities, key=lambda t:t[1], reverse=True)

        # #clip off all those with quality below 0.7
        # for (ind, grasp_with_quality) in enumerate(grasps_with_qualities):
        #     if ind == 0:
        #         continue
        #     if grasp_with_quality[1] < 0.7:
        #         rospy.loginfo("tossing out %d grasps with quality below 0.7"%(len(grasps_with_qualities)-ind))
        #         grasps_with_qualities = grasps_with_qualities[:ind]
        #         break

        rospy.loginfo("high point grasp qualities: " + self.pplist([x[1] for x in grasps_with_qualities]))

        return grasps_with_qualities[:max_grasps_to_return]


    ##pause for enter
    def keypause(self):
        print "press enter to continue"
        c = raw_input()
        return c


    ##initialization for planning grasps 
    def init_cluster_grasper(self, cluster):
        
        self.cluster_frame = cluster.header.frame_id

        #use PCA to find the object frame and bounding box dims, and to get the cluster points in the object frame (z=0 at bottom of cluster)
        (self.object_points, self.object_bounding_box_dims, self.object_bounding_box, \
                  self.object_to_base_frame, self.object_to_cluster_frame) = \
                  self.cbbf.find_object_frame_and_bounding_box(cluster)
        
        #for which directions does the bounding box fit within the hand?
        gripper_space = [self.gripper_opening - self.object_bounding_box_dims[i] for i in range(3)]
        self._box_fits_in_hand = [gripper_space[i] > 0 for i in range(3)]

        #only half benefit for the longer dimension, if one is significantly longer than the other
        if self._box_fits_in_hand[0] and self._box_fits_in_hand[1]:
            if gripper_space[0] > gripper_space[1] and self.object_bounding_box_dims[0]/self.object_bounding_box_dims[1] < .8:
                self._box_fits_in_hand[1] *= .5
            elif gripper_space[1] > gripper_space[0] and self.object_bounding_box_dims[1]/self.object_bounding_box_dims[0] < .8:
                self._box_fits_in_hand[0] *= .5
        #rospy.loginfo("bounding box dims: "+pplist(self.object_bounding_box_dims))
        #rospy.loginfo("self._box_fits_in_hand: "+str(self._box_fits_in_hand))

        #compute the average number of points per sq cm of bounding box surface (half the surface only)
        bounding_box_surf = (self.object_bounding_box_dims[0]*100 * self.object_bounding_box_dims[1]*100) + \
            (self.object_bounding_box_dims[0]*100 * self.object_bounding_box_dims[2]*100) + \
            (self.object_bounding_box_dims[1]*100 * self.object_bounding_box_dims[2]*100)
        self._points_per_sq_cm = self.object_points.shape[1] / bounding_box_surf
        #rospy.loginfo("bounding_box_surf: %.2f, number of points: %d"%(bounding_box_surf, self.object_points.shape[1]))
        #rospy.loginfo("points per sq cm: %.4f"%self._points_per_sq_cm)


    ##evaluate grasps on the point cluster
    def evaluate_point_cluster_grasps(self, grasps, frame):

        #convert the grasps to 4x4 scipy matrix wrist poses in object frame
        grasp_poses = []
        cluster_to_object_frame = self.object_to_cluster_frame**-1
        for grasp in grasps:
            pose_mat = pose_to_mat(grasp.grasp_pose)

            #transform them to gripper model poses in object frame
            obj_frame_mat = cluster_to_object_frame * pose_mat * self.model_frame_in_actual_wrist_frame
            grasp_poses.append(obj_frame_mat)

        #evaluate the grasp qualities
        probs = [self.evaluate_arbitrary_grasp(self.object_points, pose) for pose in grasp_poses]

        return probs


    ##plan grasps for a point cluster 
    def plan_point_cluster_grasps(self):

        if self.debug:
            print "about to find grasps"
            self.keypause()

        #search for grasps from all relevant sides
        found_grasps = []
        found_grasps1 = []
        found_grasps2 = []
        num_overhead_grasps = 0
        num_overhead_grasps1 = 0

        #start the wrist out so that the palm is at the edge of the bounding box
        top_wrist_z_pos_palm = self.object_bounding_box[1][2] + self._wrist_to_palm_dist
        top_y_start_pose_palm = scipy.matrix([[0.,0.,-1.,0.],
                                              [0.,-1.,0.,0.],
                                              [-1.,0.,0.,top_wrist_z_pos_palm],
                                              [0.,0.,0.,1.]])
        top_x_start_pose_palm = scipy.matrix([[0.,-1.,0.,0.],
                                              [0.,0.,1.,0.],
                                              [-1.,0.,0.,top_wrist_z_pos_palm],
                                              [0.,0.,0.,1.]])

        y_center_grasps_found = 0
        x_center_grasps_found = 0
        if not self.side_grasps_only:
            #if bounding box fits in hand, check overhead grasp with gripper along y 
            if self._box_fits_in_hand[1] > 0:
                grasp_list = self.find_grasps_along_direction(self.object_points, top_y_start_pose_palm, 0, \
                               self.side_step, self.palm_step, self.object_bounding_box, max_side_move = 0.)
                for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                    quality = self.grasp_quality(point_count, \
                               palm_dist = top_wrist_z_pos_palm-self._wrist_to_palm_dist-palm_dist_moved-self.object_bounding_box_dims[2]/2., \
                               side_dist = dist, overhead_grasp = 1, fits_in_hand = self._box_fits_in_hand[1], \
                               not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                    found_grasps.append([grasp, quality, palm_dist_moved])
                    y_center_grasps_found = 1

            #if bounding box fits in hand, check overhead grasp with gripper along x
            if self._box_fits_in_hand[0] > 0:
                grasp_list = self.find_grasps_along_direction(self.object_points, top_x_start_pose_palm, 1, \
                               self.side_step, self.palm_step, self.object_bounding_box, max_side_move = 0.)
                for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                    quality = self.grasp_quality(point_count, \
                               palm_dist = top_wrist_z_pos_palm-self._wrist_to_palm_dist-palm_dist_moved-self.object_bounding_box_dims[2]/2., \
                               side_dist = dist, overhead_grasp = 1, fits_in_hand = self._box_fits_in_hand[0], \
                               not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                    found_grasps1.append([grasp, quality, palm_dist_moved])
                    x_center_grasps_found = 1
            rospy.loginfo("overhead grasp (x) qualities: " + self.pplist([x[1] for x in found_grasps]))
            rospy.loginfo("overhead grasp (y) qualities: " + self.pplist([x[1] for x in found_grasps1]))
            num_overhead_grasps = len(found_grasps) 
            num_overhead_grasps1 = len(found_grasps1)

        if not self.overhead_grasps_only:
            #for side grasps, start the wrist either halfway up the side of the object bounding box or self.side_grasp_start_height up, whichever is higher
            #with the palm at the edge of the bounding box
            side_wrist_z_pos = max(self.object_bounding_box[1][2]/2., self.side_grasp_start_height)
            side_wrist_y_pos = self.object_bounding_box[1][1] + self._wrist_to_palm_dist
            side_wrist_x_pos = self.object_bounding_box[1][0] + self._wrist_to_palm_dist

            #find side grasps for dimensions for which the bounding box fits within the hand
            if self._box_fits_in_hand[1] > 0 and self.object_bounding_box_dims[2] > self.height_good_for_side_grasps:

                #find side grasps from the right (gripper x is -x, y is -y, z is +z in object frame)
                start_pose = scipy.matrix([[-1.,0.,0.,side_wrist_x_pos],
                                           [0.,-1.,0.,0],
                                           [0.,0.,1.,side_wrist_z_pos],
                                           [0.,0.,0.,1.]])
                grasp_list = self.find_grasps_along_direction(self.object_points, start_pose, 2, \
                                self.side_step, self.palm_step, self.object_bounding_box)
                for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                    quality = self.grasp_quality(point_count, \
                                palm_dist = side_wrist_x_pos-self._wrist_to_palm_dist-palm_dist_moved, \
                                side_dist = dist, overhead_grasp = 0, fits_in_hand = self._box_fits_in_hand[1], \
                                not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                    found_grasps.append([grasp, quality, palm_dist_moved])

                #find side grasps from the left (gripper x is +x, y is -y, z is -z in object frame)
                start_pose = scipy.matrix([[1.,0.,0.,-side_wrist_x_pos],
                                           [0.,-1.,0.,0],
                                           [0.,0.,-1.,side_wrist_z_pos],
                                           [0.,0.,0.,1.]])
                grasp_list = self.find_grasps_along_direction(self.object_points, start_pose, 2, self.side_step, \
                                self.palm_step, self.object_bounding_box)
                for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                    quality = self.grasp_quality(point_count, palm_dist = side_wrist_x_pos-self._wrist_to_palm_dist-palm_dist_moved, \
                                side_dist = dist, overhead_grasp = 0, fits_in_hand = self._box_fits_in_hand[1], \
                                not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                    found_grasps.append([grasp, quality, palm_dist_moved])

            if self._box_fits_in_hand[0] > 0 and self.object_bounding_box_dims[2] > self.height_good_for_side_grasps:

                #find side grasps from the front (gripper x is +y, y is -x, z is +z in object frame)        
                start_pose = scipy.matrix([[0.,-1.,0.,0.],
                                           [1.,0.,0.,-side_wrist_y_pos],
                                           [0.,0.,1.,side_wrist_z_pos],
                                           [0.,0.,0.,1.]])
                grasp_list = self.find_grasps_along_direction(self.object_points, start_pose, 2, self.side_step, self.palm_step, self.object_bounding_box)
                for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                    quality = self.grasp_quality(point_count, palm_dist = side_wrist_y_pos-self._wrist_to_palm_dist-palm_dist_moved, \
                                 side_dist = dist, overhead_grasp = 0, fits_in_hand = self._box_fits_in_hand[0], \
                                 not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                    found_grasps1.append([grasp, quality, palm_dist_moved])

                #find side grasps from the back (gripper x is -y, y is +x, z is +z in object frame)
                start_pose = scipy.matrix([[0.,1.,0.,0.],
                                           [-1.,0.,0.,side_wrist_y_pos],
                                           [0.,0.,1.,side_wrist_z_pos],
                                           [0.,0.,0.,1.]])        
                grasp_list = self.find_grasps_along_direction(self.object_points, start_pose, 2, self.side_step, self.palm_step, self.object_bounding_box)
                for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                    quality = self.grasp_quality(point_count, palm_dist = side_wrist_y_pos-self._wrist_to_palm_dist-palm_dist_moved, \
                                 side_dist = dist, overhead_grasp = 0, fits_in_hand = self._box_fits_in_hand[0], \
                                 not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                    found_grasps1.append([grasp, quality, palm_dist_moved])
                rospy.loginfo("side grasp (x) qualities: " + self.pplist([x[1] for x in found_grasps[num_overhead_grasps:]]))
                rospy.loginfo("side grasp (y) qualities: " + self.pplist([x[1] for x in found_grasps1[num_overhead_grasps1:]]))

        if not self.side_grasps_only:
            #look for overhead grasps along the axes (not just at the center of the box), starting with fingertips at the edge of the bounding box
            #also find overhead grasps of not-along-axis high points with the gripper oriented with one finger toward the center
            #if found_grasps == []:
            #look for overhead grasps along y
            top_wrist_z_pos_fingertip = self.object_bounding_box[1][2] + self._wrist_to_fingertip_center_dist
            top_y_start_pose_fingertip = scipy.matrix([[0.,0.,-1.,0.],
                                                       [0.,-1.,0.,0.],
                                                       [-1.,0.,0.,top_wrist_z_pos_fingertip],
                                                       [0.,0.,0.,1.]])
            top_x_start_pose_fingertip = scipy.matrix([[0.,-1.,0.,0.],
                                                       [0.,0.,1.,0.],
                                                       [-1.,0.,0.,top_wrist_z_pos_fingertip],
                                                       [0.,0.,0.,1.]])


            grasp_list = self.find_grasps_along_direction(self.object_points, top_y_start_pose_fingertip, 0, self.side_step, self.palm_step, 
                                                          self.object_bounding_box, omit_center = y_center_grasps_found)
            for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                quality = self.grasp_quality(point_count, \
                                 palm_dist = top_wrist_z_pos_fingertip-self._wrist_to_fingertip_center_dist-palm_dist_moved-self.object_bounding_box_dims[2]/2., \
                                 side_dist = dist, overhead_grasp = 1, fits_in_hand = self.object_bounding_box_dims[1] < self.gripper_opening, \
                                 not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                found_grasps2.append([grasp, quality, palm_dist_moved])

            #look for overhead grasps along x    
            grasp_list = self.find_grasps_along_direction(self.object_points, top_x_start_pose_fingertip, 1, self.side_step, self.palm_step, 
                                                          self.object_bounding_box, omit_center = x_center_grasps_found)
            for (ind, (grasp, point_count, palm_dist_moved, dist)) in enumerate(grasp_list):
                quality = self.grasp_quality(point_count, \
                                 palm_dist = top_wrist_z_pos_fingertip-self._wrist_to_fingertip_center_dist-palm_dist_moved-self.object_bounding_box_dims[2]/2., \
                                 side_dist = dist, overhead_grasp = 1, fits_in_hand = self.object_bounding_box_dims[0] < self.gripper_opening, \
                                 not_edge = ind < len(grasp_list)-2, points = self.object_points, grasp = grasp)
                found_grasps2.append([grasp, quality, palm_dist_moved])
            rospy.loginfo("along-axes overhead grasp qualities: " + self.pplist([x[1] for x in found_grasps2]))
            num_overhead_grasps = len(found_grasps)

            #look for grasps of high points
            if self.include_high_point_grasps:
                grasps_with_qualities = self.find_high_point_grasps(self.object_points, self.object_bounding_box, self.object_bounding_box_dims, self.palm_step)
                found_grasps2.extend(grasps_with_qualities)

        #sort the grasps by quality (highest quality first), but keep all the centered grasps before the more marginal grasps
        found_grasps = sorted(found_grasps, key=lambda t:t[1], reverse=True)
        found_grasps1 = sorted(found_grasps1, key=lambda t:t[1], reverse=True)        
        found_grasps2 = sorted(found_grasps2, key=lambda t:t[1], reverse=True)

        #priortize grasps around the short side first, if there's a significant difference and they both fit in hand
        if self._box_fits_in_hand[0] < self._box_fits_in_hand[1]:
            found_grasps = found_grasps + found_grasps1 + found_grasps2
        elif self._box_fits_in_hand[1] < self._box_fits_in_hand[0]:
            found_grasps = found_grasps1 + found_grasps + found_grasps2
        else:
            found_grasps = sorted(found_grasps + found_grasps1, key=lambda t:t[1], reverse=True) + found_grasps2

        rospy.loginfo("total number of grasps found:" + str(len(found_grasps)))        
        
        #generate the pregrasp poses and turn both grasps and pregrasps into Pose messages
        grasp_poses = []
        pregrasp_poses = [] 

        pregrasp_dists = []
        qualities = []

        #draw the gripper model just at the best grasp
        if found_grasps and self.draw_gripper:
            (best_grasp_mat, best_grasp_quality, palm_dist_moved) = found_grasps[0]
            self.draw_gripper_model(best_grasp_mat, pause_after_broadcast = 0)

        #generate the pregrasps and pose messages
        for (grasp_mat, quality, palm_dist_moved) in found_grasps:
            
            #pregrasp is just the grasp pose, backed up by self.pregrasp_dist (m)
            #or palm_dist_moved, if we want the pregrasp to be just outside the bounding box
            back_up_mat = scipy.identity(4)
            if not self.pregrasp_just_outside_box:
                back_up_mat[0,3] = -self.pregrasp_dist
                pregrasp_dists.append(self.pregrasp_dist)
            else:
                back_up_mat[0,3] = -palm_dist_moved-.005
                pregrasp_dists.append(palm_dist_moved+.005)
            pregrasp_mat = grasp_mat * back_up_mat

            #transform the grasp_mat so that it reflects the actual desired wrist frame 
            grasp_mat = grasp_mat * self.actual_wrist_frame_in_model_frame
            pregrasp_mat = pregrasp_mat * self.actual_wrist_frame_in_model_frame

            #convert to Pose messages in the same frame as the original cluster
            pregrasp_pose = mat_to_pose(pregrasp_mat, self.object_to_cluster_frame)
            grasp_pose = mat_to_pose(grasp_mat, self.object_to_cluster_frame)

            pregrasp_poses.append(pregrasp_pose)
            grasp_poses.append(grasp_pose)
            qualities.append(quality)

        #for now, gripper is always open all the way at the pregrasp pose
        gripper_openings = [.1]*len(grasp_poses)

        return (pregrasp_poses, grasp_poses, gripper_openings, qualities, pregrasp_dists)


    
