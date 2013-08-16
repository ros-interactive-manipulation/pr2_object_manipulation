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

## @package pick_and_place_manager
# Functions to grasp objects off flat tables and place them down in 
# specified rectangular regions in space

import roslib
roslib.load_manifest('pr2_pick_and_place_demos')
import rospy
from object_manipulation_msgs.msg import PickupAction, PickupGoal, \
    PlaceAction, PlaceGoal, GripperTranslation, ReactivePlaceAction, ReactivePlaceGoal
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from object_manipulation_msgs.msg import ManipulationResult
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_collision_map_processing.srv import \
    TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
from arm_navigation_msgs.msg import LinkPadding
import tf
import actionlib
import scipy
import time
import copy
import math
import pdb
import threading
import sys
import pr2_gripper_reactive_approach.controller_manager as controller_manager
import tabletop_collision_map_processing.collision_map_interface as collision_map_interface
import pr2_gripper_reactive_approach.reactive_grasp as reactive_grasp
import object_manipulator.draw_functions as draw_functions
from object_manipulator.convert_functions import *
from arm_navigation_msgs.msg import JointConstraint, PositionConstraint, OrientationConstraint, ArmNavigationErrorCodes, AllowedContactSpecification, Constraints
from tf_conversions import posemath

##saved info about one object
class ObjectInfo():
    def __init__(self, object, box_dims, pose, tf_listener, height_above_table, collision_name):
        self.object = object      #the original GraspableObject message
        self.box_dims = box_dims  #bounding box dims (only for point clusters)
        self.pose = pose          #the original pose on the table when detected (PoseStamped) (of bounding box if point cluster) 
        self.grasp = None         #the Grasp object returned by the Pickup service, once the service is called
        self.grasp_pose = None    #where it was grasped, once it is grasped
        self.grasp_type = 'normal' #whether the grasp was normal ('normal') or a slid-off-table grasp ('flat')
        self.collision_name = collision_name  #the name of the object in the collision space

        #for point clusters, record the transform from the cluster origin to the bounding box pose
        #(object bounding box to base_link = pose)**-1 * (cluster to base link) = (cluster origin to bounding box)
        if len(self.object.potential_models) == 0:
            self.type = 'cluster'
        else:
            self.type = 'mesh'
            #self.cluster_origin_to_bounding_box = None
        cluster_to_base_link = get_transform(tf_listener, object.cluster.header.frame_id, 'base_link')
        pose_mat = pose_to_mat(pose.pose)
        self.cluster_origin_to_bounding_box = pose_mat**-1 * cluster_to_base_link #cluster to bounding box transform

        #save the height of the object above the current table detection
        self.height_above_table = height_above_table


##Manager for pick and place actions
class PickAndPlaceManager():


    def __init__(self, use_slip_controller = 0, use_slip_detection = 0):

        #just "left", just "right", or "both" arms?
        use_right_arm = rospy.get_param("~use_right_arm", "true")
        use_left_arm = rospy.get_param("~use_left_arm", "true")
        self.arms_to_use_list = [0,1]
        self.arms_to_use = "both"
        if use_right_arm and not use_left_arm:
            self.arms_to_use = "right"
            self.arms_to_use_list = [0]
        elif use_left_arm and not use_right_arm:
            self.arms_to_use = "left"
            self.arms_to_use_list = [1]            
        elif not use_left_arm and not use_right_arm:
            rospy.logerr("grasp_executive: params say we're not using any arms!!  Defaulting to using both")
        if self.arms_to_use != "both":
            rospy.loginfo("grasp_executive: using only %s arm"%self.arms_to_use)
        
        self.stereo_camera_frame = rospy.get_param("~stereo_camera_frame", "/head_mount_kinect_rgb_optical_frame")

        #should we use the slip detecting gripper controller?
        self.use_slip_controller = use_slip_controller

        #should we use slip detection, or just normal gripper commands?
        self.use_slip_detection = use_slip_detection

        #grasp/place action names
        self.grasper_grasp_name = 'object_manipulator/object_manipulator_pickup'
        self.grasper_place_name = 'object_manipulator/object_manipulator_place'

        #grasp/place action clients
        self.grasper_grasp_action_client = actionlib.SimpleActionClient(self.grasper_grasp_name, PickupAction)
        rospy.loginfo("grasp_executive: waiting for object_manipulator_pickup action")
        self.grasper_grasp_action_client.wait_for_server()
        rospy.loginfo("grasp_executive: object_manipulator_pickup action found")
        self.grasper_place_action_client = actionlib.SimpleActionClient(self.grasper_place_name, PlaceAction)
        rospy.loginfo("grasp_executive: waiting for object_manipulator_place action")
        self.grasper_place_action_client.wait_for_server()
        rospy.loginfo("grasp_executive: object_manipulator_place action found")

        #service names
        self.grasper_detect_name = 'object_detection'
        self.collision_map_processing_name = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
        #self.grasper_transfer_name = '/grasping_app_transfer'
        self.find_bounding_box_name = '/find_cluster_bounding_box'

        #head action client
        self.head_action_client = \
            actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        rospy.loginfo("grasp_executive: waiting for point_head_action client")
        self.head_action_client.wait_for_server()
        rospy.loginfo("grasp_executive: point_head_action client found")

        #reactive place action clients
        self.reactive_place_clients = [None, None]
        if self.arms_to_use in ["both", "right"]:
            rospy.loginfo("grasp_executive: waiting for right reactive place client")
            self.reactive_place_clients[0] = actionlib.SimpleActionClient("reactive_place/right", ReactivePlaceAction)
            self.reactive_place_clients[0].wait_for_server()
        if self.arms_to_use in ["both", "left"]:
            rospy.loginfo("grasp_executive: waiting for left reactive place client")
            self.reactive_place_clients[1] = actionlib.SimpleActionClient("reactive_place/left", ReactivePlaceAction)
            self.reactive_place_clients[1].wait_for_server()
        rospy.loginfo("grasp_executive: reactive place clients found")

        #start tf listener
        self.tf_listener = tf.TransformListener()

        #initialize controller managers for both arms
        self.cms = [None, None]
        if self.arms_to_use in ["both", "right"]:
            rospy.loginfo("grasp_executive: initializing controller manager for right arm")
            self.cms[0] = controller_manager.ControllerManager('r', self.tf_listener, use_slip_controller, use_slip_detection)
        if self.arms_to_use in ["both", "left"]:
            rospy.loginfo("grasp_executive: initializing controller manager for left arm")
            self.cms[1] = controller_manager.ControllerManager('l', self.tf_listener, use_slip_controller, use_slip_detection)
        rospy.loginfo("grasp_executive: done initializing controller managers for both arms")

        #wait for services
        rospy.loginfo("grasp_executive: waiting for object_detection service")
        rospy.wait_for_service(self.grasper_detect_name)
        rospy.loginfo("grasp_executive: object_detection service found")

        rospy.loginfo("grasp_executive: waiting for collision_map_processing service")
        rospy.wait_for_service(self.collision_map_processing_name)
        rospy.loginfo("grasp_executive: collision_map_processing service found")

        #rospy.loginfo("grasp_executive: waiting for grasping_app_transfer service")
        #rospy.wait_for_service(grasper_transfer_name)
        #rospy.loginfo("grasp_executive: grasper_transfer service found")

        rospy.loginfo("grasp_executive: waiting for find_cluster_bounding_box service")
        rospy.wait_for_service(self.find_bounding_box_name)
        rospy.loginfo("grasp_executive: find_cluster_bounding_box service found")

        #service proxies
        self.grasper_detect_srv = rospy.ServiceProxy(self.grasper_detect_name, TabletopDetection)
        self.collision_map_processing_srv = rospy.ServiceProxy(self.collision_map_processing_name, \
                                                                   TabletopCollisionMapProcessing)

        #self.grasper_transfer_srv = rospy.ServiceProxy(self.grasper_transfer_name, GraspingAppTransfer)
        self.bounding_box_srv = rospy.ServiceProxy(self.find_bounding_box_name, FindClusterBoundingBox)

        #initialize a collision map interface
        self.collision_map_interface = collision_map_interface.CollisionMapInterface()

        #initialize a DrawFunctions object
        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')

        #the objects held in the each arm (None if no object)
        self.held_objects = [None]*2

        #original pose of the object in each hand
        self.original_poses = [None]*2

        #temporary height and dist until we get a table detection
        self.table_front_edge_x = rospy.get_param("~default_table_front_edge_x", .33)
        self.table_height = rospy.get_param("~default_table_height", .66)
        
        #rectangle where we can put objects down (defaults to just in front of the robot, on the table)
        self.place_rect_dims = [.3, .6]
        place_rect_pose_mat = scipy.matrix([[1.,0.,0.,self.table_front_edge_x+self.place_rect_dims[0]/2.+.1],
                                            [0.,1.,0.,0.],
                                            [0.,0.,1.,self.table_height],
                                            [0.,0.,0.,1.]])
        self.place_rect_pose_stamped = stamp_pose(mat_to_pose(place_rect_pose_mat), 'base_link')

        #occupancy grid of places to put down an object
        self.place_grid_resolution = 5
        self.place_grid = [[0]*self.place_grid_resolution for i in range(self.place_grid_resolution)]

        #saved table and object detections
        self.detected_table = None
        self.additional_tables = []
        self.detected_objects = []

        #dictionary of which arm is which
        self.arm_dict = {0:'r', 1:'l'}

        #arm-away joint angles
        r_side_trajectory = rospy.get_param("/arm_configurations/side/trajectory/right_arm")
        l_side_trajectory = rospy.get_param("/arm_configurations/side/trajectory/left_arm")
        self.arm_above_and_to_side_angles = [r_side_trajectory[0:7], l_side_trajectory[0:7]]
        self.arm_to_side_angles = [r_side_trajectory[7:14], l_side_trajectory[7:14]]
        # self.arm_above_and_to_side_angles = [[-0.968, 0.729, -0.554, -1.891, -1.786, -1.127, 0.501],
        #                                 [0.968, 0.729, 0.554, -1.891, 1.786, -1.127, 0.501]]
        # self.arm_to_side_angles = [[-2.135, 0.803, -1.732, -1.905, -2.369, -1.680, 1.398],
        #                       [2.135, 0.803, 1.732, -1.905, 2.369, -1.680, 1.398]]

        #dictionary of grasp/place error codes 
        #(SUCCESS, UNFEASIBLE, FAILED, ERROR, MOVE_ARM_STUCK, LIFT_FAILED)
        self.result_code_dict = {}
        for element in dir(ManipulationResult):
            if element[0].isupper():
                self.result_code_dict[eval('ManipulationResult.'+element)] = element

        #dictionary of tabletop_object_detector error codes 
        #(NO_CLOUD_RECEIVED, NO_TABLE, OTHER_ERROR, SUCCESS)
        self.tabletop_detection_result_dict = {}
        for element in dir(TabletopDetectionResult):
            if element[0].isupper():
                self.tabletop_detection_result_dict[eval('TabletopDetectionResult.'+element)] = element

        #name of the support surface's collision object
        self.collision_support_surface_name = "table"

        #offset for place override approach (where to come in from when dropping open-loop in base_link frame; z is separate)
        self.place_override_approach_offset = [-.15, 0, 0]


    ##shift a place pose to be above the current table plane
    def shift_place_pose_height(self, object, place_pose, table, z_offset):
        
        #find the height of the place_pose above the current table
        new_height = self.find_height_above_table(place_pose, table)

        #figure out the desired offset to make it the same height as when it was grasped
        #plus the desired z_offset
        height_diff = object.height_above_table +z_offset - new_height

        #don't move down (which would probably make the desired pose into the actual table)
        if height_diff <= 0:
            height_diff = 0
        
        #add that offset to the place_pose plus a tiny offset
        place_pose.pose.position.z += height_diff + .001

        rospy.loginfo("shifting place pose height, height_diff=%5.3f"%height_diff)


    ##find the height of an object pose above a detected table plane
    def find_height_above_table(self, pose, table):
        
        #shift the pose into the table's header frame
        (pos, rot) = pose_stamped_to_lists(self.tf_listener, pose, table.pose.header.frame_id)
       
        #find the pose's location in the table pose frame
        pos_mat = scipy.matrix(pos+[1]).T
        table_mat = pose_to_mat(table.pose.pose)
        pos_in_table_frame = table_mat**-1 * pos_mat

        #return the z-position
        #rospy.loginfo("height above table: %5.3f"%pos_in_table_frame[2,0])
        return pos_in_table_frame[2,0]


    ##point the head to find the front edge of the table and detect
    def find_table(self, point = None):
        if point == None:
            point = [self.table_front_edge_x, 0, self.table_height]
        self.point_head(point, 'base_link')
        self.call_tabletop_detection(update_table = 1, update_place_rectangle = 1)


    ##call find_cluster_bounding_box to get the bounding box for a cluster
    def call_find_cluster_bounding_box(self, cluster):

        req = FindClusterBoundingBoxRequest()
        req.cluster = cluster
        try:
            res = self.bounding_box_srv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%(self.find_bounding_box_name, e))  
            self.throw_exception()
            return 0
        return (res.pose, [res.box_dims.x, res.box_dims.y, res.box_dims.z])

        
    ##call tabletop object detection and collision_map_processing 
    #(detects table/objects and adds them to collision map)
    def call_tabletop_detection(self, update_table = 0, clear_attached_objects = 1, \
                            replace_table_in_collision_map = 1, update_place_rectangle = 0, \
                            num_models = 0):

        rospy.loginfo("calling tabletop detection")

        det_req = TabletopDetectionRequest()
        det_req.return_clusters = 1
        det_req.return_models = 1
        det_req.num_models = num_models

        #call tabletop detection, get a detection result
        for try_num in range(3):
            try:
                det_res = self.grasper_detect_srv(det_req)
            except rospy.ServiceException, e:
                rospy.logerr("error when calling %s: %s"%(self.grasper_detect_name, e))
                self.throw_exception()
                return ([], None)        
            if det_res.detection.result == det_res.detection.SUCCESS:
                rospy.loginfo("tabletop detection reports success")
                break
            else:
                rospy.logerr("tabletop detection failed with error %s, trying again"%\
                                 self.tabletop_detection_result_dict[det_res.detection.result])
        else:
            rospy.logerr("tabletop detection failed too many times.  Returning.")
            return ([], None)

        col_req = TabletopCollisionMapProcessingRequest()
        col_req.reset_collision_models = 1
        col_req.reset_attached_models = clear_attached_objects
        col_req.detection_result = det_res.detection
        col_req.desired_frame = 'base_link'

        #call collision map processing to add the detected objects to the collision map 
        #and get back a list of GraspableObjects
        try:
            col_res = self.collision_map_processing_srv(col_req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%(self.collision_map_processing_name, e))
            self.throw_exception()
            return ([], None)

        table = det_res.detection.table
        self.collision_support_surface_name = col_res.collision_support_surface_name

        #save the new detected table (already in collision map)
        if update_table:
            self.detected_table = table
            self.update_table_info(update_place_rectangle)

        #sort objects by distance from the edge of the table 
        rospy.loginfo("detection finished, finding bounding boxes for clusters and sorting objects")
        dists = []
        object_box_dims = []
        object_poses = []
        heights_above_table = []
        for (ind, object) in enumerate(col_res.graspable_objects):

            #convert the pose for database objects into the object reference frame
            if len(object.potential_models) != 0:
                pose_stamped = object.potential_models[0].pose
                obj_frame_pose_stamped = change_pose_stamped_frame(self.tf_listener, pose_stamped, object.reference_frame_id)
                (pos, rot) = pose_stamped_to_lists(self.tf_listener, pose_stamped, 'base_link')
                object_poses.append(obj_frame_pose_stamped)
                object_box_dims.append([.1, .1, .01])
                dists.append(pos[0])

            #find the bounding box for point clusters and store the box pose (should already be in base_link frame)
            if len(object.potential_models) == 0:
                (pose_stamped, box_dims) = self.call_find_cluster_bounding_box(object.cluster)
                object_box_dims.append(box_dims)
                object_poses.append(pose_stamped)
                dists.append(pose_stamped.pose.position.x)

            #find the heights of the objects above the detected table plane
            heights_above_table.append(self.find_height_above_table(pose_stamped, table))

            #print "new pose stamped:\n", object.model_pose.pose

        indices = scipy.array(dists).argsort()

        detected_objects = [ObjectInfo(col_res.graspable_objects[indices[i]], object_box_dims[indices[i]], \
                                  object_poses[indices[i]], self.tf_listener, heights_above_table[indices[i]], \
                                  col_res.collision_object_names[indices[i]]) for i in range(len(indices))]

        self.detected_objects = detected_objects

        #print out the new object list
        self.print_object_list()

        return (detected_objects, table)

    
    ##adds the point cloud of object2 to object1 and updates the bounding box
    def combine_point_cloud_objects(self, object1, object2):

        object1.cluster.points += object2.cluster.points

        #re-find the bounding box
        (pose_stamped, box_dims) = self.call_find_cluster_bounding_box(object1.cluster)
        object1.pose = pose_stamped
        object1.box_dims.append = box_dims

        return object1


    ##make sure the joint controllers are on
    def check_joint_controllers(self, whicharm = None):

        if whicharm:
            self.cms[whicharm].check_controllers_ok('joint')
        else:
            for arm in self.arms_to_use_list:
                self.cms[arm].check_controllers_ok('joint')


    ##tell the grasp action to grasp an object (whicharm: right = 0, left = 1)
    def grasp_object(self, object, whicharm = None, use_reactive_grasp = 1, use_slip_detection = 0):
        rospy.loginfo("attempting to grasp an object, whicharm = %s"%str(whicharm))

        if not self.check_arms_to_use(whicharm):
            return (0, None, None)

        #make sure the joint controllers are on
        self.check_joint_controllers(whicharm)

        #open the gripper
        self.open_gripper(whicharm)

#         #tell reactive grasping not to close too hard
#         if whicharm == 0:
#             rospy.set_param('reactive_grasp_node_right/close_force', 20)
#         else:
#             rospy.set_param('reactive_grasp_node_left/close_force', 20)

        #put together the grasp goal
        goal = PickupGoal()

        #tell the grasp action how to lift
        goal.lift = GripperTranslation()
        goal.lift.desired_distance = .1
        goal.lift.min_distance = 0.
        goal.lift.direction = create_vector3_stamped([0.,0.,1.], 'base_link')

        goal.target = object.object
        goal.use_reactive_execution = use_reactive_grasp
        goal.use_reactive_lift = use_slip_detection
        goal.collision_object_name = object.collision_name
        goal.collision_support_surface_name = self.collision_support_surface_name
        goal.max_contact_force = 50.

        print "sending object collision name of:", object.collision_name
        if whicharm == 0:  
            goal.arm_name = "right_arm"
        else:
            goal.arm_name = "left_arm"

        #send the goal
        try:
            self.grasper_grasp_action_client.send_goal(goal)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s"%self.grasper_grasp_name)
            self.throw_exception()
            return ("ERROR", None, None)

        #wait for result
        finished_within_time = self.grasper_grasp_action_client.wait_for_result(rospy.Duration(240))
        if not finished_within_time:
            self.grasper_grasp_action_client.cancel_goal()
            rospy.logerr("timed out when asking the grasp action client to grasp")
            return (0, None, None)

        #return the result
        result = self.grasper_grasp_action_client.get_result()
        resultval = self.result_code_dict[result.manipulation_result.value]
        rospy.loginfo("grasp result: %s"%resultval)

        return (resultval, whicharm, result.grasp)
                
        
    ##distance from a point (2-list) to a bounding box (projected onto table plane)
    #returns 0 if the point is inside; positive if it's outside
    def point_to_box_dist(self, point, box_pose, box_dims):
        
        #convert point to the object (pose) frame
        box_mat = pose_to_mat(box_pose.pose)
        box_mat[2, 3] = 0.
        point4 = scipy.matrix([point[0], point[1], 0, 1]).T
        box_frame_point = (box_mat**-1 * point4).T.tolist()[0]        

        #find the distance from the rectangle in 2-D (inside rectangle has dist of 0)
        dist = 0
        for dim in range(2):
            if box_frame_point[dim] < -box_dims[dim]/2:
                dist += (-box_dims[dim]/2 - box_frame_point[dim])**2
            elif box_frame_point[dim] > box_dims[dim]/2:
                dist += (box_frame_point[dim] - box_dims[dim]/2)**2
        dist = dist**.5

        return dist


    ##return the sign of only the (x,y) components of (p1-p2)x(p3-p2) for convex hull inclusion testing
    #p1, p2, and p3 are lists of coords
    def find_cross_direction(self, p1, p2, p3):
        z = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])
        return z > 0


    ##fills in place grid spots that are already taken by objects seen by the last tabletop detection
    def fill_in_taken_place_grid_spots(self, buffer = .05):

        #go through each grid point
        for i in range(self.place_grid_resolution):
            for j in range(self.place_grid_resolution):
                
                self.place_grid[i][j] = 0

                #2-D location of the grid point
                point = self.place_grid_location(i, j)

                #check if it's within buffer distance of a detected object's bounding box
                for (ind, object) in enumerate(self.detected_objects):
                    dist = self.point_to_box_dist(point, object.pose, object.box_dims)

                    #if so, mark that spot as taken
                    if dist < buffer:
                        rospy.loginfo("grid point %d %d is within %4.2f of detected object %d, dist=%4.2f"\
                                          %(i, j, buffer, ind, dist))
                        self.place_grid[i][j] = 1
#                     else:
#                         rospy.loginfo("grid point %d %d is clear of object %d, dist=%4.2f" %(i, j, ind, dist))

                #check if the point is above the table's convex hull
                if self.detected_table and len(self.detected_table.convex_hull.vertices) >= 2:
                    vertices = self.detected_table.convex_hull.vertices
                    direction = self.find_cross_direction(get_xyz(vertices[0]), get_xyz(vertices[1]), point)
                    for ind in range(1, len(vertices)-2):
                        next_dir = self.find_cross_direction(get_xyz(vertices[ind]), get_xyz(vertices[ind+1]), point)
                        if direction != next_dir:

                            #if the point is not above the table, mark it as taken
                            self.place_grid[i][j] = 1
                            rospy.loginfo("grid point %d %d is not above the table"%(i,j))
                            break                
                
   
    ##return a LinkPadding list with the gripper touch links for whicharm (right = 0, left = 1) and padding pad
    def create_gripper_link_padding(self, whicharm, pad):
        link_name_list = ["_gripper_palm_link", "_gripper_r_finger_tip_link", "_gripper_l_finger_tip_link", "_gripper_l_finger_link", 
                          "_gripper_r_finger_link", "_wrist_roll_link"]
        if whicharm == 1:
            prefix = 'l'
        else:
            prefix = 'r'
        if pad < 0:
            rospy.logerr("pad was less than 0!  Using 0")
            pad = 0.
        arm_link_names = [prefix + link_name for link_name in link_name_list]
        link_padding_list = [LinkPadding(link_name, pad) for link_name in arm_link_names]
        return link_padding_list            


    ##tell the place service to place an object held by whicharm at a particular location (right = 0, left = 1)
    #expressed as a PoseStamped
    def place_object(self, whicharm, pose, padding = .05, constrained = False):
        if not self.check_arms_to_use(whicharm):
           return "ERROR"

        rospy.loginfo("attempting to place object")
        if constrained == True:
            rospy.loginfo("using constrained place");

        #send in where the wrist ought to be in base_link frame to put the object there
        wrist_to_cluster_mat = pose_to_mat(self.held_objects[whicharm].grasp_pose.pose)
        cluster_to_object_mat = self.held_objects[whicharm].cluster_origin_to_bounding_box
        object_to_base_link_mat = pose_to_mat(pose.pose)
        wrist_to_base_link_mat = object_to_base_link_mat * cluster_to_object_mat * wrist_to_cluster_mat        
        wrist_pose = stamp_pose(mat_to_pose(wrist_to_base_link_mat), 'base_link')
        wrist_pose.header.stamp = rospy.Time.now()

        #draw the wrist pose
        self.draw_functions.draw_rviz_axes(wrist_to_base_link_mat, 'base_link', id = 0, duration = 60.)

        goal = PlaceGoal()
        if whicharm == 0:  
            goal.arm_name = "right_arm"
            rospy.loginfo("asking the right arm to place")

        elif whicharm == 1:
            goal.arm_name = "left_arm"
            rospy.loginfo("asking the left arm to place")

        goal.place_locations = [wrist_pose]
        
        goal.desired_retreat_distance = 0.1
        goal.min_retreat_distance = 0.05

        goal.approach = GripperTranslation()
        goal.approach.desired_distance = .1
        goal.approach.min_distance = 0.05
        goal.approach.direction = create_vector3_stamped([0.,0.,-1.], 'base_link')

        goal.collision_object_name = self.held_objects[whicharm].collision_name

        #take the gripper link paddings down to 0 and disable collisions between the gripper and 
        #everything for the move from the pre-place to the place
        goal.place_padding = 0.        
        goal.additional_link_padding = self.create_gripper_link_padding(whicharm, 0.0)
        goal.collision_support_surface_name = "all"
        goal.allow_gripper_support_collision = True

        goal.use_reactive_place = self.use_slip_detection
        if constrained == True:
            current_pose = self.cms[whicharm].get_current_wrist_pose_stamped('base_link')
            orientation_constraint = self.get_keep_object_level_constraint(whicharm,current_pose)
            goal.path_constraints.orientation_constraints.append(orientation_constraint)

        #send the goal
        try:
            self.grasper_place_action_client.send_goal(goal)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s", self.grasper_place_name)
            self.throw_exception()
            return "ERROR"

        #wait for result
        finished_within_time = self.grasper_place_action_client.wait_for_result(rospy.Duration(240))
        if not finished_within_time:
            self.grasper_place_action_client.cancel_goal()
            rospy.logerr("timed out when asking the place action client to place")
            return "FAILED"

        #return the result
        result = self.grasper_place_action_client.get_result()
        resultval = self.result_code_dict[result.manipulation_result.value]
        rospy.loginfo("place result: %s"%resultval)
        return resultval


    ##find an IK solution for a pose mat
    def find_IK_solution(self, whicharm, pose_mat, start_angles, collision_aware = 1):
        pose_stamped = stamp_pose(mat_to_pose(pose_mat), 'base_link')
        (joint_angles, error_code) = self.cms[whicharm].ik_utilities.run_ik(pose_stamped, \
                                  start_angles, self.cms[whicharm].ik_utilities.link_name, \
                                  collision_aware = collision_aware)
        return (joint_angles, error_code)


    #make sure whicharm is consistent with the arms being used
    def check_arms_to_use(self, whicharm):
        if whicharm == 0 and self.arms_to_use not in ["right", "both"]:
            rospy.logerr("not using right arm!")
            return 0
        if whicharm == 1 and self.arms_to_use not in ["left", "both"]:
            rospy.logerr("not using left arm!")
            return 0
        return 1


    ##place the object open-loop, shoving other objects aside
    def place_object_override(self, whicharm, pose, z_dist = .1, use_joint_open_loop = 0):
        rospy.loginfo("last-resort place")

        if not self.check_arms_to_use(whicharm):
            return 0


        #figure out where the wrist has to be to make the object be at pose
        if self.held_objects[whicharm] and not self.held_objects[whicharm].grasp_type == 'flat':

            # #recognized object, saved grasp pose is wrist pose in object frame
            # if self.held_objects[whicharm].type == 'mesh':
                
            #     #(wrist to base_link) = (object to base_link) * (wrist to object) 
            #     wrist_to_object_mat = pose_to_mat(self.held_objects[whicharm].grasp_pose)
            #     object_to_base_link_mat = pose_to_mat(pose.pose)
            #     wrist_to_base_link_mat = object_to_base_link_mat * wrist_to_object_mat

            #point cluster, saved grasp pose is wrist pose in cluster frame
            # else:
                #(wrist to base link) = (object to base_link) * (cluster origin to object) * (wrist to cluster)
            wrist_to_cluster_mat = pose_to_mat(self.held_objects[whicharm].grasp_pose.pose)
            cluster_to_object_mat = self.held_objects[whicharm].cluster_origin_to_bounding_box
            object_to_base_link_mat = pose_to_mat(pose.pose)
            wrist_to_base_link_mat = object_to_base_link_mat * cluster_to_object_mat * wrist_to_cluster_mat
            
            wrist_pose = stamp_pose(mat_to_pose(wrist_to_base_link_mat), 'base_link')

            #draw the wrist pose
            self.draw_functions.draw_rviz_axes(wrist_to_base_link_mat, 'base_link', id = 0, duration = 60.)


        #if we don't think there's an object in the hand (or if we're dropping a flat object), 
        #assume the gripper orientation should be held fixed
        #and let go of the object above pose (pose orientation should contain the current gripper orientation already)
        else:
            #figure out how high to raise the gripper to put the middle of the fingertips just above the loc            
            if self.held_objects[whicharm] and self.held_objects[whicharm].grasp_type == 'flat':

                #use the longest box dim plus a bit for flat objects, and rotate the gripper by 90 degrees
                pose.pose.position.z += self.held_objects[whicharm].box_dims[0]+.05
                pose = stamp_pose(mat_to_pose(pose_to_mat(pose.pose) * \
                       scipy.matrix(tf.transformations.rotation_matrix(math.pi/2, [1,0,0]))), 'base_link')
            else:
                pose.pose.position.z += .1
            fingertip_to_base_link_mat = pose_to_mat(pose.pose)
            wrist_to_fingertip_mat = scipy.matrix(tf.transformations.translation_matrix([-.167, 0, 0]))
            wrist_to_base_link_mat = fingertip_to_base_link_mat * wrist_to_fingertip_mat
            wrist_pose = pose
            set_xyz(wrist_pose.pose.position, wrist_to_base_link_mat[0:3, 3].T.tolist()[0])

#             #attach a medium-sized box to the gripper so the object isn't guaranteed to hit the table
#             box_dims = [.15]*3
#             current_wrist_pose = self.cms[whicharm].get_current_wrist_pose_stamped('base_link')
#             box_to_wrist = scipy.matrix(tf.transformations.translation_matrix([.12+box_dims[0], 0, 0]))
#             box_pose = mat_to_pose(pose_to_mat(current_wrist_pose.pose) * box_to_wrist)
#             self.collision_map_interface.add_collision_box(box_pose, box_dims, 'base_link', 'temporary_box')
#             self.attach_object(whicharm, 'temporary_box')

        #come in from the front, just above the table 
        front_above_pose = copy.deepcopy(wrist_pose)

        #temporary, while deepcopy of rospy.Time is broken
        front_above_pose.header.stamp = rospy.Time(wrist_pose.header.stamp.secs)
        front_above_pose.pose.position.x += self.place_override_approach_offset[0]
        front_above_pose.pose.position.y += self.place_override_approach_offset[1]
        front_above_pose.pose.position.z += self.place_override_approach_offset[2] + z_dist

        #draw the wrist_pose and front_above_pose
        self.draw_functions.draw_rviz_axes(wrist_to_base_link_mat, 'base_link', id = 0, duration = 60.)
        front_above_mat = pose_to_mat(front_above_pose.pose)
        self.draw_functions.draw_rviz_axes(front_above_mat, 'base_link', id = 1, duration = 60.)
        
        #ask move arm to get us there, or move open-loop if move arm fails and we can find an IK solution 
        result = self.try_hard_to_move_pose(whicharm, front_above_pose, use_joint_open_loop = use_joint_open_loop, \
                                                use_cartesian = 0, try_constrained = 1)
        if not result:
            return 0

        #now move to the place pose using the Cartesian controllers
        #self.cms[whicharm].move_cartesian(wrist_pose, settling_time = rospy.Duration(10))

        #now move to the place pose using reactive place
        self.call_reactive_place(whicharm, wrist_pose)

        #drop the object, if we haven't already opened the gripper
        self.open_gripper(whicharm)

        #detach the object (and add it back where it was dropped if we knew what object it was)
        if self.held_objects[whicharm]:
            self.detach_and_add_back_object(whicharm)
        else:
            self.detach_object(whicharm)

        #lift the arm
        wrist_pose.pose.position.z += .2
        self.cms[whicharm].move_cartesian(wrist_pose, settling_time = rospy.Duration(10), timeout = rospy.Duration(3))

        #get out of there
        self.move_arm_to_side(whicharm)
        return 1

    
    ##call the reactive place service for the appropriate arm
    def call_reactive_place(self, whicharm, place_pose):
            
        goal = ReactivePlaceGoal()
        goal.final_place_pose = place_pose
        
        self.reactive_place_clients[whicharm].send_goal(goal)
        self.reactive_place_clients[whicharm].wait_for_result()
        result = self.reactive_place_clients[whicharm].get_result()
    
        return result


    ##attach the object in the hand to the gripper (right = 0, left = 1)
    def attach_object(self, whicharm, object_name):
        rospy.loginfo("attaching object %s to gripper %s"%(object_name, self.arm_dict[whicharm]))
        self.collision_map_interface.attach_object_to_gripper(object_name, self.arm_dict[whicharm])
        

    ##detach the object in the hand from the gripper (right = 0, left = 1)
    def detach_object(self, whicharm):
        rospy.loginfo("detaching object from %s gripper"%self.arm_dict[whicharm])
        self.collision_map_interface.detach_all_objects_from_gripper(self.arm_dict[whicharm])
        

    ##detach and add the object back to the collision map
    def detach_and_add_back_object(self, whicharm, collision_name = None):
        if collision_name == None:
            if self.held_objects[whicharm]:
                rospy.loginfo("detaching object %s from %s gripper and adding it back to the collision map" \
                                 %(self.held_objects[whicharm].collision_name, self.arm_dict[whicharm]))
                self.collision_map_interface.detach_and_add_back_objects_attached_to_gripper(self.arm_dict[whicharm], \
                                                            self.held_objects[whicharm].collision_name)
            else:
                rospy.loginfo("gripper doesn't think it's holding any objects; detaching all objects without re-adding")
                self.collision_map_interface.detach_all_objects_from_gripper(self.arm_dict[whicharm])
        else:
            self.collision_map_interface.detach_and_add_back_objects_attached_to_gripper(self.arm_dict[whicharm], collision_name)


    ##remove an object from the collision map
    def remove_object(self, collision_name):
        rospy.loginfo("removing object %s from the collision map"%collision_name)
        self.collision_map_interface.remove_collision_object(collision_name)


    # ##remove an object and add it back in as the current bounding box
    # def update_object_box(self, object_info):
        
    #     self.remove_object(object_info.collision_name)
    #     self.collision_map_interface.add_collision_box(object_info.pose, object_info.box_dims, \
    #                                                        'base_link', object_info.collision_name)


    ##point the head (really the narrow-stereo camera frame) at a location
    def point_head(self, point, frame, pause = 1):
        goal = PointHeadGoal()
        goal.target = create_point_stamped(point, frame)
        goal.pointing_frame = "/narrow_stereo_optical_frame"
        goal.max_velocity = 1.0

        self.head_action_client.send_goal(goal)
        finished_within_time = self.head_action_client.wait_for_result(rospy.Duration(3))
        if not finished_within_time:
            self.head_action_client.cancel_goal()
            rospy.logerr("timed out when asking the head to point to a new goal")
            return 0

        #sleep a bit so a new point cloud has time to be generated
        if pause:
            time.sleep(1.0)
        return 1


    ##use the table detection to update the table information
    #if adjust_place_rectangle is 1, adjusts a place rectangle on the table (assumed base_link frame) 
    #with the new table params
    def update_table_info(self, adjust_place_rectangle = 0):

        #find the table's front edge and height
        base_link_pose_stamped = change_pose_stamped_frame(self.tf_listener, self.detected_table.pose, 'base_link')
        table_mat = pose_to_mat(base_link_pose_stamped.pose)
        corners = scipy.matrix([[self.detected_table.x_min, self.detected_table.y_min,0,1],
                                [self.detected_table.x_min, self.detected_table.y_max,0,1],
                                [self.detected_table.x_max, self.detected_table.y_min,0,1],
                                [self.detected_table.x_max, self.detected_table.y_max,0,1]]).T
        transformed_corners = table_mat * corners
        front_edge_x = transformed_corners[0,:].min()
        height = transformed_corners[2,:].max()
        self.table_front_edge_x = front_edge_x
        self.table_height = height
        rospy.loginfo("table front edge x: %5.3f"%front_edge_x)
        rospy.loginfo("table height: %5.3f"%height)

        #adjust the place rectangle with the new table params
        if adjust_place_rectangle:
            #print "self.table_height before update:", self.table_height
            place_rect_pose_mat = pose_to_mat(self.transform_place_rect_pose())
            #print "place_rect_pose_mat before update:\n", ppmat(place_rect_pose_mat)
            place_rect_pose_mat[0, 3] = self.table_front_edge_x+self.place_rect_dims[0]/2.+.1
            place_rect_pose_mat[2, 3] = self.table_height
            self.place_rect_pose_stamped = stamp_pose(mat_to_pose(place_rect_pose_mat), 'base_link')
            #print "place_rect_pose_mat:\n", ppmat(place_rect_pose_mat)

            #draw the new place grid
            self.set_place_area(self.place_rect_pose_stamped, self.place_rect_dims)


    ##convert the place rectangle pose to base_link frame if it isn't already (returns pose, not pose_stamped)
    def transform_place_rect_pose(self):
        if self.place_rect_pose_stamped.header.frame_id != 'base_link':
            transformed_rect_pose = change_pose_stamped_frame(self.tf_listener, \
                           self.place_rect_pose_stamped, 'base_link').pose
        else:
            transformed_rect_pose = self.place_rect_pose_stamped.pose
        return transformed_rect_pose


    ##set a rectangle centered at rect_pose_stamped of dims (x, y) to be the desired place area
    def set_place_area(self, rect_pose_stamped, dims):
        self.place_rect_dims = dims
        self.place_rect_pose_stamped = rect_pose_stamped

        self.draw_place_area()


    ##draw the current place area as a set of boxes
    def draw_place_area(self):
        #convert the place rectangle pose to base_link frame if it isn't already (just for drawing)
        transformed_rect_pose = self.transform_place_rect_pose()
        place_grid_loc_mat = pose_to_mat(transformed_rect_pose)
        self.draw_functions.draw_rviz_box(place_grid_loc_mat, [.01, .01, .01], duration = 0,\
                                              frame = 'base_link', \
                                              color = [1,0,0], id=101)

        #draw boxes at each place location
        for i in range(self.place_grid_resolution):
            for j in range(self.place_grid_resolution):
                location = self.place_grid_location(i,j)
                place_grid_loc_mat[0,3] = location[0]
                place_grid_loc_mat[1,3] = location[1]
                self.draw_functions.draw_rviz_box(place_grid_loc_mat, [.01, .01, .01], duration = 0,\
                                    frame = 'base_link', color = [0,0,1], id=1000+self.place_grid_resolution*i+j)


    ##return the location of a point in the place grid
    def place_grid_location(self, xind, yind):

        grid_x_size = self.place_rect_dims[0]/self.place_grid_resolution
        grid_y_size = self.place_rect_dims[1]/self.place_grid_resolution
        grid_center = int(math.floor(self.place_grid_resolution/2.))  #make place_grid_resolution odd for now

        #convert the place rectangle pose to base_link frame if it isn't already
        transformed_rect_pose = self.transform_place_rect_pose()

        #convert the place grid relative location to a location in the base_link frame (returned as list)
        rel_location = scipy.matrix([(grid_center-xind)*grid_x_size, (grid_center-yind)*grid_y_size, 0, 1]).T
        location = pose_to_mat(transformed_rect_pose) * rel_location
        location = location.T.tolist()[0]  
        return location

                
    ##return the pose of the object at a place grid location
    def place_grid_pose(self, xind, yind, whicharm, add_noise = 0):

        #find the place grid location for xind, yind
        location = self.place_grid_location(xind, yind)
        rospy.loginfo("proposed place location: %5.3f, %5.3f"%(location[0], location[1]))

        #create a pose_stamped for the object at that location based on its original pose
        if self.held_objects[whicharm] and not self.held_objects[whicharm].grasp_type == 'flat':
            place_pose = copy.deepcopy(self.held_objects[whicharm].pose)
            
            #temporary, while deepcopy of rospy.Time is broken
            #place_pose.header.stamp = rospy.Time(self.held_objects[whicharm].pose.header.stamp.secs)

        #no object in the hand, use the current wrist orientation and change the location
        else:
            place_pose = self.cms[whicharm].get_current_wrist_pose_stamped('base_link')
            
        #set the place_pose z to be the place grid center z
        transformed_rect_pose = self.transform_place_rect_pose()
        place_pose.pose.position.z = transformed_rect_pose.position.z

        #and the x and y to be the current grid location x and y
        place_pose.pose.position.x = location[0]
        place_pose.pose.position.y = location[1]

        #add noise to the place pose, if requested
        if add_noise:
            rospy.loginfo("adding noise to place pose")
            place_pose.pose.position.x += scipy.rand()*.05
            place_pose.pose.position.y += scipy.rand()*.05

        #draw the object's visualization box (bounding or at-the-base) at the place location
        self.draw_object_pose(self.held_objects[whicharm], place_pose, [0,1,1], 100)

        return place_pose

    
    ##draw a visualization box (bounding or at-the-base) for an object at a PoseStamped
    def draw_object_pose(self, object_info, pose_stamped, color, id, duration = 300.):
        pose_mat = pose_to_mat(pose_stamped.pose)
        if object_info:
            box_dims = object_info.box_dims
        else:
            box_dims = [.05, .05, .05]
        self.draw_functions.draw_rviz_box(pose_mat, box_dims, duration = duration,\
                          frame = pose_stamped.header.frame_id, color = color, id = id)         


    ##choose a new location to try to place an object held by whicharm (right = 0, left = 1)
    #if override_pose >=0, then pick an override pose from front to back
    def choose_place_location(self, whicharm, override_pose = -1, z_offset = 0):

        #start from the back and work your way to the front
        i = j = 0

        #unless this is an open-loop place, in which case start from the front and work your way back
        num_poses_to_skip = 0
        if override_pose >= 0:
            i = self.place_grid_resolution-1
            num_poses_to_skip = override_pose

        #go through the grid, looking for a good spot
        for grid_ind in range(self.place_grid_resolution**2+num_poses_to_skip):
            if self.place_grid[i][j] != 0:

                #we only want empty/not previously colliding grid spots (=0) 
                #unless it's an override pose, in which case we'll settle for anything that 
                #isn't explictly taken (=1)
                if override_pose < 0 or self.place_grid[i][j] == 1:

                    if num_poses_to_skip == 0:
                        rospy.loginfo("place_grid spot %d %d taken with value %d"%(i,j, self.place_grid[i][j]))
                    j = j+1
                    if j > self.place_grid_resolution - 1:
                        j = 0
                        if override_pose < 0:
                            i = (i+1)%self.place_grid_resolution
                        else:
                            i = (i-1)%self.place_grid_resolution
                    continue

            #for an override pose, skip over good poses that we've tried before
            if num_poses_to_skip > 0:
                num_poses_to_skip -= 1
                j = j+1
                if j > self.place_grid_resolution - 1:
                    j = 0
                    i = (i-1)%self.place_grid_resolution
                continue

            rospy.loginfo("returning grid location %d %d"%(i, j))

            #compute the relevant pose
            if override_pose >= 0 or z_offset > 0:
                pose = self.place_grid_pose(i,j, whicharm, add_noise = 0)
            else:
                pose = self.place_grid_pose(i,j, whicharm)

            #add the desired z-offset to make sure we're not just grazing the table
            pose.pose.position.z += z_offset

            return (i, j, pose, 0)

        rospy.logerr("all grid locations are taken!  Returning")
        pose = self.place_grid_pose(i,j, whicharm)
        return (i,j, pose, 1)

        
    ##reset all the -1s (temporary collision entries) in the place grid
    def reset_place_grid_temporary_collisions(self):
        for i in range(self.place_grid_resolution):
            for j in range(self.place_grid_resolution):
                if self.place_grid[i][j] == -1:
                    self.place_grid[i][j] = 0


    ##rotate the pose_stamped about the base-link z-axis by rotation rad
    #but keep the position the same
    def rotate_pose(self, pose_stamped, rotation):
        if rotation == 0:
            return pose_stamped

        rot_mat = scipy.matrix(tf.transformations.rotation_matrix(rotation, [0,0,1]))
        rotated_pose_stamped = transform_pose_stamped(pose_stamped, rot_mat, "pre")
        rotated_pose_stamped.pose.position = copy.deepcopy(pose_stamped.pose.position)

        return rotated_pose_stamped


    ##point the head at the place rectangle
    def point_head_at_place_rect(self, offset = 0):

        rospy.loginfo("pointing the head at the place rectangle")

        #convert the place rect pose to base_link if it isn't there already
        transformed_rect_pose = self.transform_place_rect_pose()

        #grab the center 
        (x,y,z) = get_xyz(transformed_rect_pose.position)
        
        #point the head just off of that position
        if offset:
            self.point_head([x-.15, y*.7, z], 'base_link')
        else:
            self.point_head([x,y,z], 'base_link')


    ##put down an object held in whicharm (right = 0, left = 1)
    def put_down_object(self, whicharm, max_place_tries = None, use_place_override = 0, move_to_side = True, constrained = False, update_table = True, update_place_rectangle = True, point_head = True):
        
        if not self.check_arms_to_use(whicharm):
            return 0

        padding = .02  #initial padding around recognized objects (cm)

        #make sure the joint controllers are on
        self.check_joint_controllers(whicharm)

        #check if there's actually something still in the gripper (if not, give up)
        if not self.check_grasp_successful(whicharm):
            rospy.loginfo("no object in gripper!  Stopping put-down")
            
            #detach the object and zero out held objects
            self.held_objects[whicharm] = 0
            self.detach_object(whicharm)
            return 1
   
        self.check_preempted()

        #point head at the place rectangle
        if point_head:
            self.point_head_at_place_rect()
        
        self.check_preempted()

        #detect objects and table (and clear out extra collision points)
        (objects, table) = self.call_tabletop_detection(update_table = update_table, update_place_rectangle = update_place_rectangle, clear_attached_objects = 0)
        
        self.check_preempted()

        #mark spots that are too close to detected objects as taken
        if self.held_objects[whicharm]:
            self.fill_in_taken_place_grid_spots(buffer = min(self.held_objects[whicharm].box_dims[0:2])+.03)
        else:
            self.fill_in_taken_place_grid_spots(buffer = padding)

        self.check_preempted()

        #how many times to do a regular place before trying the override place
        if max_place_tries == None and self.held_objects[whicharm] and not self.held_objects[whicharm].grasp_type == 'flat':
            place_tries_left = self.place_grid_resolution**2*2
        elif not self.held_objects[whicharm] or self.held_objects[whicharm].grasp_type == 'flat':
            place_tries_left = 0 #no object in the hand, or flat object, go directly to override place 
        else:
            place_tries_left = max_place_tries

        #try a bunch of times to place carefully
        success = 0
        z_offset = 0.01
        max_z_offset = 0.02
        while(place_tries_left > 0):
            rospy.loginfo("place_tries_left: %d"%place_tries_left)
            place_tries_left -= 1

            self.check_preempted()

            #check to make sure there's still something in the gripper (if not, give up)
            if not self.check_grasp_successful(whicharm):
                rospy.loginfo("no object in gripper!  Stopping put-down")

                #detach the object and zero out held objects
                self.held_objects[whicharm] = 0
                self.detach_object(whicharm)
                return 1

            #search for a place to put the object
            (i, j, pose, full) = self.choose_place_location(whicharm, -1, z_offset)

            #choose_place_location didn't find any suitable place locations, so raise z_offset (quit if already too high)
            if full == 1:
                if z_offset >= max_z_offset:
                    break

                #raise the z-offset and try again
                else:
                    z_offset += .01
                    padding = .01
                    rospy.loginfo("grid was full, raising the z_offset to %5.3f and lowering the padding"%z_offset)
                    self.fill_in_taken_place_grid_spots(buffer = padding)
                    (i, j, pose, full) = self.choose_place_location(whicharm, -1, z_offset)
                    
                    #all grid locations were 1, never mind
                    if full == 1:
                        rospy.loginfo("all grid locations were 1, moving on to last-resort place")
                        break

            self.check_preempted()

            #shift the object to be above the current detected table, if there is one
            if table:
                self.shift_place_pose_height(self.held_objects[whicharm], pose, table, z_offset)
            
            self.check_preempted()

            #try to place it there at one of several rotations
            inc = math.pi/4.
            rotationlist = [0.,]
            for mult in range(1, int(math.floor(math.pi/inc))+1):
                rotationlist.append(mult*inc)
                rotationlist.append(-mult*inc)

            success = 0
            for rotation in rotationlist:
                rotated_pose = self.rotate_pose(pose, rotation)
                self.draw_object_pose(self.held_objects[whicharm], rotated_pose, [0,1,1], 100)

                #call the place object service
                result = self.place_object(whicharm, rotated_pose, padding, constrained)
                if result == "SUCCESS":
                    success = 1
                    break
                elif result == "ERROR":
                    #for some reason, the retreat trajectory tends to time out when the object is dropped before placing
                    #which causes an error to be thrown (but we don't need to worry about it)
                    success = 1
                    #self.throw_exception()
                    break
                elif result == "FAILED":
                    rospy.logerr("place object returned failed, arm is probably stuck (opening the gripper and resetting map objects before moving back to the side)")
                    self.open_gripper(whicharm)
                    self.reset_collision_map()
                    success = 1
                    break
                elif result == "RETREAT_FAILED":
                    rospy.loginfo("place object returned retreat failed; doing an open-loop retreat")
                    wrist_pose = self.cms[whicharm].get_current_wrist_pose_stamped(frame = 'base_link')
                    wrist_mat = pose_to_mat(wrist_pose.pose)
                    shift = scipy.matrix(tf.transformations.translation_matrix([-.1, 0, 0]))
                    retreat_mat = wrist_mat * shift
                    retreat_pose = stamp_pose(mat_to_pose(retreat_mat), 'base_link')
                    self.move_cartesian_step(whicharm, retreat_pose, timeout = 5.0, \
                                                 settling_time = 3.0, blocking = 1)
                    success = 1
                    break
                elif result == "MOVE_ARM_STUCK":
                    rospy.loginfo("grasp object returned move arm stuck; resetting map objects and moving to side before trying next place")
                    self.reset_collision_map()
                    self.move_arm_to_side(whicharm, try_constrained = 1)

                self.check_preempted()

            #succeeded, stop
            if success:
                break

            #didn't succeed at any rotation, go on to the next place
            else:
                rospy.loginfo("marking place grid spot %d %d as colliding" %(i,j))
                self.place_grid[i][j] = -1


        #place failed, put it down open-loop at an override position
        if not success and use_place_override:
            self.check_preempted()

            for tries in range(self.place_grid_resolution**2*2):
                rospy.loginfo("place object override try number:%d"%tries)

                (i,j,pose,z_offset) = self.choose_place_location(whicharm, override_pose = tries)

                #try to place it there at one of several rotations
                rotationlist = [0, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2]
                for rotation in rotationlist:
                    rotated_pose = self.rotate_pose(pose, rotation)
                    if self.held_objects[whicharm]:
                        self.draw_object_pose(self.held_objects[whicharm], rotated_pose, [1,0,0], 100)

                    z_dist = .1 * (math.floor(tries/10.)+1)
                    success = self.place_object_override(whicharm, rotated_pose, z_dist)
                    if success:
                        #record that something was placed there
                        self.place_grid[i][j] = 1
                        break
                    else:
                        rospy.loginfo("Place object override failed!")
                if success:
                    break


        #if we managed to put it down, record that the hand is empty
        if success:
            rospy.loginfo("removing object from held_objects for the %s hand"%self.arm_dict[whicharm])
            self.held_objects[whicharm] = None
            self.reset_place_grid_temporary_collisions()

            #move the arm back out of the way
	    if move_to_side == True:
            	self.move_arm_to_side(whicharm)
            return 1

        else:
            rospy.loginfo("place failed")
            return 0


    ##grasp the object and check if there's something in the hand
    def grasp_object_and_check_success(self, object, whicharm = None):

        if not self.check_arms_to_use(whicharm):
            return("error", None)

        #try picking it up
        (grasp_result, arm_used, executed_grasp) = self.grasp_object(object, whicharm, 1, \
                                                      use_slip_detection = self.use_slip_detection)

        self.check_preempted()

        #if succeeded, record that the arm has an object 
        if grasp_result == "SUCCESS":

            #check if there's an object in the hand
            if self.check_grasp_successful(arm_used):

                self.held_objects[arm_used] = object
                object.grasp_pose = executed_grasp.grasp_pose
                object.grasp = executed_grasp
                rospy.loginfo("grasp succeeded, adding object to held_objects for the %s arm and moving it to the side"%self.arm_dict[arm_used])
                self.move_arm_to_side(arm_used, try_constrained = 1)

                return ("succeeded", arm_used)

            #if failed and we actually tried to grasp, detach the object and return that the attempt failed
            else:
                self.detach_object(arm_used)
                rospy.loginfo("grasp attempt failed and probably moved the object, detaching the object and returning")
                self.move_arm_to_side(arm_used, try_constrained = 1)                
                return ("attempt failed", None)

        #object grasper failed
        elif grasp_result == "FAILED":

            #open all the way
            self.open_gripper(arm_used)

            #move up to make sure we're not in collision, then move to the side
            up_goal = self.return_current_pose_as_list(arm_used)
            up_goal[2] += .1
            self.move_cartesian_step(arm_used, up_goal, blocking = 1)
            self.move_arm_to_side(arm_used)
            return ("attempt failed", None)

        #grasp succeeded, but lift failed, which we can do instead
        elif grasp_result == "LIFT_FAILED":

            if self.check_grasp_successful(arm_used):
                rospy.loginfo("just lift failed; lifting with Cartesian controllers")
                self.held_objects[arm_used] = object
                object.grasp_pose = executed_grasp.grasp_pose
                object.grasp = executed_grasp
                up_goal = self.return_current_pose_as_list(arm_used)
                up_goal[2] += .1
                self.move_cartesian_step(arm_used, up_goal, blocking = 1)

                #still something in the gripper after lifting, so we succeeded
                if self.check_grasp_successful(arm_used):
                    rospy.loginfo("grasp succeeded, adding object to held_objects for the %s arm"%self.arm_dict[arm_used])
                    self.move_arm_to_side(arm_used, try_constrained = 1)
                    return ("succeeded", arm_used)

            #dropped whatever it was, give up
            self.detach_object(arm_used)
            self.open_gripper(arm_used)
            self.move_arm_to_side(arm_used)
            rospy.loginfo("lift attempt failed, detaching object and returning to the side")
            return ("attempt failed", None)

        #infeasible grasp, but nothing was disturbed
        elif grasp_result == "UNFEASIBLE":
            return ("no feasible grasp", None)

        #start state was in collision even after trying to add allowed contact regions, better take a new map
        elif grasp_result == "MOVE_ARM_STUCK":
            rospy.loginfo("grasp object returned move arm stuck; resetting map objects and moving to side")
            self.reset_collision_map()
            self.move_arm_to_side(arm_used)
            return ("attempt failed", None)

        #grasper returned "ERROR", better ask a human for help
        else:
            self.throw_exception()
            return ("error", None) 


    ##refine an object detection by pointing the head at it (input the object's position on the table)
    def refine_object_detection(self, objectx, objecty):
        rospy.loginfo("refining object detection")
        self.point_head([objectx, objecty, self.table_height], 'base_link')
        (objects, table) = self.call_tabletop_detection(update_table = 0)

        #find the object closest to the original location and return it
        closest_object = None
        closest_object_dist = 1e6
        for new_object in objects:
            xdist = new_object.pose.pose.position.x-objectx
            ydist = new_object.pose.pose.position.y-objecty
            if (xdist**2+ydist**2) < closest_object_dist:
                closest_object_dist = xdist**2+ydist**2
                closest_object = new_object

        return closest_object


    ##distance between the stereo camera's center ray and a pose_stamped's center
    def pose_to_narrow_stereo_center_dist(self, pose_stamped):
        
        #convert the pose to the stereo camera frame
        transformed_pose = change_pose_stamped_frame(self.tf_listener, pose_stamped, self.stereo_camera_frame)

        #find the distance of the pose from the z-axis (dist in x-y plane)
        pose_dist = math.sqrt(transformed_pose.pose.position.x**2 + transformed_pose.pose.position.y**2)

        return pose_dist


    ##refine the view of the object, if it's too far from the center of the narrow stereo center
    def refine_if_not_centered(self, object):

        #if the object is too far away from where the head is pointing, point the head at it and re-detect first
        dist_from_point = self.pose_to_narrow_stereo_center_dist(object.pose)
        object_to_grasp = object
        rospy.loginfo("dist_from_point:%5.3f"%dist_from_point) 
        if dist_from_point > .1:
            refined_object = self.refine_object_detection(object.pose.pose.position.x, object.pose.pose.position.y)
            if refined_object != None:
                object_to_grasp = refined_object
                rospy.loginfo("refined object's collision name: %s"%refined_object.collision_name)
            else:
                rospy.logerr("refining object detection resulted in no object detections!  Adding old object back to collision map")
                self.collision_map_interface.add_collision_box(object.pose.pose, object.box_dims, object.pose.header.frame_id, object.collision_name)
        return object_to_grasp


    ##pick up the object_num-th nearest feasible object (tries each one in turn)
    #arms_to_try indicates which arms to try in what order (0=right, 1=left)
    def pick_up_nearest_object(self, object_num = 0, arms_to_try = [0,1]):

        #objects are already sorted by distance to edge of table
        for object in self.detected_objects[object_num:]:

            object_to_grasp = self.refine_if_not_centered(object)

            #try one arm and then the other
            for whicharm in arms_to_try:

                self.check_preempted()

                (result, arm_used) = self.grasp_object_and_check_success(object_to_grasp, whicharm)

                self.check_preempted()

                if result == "succeeded":
                    return (1, arm_used)
                elif result == "attempt failed":
                    return (0, None)

                #if we didn't succeed but didn't try to grasp, just continue
                elif result == "no feasible grasp":
                    rospy.loginfo("no feasible grasp for this object with the %s arm, trying the next"%self.arm_dict[whicharm])

        #didn't manage to pick up any object
        rospy.loginfo("no feasible grasps found for any object")
        return (0, None)


    ##open the gripper (whicharm is 0 for right, 1 for left)
    def open_gripper(self, whicharm):
        self.cms[whicharm].command_gripper(.1, -1, 1)


    ##close the gripper (whicharm is 0 for right, 1 for left)
    def close_gripper(self, whicharm):
        self.cms[whicharm].command_gripper(0, 100, 1)


    ##reset the collision map and take a new one (without tabletop or objects)
    def reset_collision_map(self):
        self.collision_map_interface.reset_collision_map()


    ##try to move to a set of joint angles using move_arm, ignoring current collisions if necessary, and if that fails, 
    #move open-loop
    def try_hard_to_move_joint(self, whicharm, trajectory, max_tries = 5, use_open_loop = 1):

        if not self.check_arms_to_use(whicharm):
            return 0

        reset_collision_map = 0
        for tries in range(max_tries):
            rospy.loginfo("try_hard_to_move_joint try number: %d"%tries)
            error_code = self.cms[whicharm].move_arm_joint(trajectory[-1], blocking = 1)

            if error_code == 1:
                rospy.loginfo("move arm reported success")
                return 1
            elif error_code == 0:
                if not reset_collision_map:
                    rospy.loginfo("problem with collision map!  Resetting collision objects")
                    self.reset_collision_map()
                    reset_collision_map = 1
                else:
                    rospy.loginfo("resetting collision map didn't work, move arm is still stuck!")
                    break
            rospy.loginfo("move arm reports failure with error code %d"%error_code)

        if use_open_loop:
            rospy.loginfo("moving open-loop to desired joint angles")
            self.cms[whicharm].command_joint_trajectory(trajectory, blocking = 1)
            return 1

        return 0

    ##move open-loop through trajectory
    def try_hard_to_move_joint_open_loop(self, whicharm, trajectory):

        if not self.check_arms_to_use(whicharm):
            return 0

        rospy.loginfo("moving open-loop to desired joint angles")
        self.cms[whicharm].command_joint_trajectory(trajectory, blocking = 1)
        return 1

    ##try to move to a pose using move_arm, ignoring current collisions if necessary, and if that fails, 
    #move open-loop (either to the IK solution if there is one, or using the Cartesian controllers if there isn't)
    def try_hard_to_move_pose(self, whicharm, pose_stamped, max_tries = 3, use_joint_open_loop = 0, use_cartesian = 0, try_constrained = 0):

        if not self.check_arms_to_use(whicharm):
            return 0

        #set the planning scene
        self.collision_map_interface.set_planning_scene()

        current_angles = self.cms[whicharm].get_current_arm_angles()
        start_angles = current_angles
        ik_solution = None

        #try max_tries times to make move_arm get us there
        for tries in range(max_tries):
            rospy.loginfo("try_hard_to_move_pose try number: %d"%tries)

            #look for a collision-free IK solution at the goal
            (solution, error_code) = self.cms[whicharm].ik_utilities.run_ik(pose_stamped, start_angles, \
                    self.cms[whicharm].ik_utilities.link_name)
            if solution:
                ik_solution = solution
            else:
                start_angles = self.cms[whicharm].ik_utilities.start_angles_list[tries]
                continue

            #try to keep the gripper approximately level while moving
            if try_constrained:
                location = get_xyz(pose_stamped.pose.position)
                current_pose = self.cms[whicharm].get_current_wrist_pose_stamped('torso_lift_link')
                orientation_constraint = self.get_keep_object_level_constraint(whicharm,current_pose)
                constraint = Constraints()
                constraint.orientation_constraints.append(orientation_constraint)
                result = self.try_to_move_constrained(whicharm,constraint,3,solution,location,pose_stamped.header.frame_id)
                #result = self.try_to_move_constrained(whicharm, 3, solution, location)
                if result == 1:
                    return 1

            #having found IK goal angles that are not in collision, try hard to get there
            success = self.try_hard_to_move_joint(whicharm, [solution,], 2, use_open_loop = 0)
            if success:
                rospy.loginfo("try_hard_to_move_joint reported success")
                return 1
            rospy.loginfo("try_hard_to_move_joint reported failure")
            start_angles = self.cms[whicharm].ik_utilities.start_angles_list[tries]

        #if we found an IK solution, move there open-loop, going to arm_above_and_to_side angles first
        if ik_solution and use_joint_open_loop:
            rospy.loginfo("ran out of try_hard_to_move_joint tries using move_arm, moving to the joint angles open-loop")
            self.cms[whicharm].command_joint_trajectory([self.arm_above_and_to_side_angles[whicharm], ik_solution], blocking = 1)
            return 1

        #no collision-free ik solution available, either use the Cartesian controllers or just return
        if use_cartesian:
            rospy.logerr("no IK solution found for goal pose!  Using Cartesian controllers to move")
            self.cms[whicharm].move_cartesian(pose_stamped, settling_time = rospy.Duration(10))

        return 0


    ##move to a nearby Cartesian pose using the Cartesian controllers
    def move_cartesian_step(self, whicharm, pose, timeout = 10.0, settling_time = 3.0, blocking = 0):
        if type(pose) == list:
            pose = create_pose_stamped(pose, 'base_link')
        self.cms[whicharm].move_cartesian(pose, blocking = blocking, \
                                   pos_thres = .003, rot_thres = .05, \
                                   timeout = rospy.Duration(timeout), \
                                   settling_time = rospy.Duration(settling_time))


    ##is an object in the hand? (is the gripper not closed all the way?)
    def check_grasp_successful(self, whicharm, min_gripper_opening = .0021, max_gripper_opening = .1):
        gripper_opening = self.cms[whicharm].get_current_gripper_opening()
        if gripper_opening > min_gripper_opening and gripper_opening < max_gripper_opening:
            rospy.loginfo("gripper opening acceptable: %5.3f"%gripper_opening)
            return 1
        rospy.loginfo("gripper opening unacceptable: %5.3f"%gripper_opening)
        return 0


    ##detect objects and try to pick up the nearest one
    def detect_and_pick_up_object(self, point_head_loc, frame = 'base_link', arms_to_try = [0,1], constrained = 0):

        #point the head at the desired location
        self.point_head(point_head_loc, frame)

        self.check_preempted()

        #detect objects and clear out extra collision points
        self.call_tabletop_detection(update_table = 0, clear_attached_objects = 1)

        #no objects left!  Quit
        if self.count_objects() == 0:
            rospy.loginfo("no objects in view!")
            return ("no objects left", None)

        self.check_preempted()

        #try to pick up the nearest object
        (success, arm_used) = self.pick_up_nearest_object(arms_to_try = arms_to_try)

        #succeeded
        if success:
            return ("succeeded", arm_used)
        else:
            return ("grasp failed", None)


    ##how many relevant objects did we detect?  
    def count_objects(self):
        return len(self.detected_objects)


    ##try to move to a location while constraining the gripper to remain approximately level
    def try_to_move_constrained(self, whicharm, constraint, max_tries = 3, start_angles = None, location = None, frame_id = 'torso_lift_link'):
        reset_collision_map = 0
        for tries in range(max_tries):
            rospy.loginfo("constrained move try number: %d"%tries)
            error_code = self.cms[whicharm].move_arm_constrained(constraint, start_angles, location, frame_id)
            if error_code == 1:
                rospy.loginfo("move arm constrained reported success")
                return 1
            rospy.loginfo("move arm constrained reports failure with error code %d"%error_code)
        return 0


    ##move whicharm off to the side
    def move_arm_to_side(self, whicharm, try_constrained = 0):

        #check if arms are already there
        current_arm_angles = self.cms[whicharm].get_current_arm_angles()
        desired_arm_angles = self.cms[whicharm].normalize_trajectory([self.arm_to_side_angles[whicharm]])[0]
        for (current_angle, desired_angle) in zip(current_arm_angles, desired_arm_angles):
            if math.fabs(current_angle-desired_angle) > 0.05:
                break
        else:
            rospy.loginfo("arm is already at the side")
            return 1

        #trying to keep the object orientation the same as current orientation
        if try_constrained:
            #default search-starting arm angles are arm-to-the-side
            if whicharm == 1:
                start_angles = [2.135, 0.803, 1.732, -1.905, 2.369, -1.680, 1.398]
            else:
                start_angles = [-2.135, 0.803, -1.732, -1.905, -2.369, -1.680, 1.398]
                     
            #default location is arm-to-the-side
            if whicharm == 1:
                rospy.loginfo("Planning for the left arm")
                location = [0.05, 0.65, -0.05]
            else:
                rospy.loginfo("Planning for the right arm")
                location = [0.05, -0.65, -0.05]

            current_pose = self.cms[whicharm].get_current_wrist_pose_stamped('torso_lift_link')
            orientation_constraint = self.get_keep_object_level_constraint(whicharm,current_pose)
            constraint = Constraints()
            constraint.orientation_constraints.append(orientation_constraint)
            result = self.try_to_move_constrained(whicharm,constraint,3,start_angles,location,'torso_lift_link')
            if result == 1:
                return 1

        #either constrained move didn't work or we didn't request a constrained move
        result = self.try_hard_to_move_joint(whicharm, [self.arm_above_and_to_side_angles[whicharm],self.arm_to_side_angles[whicharm]], use_open_loop = 1)
        return result


    ## create an OrientationConstraint to keep the object level
    def get_keep_object_level_constraint(self, whicharm, current_pose):
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = current_pose.header
        orientation_constraint.orientation = current_pose.pose.orientation
        orientation_constraint.type = OrientationConstraint.HEADER_FRAME
        orientation_constraint.link_name = self.cms[whicharm].ik_utilities.link_name
        orientation_constraint.absolute_roll_tolerance = 0.2
        orientation_constraint.absolute_pitch_tolerance = 0.2
        orientation_constraint.absolute_yaw_tolerance = math.pi
        orientation_constraint.weight = 1.0

        [roll,pitch,yaw] = posemath.fromMsg(current_pose.pose).M.GetRPY()
        if math.fabs(pitch - math.pi/2.0) < 0.3:
            orientation_constraint.absolute_roll_tolerance = math.pi
            rospy.loginfo("roll is unconstrained")
        return orientation_constraint


    ##move whicharm off to the side open-loop
    def move_arm_to_side_open_loop(self, whicharm, try_constrained = 0):
        
        result = self.try_hard_to_move_joint_open_loop(whicharm, [self.arm_above_and_to_side_angles[whicharm], \
                                                                      self.arm_to_side_angles[whicharm]])
        return result


    ##pick up the object nearest to a PointStamped with whicharm (0=right, 1=left)
    def pick_up_object_near_point(self, point_stamped, whicharm, refine_view = True):

        if not self.check_arms_to_use(whicharm):
            return 0

        #convert point to base_link frame
        base_link_point = point_stamped_to_list(self.tf_listener, point_stamped, 'base_link')

        #find the closest object
        nearest_dist = 1e6
        nearest_object_ind = None
        for (ind, object) in enumerate(self.detected_objects):
            (object_point, object_rot) = pose_stamped_to_lists(self.tf_listener, object.pose, 'base_link')
            dist = sum([(x-y)**2 for (x,y) in list(zip(object_point, base_link_point))])**.5
            rospy.loginfo("dist: %0.3f"%dist)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_object_ind = ind

        if nearest_object_ind != None:
            rospy.loginfo("nearest object ind: %d"%nearest_object_ind)
        else:
            rospy.logerr('No nearby objects. Unable to select grasp target')
            return 0

        #refine the view of the object if it's too far from the narrow stereo center
        if refine_view:
            object_to_grasp = self.refine_if_not_centered(self.detected_objects[nearest_object_ind])
        else:
            object_to_grasp = self.detected_objects[nearest_object_ind]
        
        #try to pick it up with the requested arm
        (result, arm_used) = self.grasp_object_and_check_success(object_to_grasp, whicharm)

        if result == "succeeded":
            return 1
        elif result == "attempt failed":
            return 0

        #if we didn't succeed but didn't try to grasp, just continue
        elif result == "no feasible grasp":
            rospy.loginfo("no feasible grasp for this object with the %s arm"%self.arm_dict[whicharm])
            return 0


    ##print the list of possible objects to grasp
    def print_object_list(self):

        #draw colors on the clusters and objects so we know which one is which, and print out the list of objects and their colors
        colors = [[1,0,0],
                  [0,1,0],
                  [0,0,1],
                  [0,1,1],
                  [1,0,1],
                  [1,1,0],
                  [1, 1, 1],
                  [.5, .5, .5],
                  [0, 0, 0]]
        colornames = ["red", "green", "blue", "cyan", "magenta", "yellow", "white", "grey", "black"]

        self.draw_functions.clear_rviz_points('grasp_markers')
        color_ind = 0
        for (ind, object) in enumerate(self.detected_objects):

            self.draw_object_pose(object, object.pose, colors[color_ind], ind)
            if object.type == 'mesh':
                rospy.loginfo("object %d, %s: recognized object with id %d and col name %s"%(ind, colornames[color_ind], object.object.potential_models[0].model_id, object.collision_name))
            else:
                rospy.loginfo("object %d, %s: point cluster with %d points and col name %s"%(ind, colornames[color_ind], len(object.object.cluster.points), object.collision_name))
#                 points = self.point_cloud_to_mat(cluster)
#                 self.draw_functions.draw_rviz_points(points, color = colors[color_ind], duration = 0., id = ind)
#                 time.sleep(.05) 
                
            color_ind = (color_ind + 1) % len(colors)


    ##check for a preempt request (overload this for your application)
    def check_preempted(self):
        pass


    ##saw a serious error (problem with service call), overload this for your application
    def throw_exception(self):
        pass


    ##pause for input
    def keypause(self):
        print "press enter to continue"
        input = raw_input()
        return input


    ##get the number of an object to pick up from the user
    def input_object_num(self):

        if not self.detected_objects:
            print "no objects detected!"
            return -1

        print "which object number?"
        input = self.keypause()
        try:
            object_ind = int(input)
        except:
            print "bad input: ", input
            return -1

        if object_ind > len(self.detected_objects):
            print "object number too high"
            return -1

        return object_ind


    ##get which arm/side to use from the user
    def input_side(self):
        c = raw_input()
        if c != 'r' and c != 'l':
            print "bad input: %s"%c
            return -1
        if c == 'r':
            whicharm = 0
            print "using right side"
        elif c == 'l':
            whicharm = 1
            print "using left side"
        return whicharm


    ##return the current pos and rot of the wrist for whicharm as a 7-list (pos, quaternion rot)
    def return_current_pose_as_list(self, whicharm):
        (pos, rot) = self.cms[whicharm].return_cartesian_pose()        
        return pos+rot


    ##print instructions for using extensions to the keyboard interface here
    def print_keyboard_extensions(self):
        print "pr, pl, or pc to change the place rectangle to be on the robot's right/left/center"


    ##add extensions to the keyboard interface here, return 1 to continue and 0 to go on down the possibilities
    def keyboard_extensions(self, input):
        if input == 'pr' or input == 'pl' or input == 'pc':
            y = 0
            if input == 'pr':
                y = -.3
            elif input == 'pl':
                y = .3

            self.place_rect_dims = [.3, .3]
            place_rect_pose_mat = scipy.matrix([[1.,0.,0.,self.table_front_edge_x+self.place_rect_dims[0]/2.+.1],
                                                [0.,1.,0.,y],
                                                [0.,0.,1.,self.table_height],
                                                [0.,0.,0.,1.]])
            self.place_rect_pose_stamped = stamp_pose(mat_to_pose(place_rect_pose_mat), 'base_link')
            self.draw_place_area()

        return 0


    def pick_side(self):
        if self.arms_to_use == "both":
            print "which arm?  r for right, l for left"
            whicharm = self.input_side()
        elif self.arms_to_use == "right":
            whicharm = 0
        elif self.arms_to_use == "left":
            whicharm = 1
        return whicharm


    ##keyboard interface for starting up the demo and overriding with manual commands
    def keyboard_interface(self):

        #set the current positions of the wrists as the current 'goals' relative to which Cartesian movements are expressed
        currentgoals = [0]*2
        for arm_ind in self.arms_to_use_list:
            currentgoals[arm_ind] = self.return_current_pose_as_list(arm_ind)

        #keyboard loop
        while not rospy.is_shutdown():

            print "type:"
            self.print_keyboard_extensions()
            if self.arms_to_use in ["right", "both"]:
                print "r to control the right arm"
            if self.arms_to_use in ["left", "both"]:    
                print "l to control the left arm"
            print "d to detect objects, da to detect and clear attached objects"
            print "dp to detect several possible models for each object, and clear attached objects"
            print "p to pick up an object"
            print "w to place the object in the place rectangle, wo to place the object where it came from"
            print "h to point the head at the current place rectangle and draw it"
            print "t to find the table"
            print "det to detach the object in the gripper"
            print "q to quit"

            input = self.keypause()

            #extensions to the interface get checked for here
            cont = self.keyboard_extensions(input)
            if cont:
                continue

            #detect objects
            if input == 'd':
                rospy.loginfo("detecting objects")
                self.call_tabletop_detection(update_table = 0, clear_attached_objects = 0)
            elif input == 'da':
                rospy.loginfo("detecting objects and clearing attached objects")
                self.call_tabletop_detection(update_table = 0, clear_attached_objects = 1)
            elif input == 'dp':
                rospy.loginfo("detecting several models for each object")
                self.call_tabletop_detection(update_table = 0, clear_attached_objects = 1, num_models = 5)

            #pick up an object
            elif input == 'p':

                self.print_object_list()
                object_num = self.input_object_num()
                if object_num == -1:
                    continue

                whicharm = self.pick_side()
                if whicharm not in [0,1]:
                    print "invalid arm"
                    continue

                (result, arm_used) = self.grasp_object_and_check_success(self.detected_objects[object_num], whicharm)

            #put down an object where it was originally picked up
            elif input == 'w' or input == 'wo':

                #if only one arm is holding the object, that's the one we mean
                if not any(self.held_objects):
                    print "the robot doesn't think it's holding any objects!\nPick an arm to do an open-loop place anyway: r for right and l for left"
                if self.held_objects[0] and not self.held_objects[1]:
                    whicharm = 0
                    print "only holding an object in the right arm, dropping the right arm object"
                elif self.held_objects[1] and not self.held_objects[0]:
                    whicharm = 1
                    print "only holding an object in the left arm, dropping the left arm object"
                else:
                    whicharm = self.pick_side()
                    if whicharm != 0 and whicharm != 1:
                        continue

                if input == 'wo':
                    if not self.held_objects[whicharm]:
                        print "no recorded pose for an object in that gripper!"
                        continue
                    self.place_object(whicharm, self.held_objects[whicharm].pose)
                else:
                    self.put_down_object(whicharm, use_place_override = 1)

            #point the head at the current place location
            elif input == 'h':
                rospy.loginfo("pointing the head at the current place location and drawing the place area")
                self.point_head_at_place_rect()
                self.draw_place_area()

            #find the table
            elif input == 't':
                rospy.loginfo("finding the table")
                self.find_table()

            #move an arm
            elif input == 'r' or input == 'l':
                if input == 'r':
                    if self.arms_to_use not in ["right", "both"]:
                        rospy.loginfo("right arm not allowed")
                        continue
                    whicharm = 0
                else:
                    if self.arms_to_use not in ["left", "both"]:
                        rospy.loginfo("left arm not allowed")
                        continue
                    whicharm = 1

                while not rospy.is_shutdown():
                    print "c to close, o to open the gripper"
                    print "u to go up, d to go down, rt to go right, lt to go left"
                    print "ay to go away from the robot, td to go toward the robot"
                    print "jm to go to arm-away-on-the-side angles using move_arm"
                    print "jc to go to arm-away-on-the-side angles while trying to constrain the pose"
                    print "j to go to arm-away-on-the-side angles open-loop using the joint trajectory action"
                    print "e to exit the arm-control menu for this arm (%s)"%input
                    c = self.keypause()

                    if c == 'c':
                        self.close_gripper(whicharm)
                    elif c == 'o':
                        self.open_gripper(whicharm)
                        if self.held_objects[whicharm]:
                            self.detach_object(whicharm)
                            self.held_objects[whicharm] = 0

                    elif c == 'jm':
                        #try moving collision-free using move_arm
                        self.move_arm_to_side(whicharm)
                        currentgoals[whicharm] = self.return_current_pose_as_list(whicharm)

                    elif c == 'jc':
                        #try moving collision-free using move_arm, keeping the pose constrained
                        self.move_arm_to_side(whicharm, try_constrained = 1)
                        currentgoals[whicharm] = self.return_current_pose_as_list(whicharm)

                    elif c == 'j':
                        #just move the arm open-loop
                        self.cms[whicharm].command_joint_trajectory([self.arm_above_and_to_side_angles[whicharm], self.arm_to_side_angles[whicharm]], blocking = 1)
                        currentgoals[whicharm] = self.return_current_pose_as_list(whicharm)

                    elif c == 'j2':
                        self.cms[whicharm].command_joint_trajectory([self.arm_to_side_angles[whicharm],], blocking = 1)
                        currentgoals[whicharm] = self.return_current_pose_as_list(whicharm)

                    elif c == 'u':
                        print "going up"
                        currentgoals[whicharm][2] += .1
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)
                    elif c == 'us':
                        print "going up a small amount"
                        currentgoals[whicharm][2] += .02
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)
                    elif c == 'd':
                        print "going down"
                        currentgoals[whicharm][2] -= .1
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)
                    elif c == 'ds':
                        print "going down a small amount"
                        currentgoals[whicharm][2] -= .02
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)
                    elif c == 'rt':
                        print "moving right"
                        currentgoals[whicharm][1] -= .02
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)
                    elif c == 'lt':
                        print "moving left"
                        currentgoals[whicharm][1] += .02
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)
                    elif c == 'ay':
                        print "moving away from robot"
                        currentgoals[whicharm][0] += .02
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)
                    elif c == 'td':
                        print "moving toward the robot"
                        currentgoals[whicharm][0] -= .02
                        self.move_cartesian_step(whicharm, currentgoals[whicharm], blocking = 1, timeout = 5.0)

                    elif c == 'e':
                        break

            elif input == 'det':
                print "detach the object in which gripper?  r for right and l for left"
                whicharm = self.input_side()
                if whicharm != None:
                    self.detach_object(whicharm)  

            elif input == 'q':
                return


            #update the current arm goals after each action
            for arm_ind in self.arms_to_use_list:
                currentgoals[arm_ind] = self.return_current_pose_as_list(arm_ind)


if __name__ == '__main__':

    rospy.init_node('pick_and_place_manager', anonymous=True)
    pick_and_place_manager = PickAndPlaceManager()
    pick_and_place_manager.keyboard_interface()
