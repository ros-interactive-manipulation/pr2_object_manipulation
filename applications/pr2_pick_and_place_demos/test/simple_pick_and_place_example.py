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

"""
Simple example of using the pick_and_place_manager to pick up an object
near a specified point, and to put it back down in a desired region.
Launch pr2_tabletop_manipulation_launch pr2_tabletop_manipulation.launch 
before running.  Works better with fingertip sensors installed.
"""

import roslib
roslib.load_manifest('pr2_pick_and_place_demos')
import rospy
from pr2_pick_and_place_demos.pick_and_place_manager import *
from object_manipulator.convert_functions import *

class SimplePickAndPlaceExample():

    def __init__(self):

        rospy.loginfo("initializing pick and place manager")
        self.papm = PickAndPlaceManager()
        rospy.loginfo("finished initializing pick and place manager")


    #pick up the nearest object to PointStamped target_point with whicharm 
    #(0=right, 1=left)
    def pick_up_object_near_point(self, target_point, whicharm):
        
        rospy.loginfo("moving the arms to the side")
        self.papm.move_arm_to_side(0)  #right arm
        self.papm.move_arm_to_side(1)  #left arm

        rospy.loginfo("pointing the head at the target point")
        self.papm.point_head(get_xyz(target_point.point),
                             target_point.header.frame_id)

        rospy.loginfo("detecting the table and objects")
        self.papm.call_tabletop_detection(update_table = 1, update_place_rectangle = 1, 
                             clear_attached_objects = 1)     

        rospy.loginfo("picking up the nearest object to the target point")
        success = self.papm.pick_up_object_near_point(target_point, 
                                                      whicharm)
        
        if success:
            rospy.loginfo("pick-up was successful!  Moving arm to side")
            self.papm.move_arm_to_side(whicharm)
        else:
            rospy.loginfo("pick-up failed.")

        return success

    
    #place the object held in whicharm (0=right, 1=left) down in the 
    #place rectangle defined by place_rect_dims (x,y) 
    #and place_rect_center (PoseStamped)
    def place_object(self, whicharm, place_rect_dims, place_rect_center):

        self.papm.set_place_area(place_rect_center, place_rect_dims)
        
        rospy.loginfo("putting down the object in the %s gripper"\
                      %self.papm.arm_dict[whicharm])
        success = self.papm.put_down_object(whicharm, 
                      max_place_tries = 25,
                      use_place_override = 1)

        if success:
            rospy.loginfo("place returned success")
        else:
            rospy.loginfo("place returned failure")

        return success


if __name__ == "__main__":
    rospy.init_node('simple_pick_and_place_example')
    sppe = SimplePickAndPlaceExample()

    #adjust for your table 
    table_height = .72                                          

    #.5 m in front of robot, centered
    target_point_xyz = [.5, 0, table_height-.05]                
    target_point = create_point_stamped(target_point_xyz, 'base_link')
    success = sppe.pick_up_object_near_point(target_point, 0)   #right arm

    if success:

        #square of size 30 cm by 30 cm
        place_rect_dims = [.3, .3]                              

        #.5 m in front of robot, to the right
        center_xyz = [.5, -.15, table_height-.05]               

        #aligned with axes of frame_id
        center_quat = [0,0,0,1]                                 
        place_rect_center = create_pose_stamped(center_xyz+center_quat,
                                                    'base_link')

        sppe.place_object(0, place_rect_dims, place_rect_center)
    
    
