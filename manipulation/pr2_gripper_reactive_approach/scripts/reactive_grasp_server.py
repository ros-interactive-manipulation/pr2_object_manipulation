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

## @package reactive_grasp_server
#Actions and services for reactive grasp, reactive approach, reactive lift, 
#compliant close, and grasp adjustment.

from __future__ import division
import roslib; roslib.load_manifest('pr2_gripper_reactive_approach')
import rospy
import pr2_gripper_reactive_approach.reactive_grasp as reactive_grasp
import pr2_gripper_reactive_approach.controller_manager as controller_manager
import sys
import math
from object_manipulation_msgs.msg import ReactiveGraspAction, ReactiveGraspResult, \
    ManipulationResult, GripperTranslation, ReactiveLiftAction, ReactiveLiftResult, \
    ReactivePlaceAction, ReactivePlaceResult, ReactiveGraspFeedback, \
    ReactiveLiftFeedback, ReactivePlaceFeedback, ManipulationPhase
from object_manipulator.convert_functions import *
from std_srvs.srv import Empty, EmptyResponse
import actionlib
import pdb
import time

##preempt exception
class Preempted(Exception): pass

##reactive grasp class with overloaded preempt
class ReactiveGrasperWithPreempt(reactive_grasp.ReactiveGrasper):
    
    def __init__(self, cm, rg_as, lift_as, approach_as, place_as):
        
        reactive_grasp.ReactiveGrasper.__init__(self, cm)

        #add the action servers so we can check for preempts and send feedback
        self._rg_as = rg_as
        self._lift_as = lift_as
        self._approach_as = approach_as
        self._place_as = place_as

        self.rg_state = "off"
        
        self.pause_for_recording = 0


    ##overloaded to add action preempts
    def check_preempt(self):
        
        if self.rg_state == "rg" and self._rg_as.is_preempt_requested() or \
                self.rg_state == "lift" and self._lift_as.is_preempt_requested() or \
                self.rg_state == "approach" and self._approach_as.is_preempt_requested() or \
                self.rg_state == "place" and self._place_as.is_preempt_requested():
            
            rospy.loginfo("preempt seen, state=%s"%self.rg_state)

            #switch back to joint controllers so we're not left in a bad state
            rospy.loginfo("switching back to joint controllers")
            self.cm.switch_to_joint_mode()
            
            raise Preempted

    ##overloaded to add feedback broadcasts
    def broadcast_phase(self, phase, send_feedback = 1):
        self._phase_pub.publish(phase)

        if self.rg_state == "rg":
            feedback = ReactiveGraspFeedback()
            feedback.manipulation_phase.phase = phase
            self._rg_as.publish_feedback(feedback)
        elif self.rg_state == "lift":
            feedback = ReactiveLiftFeedback()
            feedback.manipulation_phase.phase = phase
            self._lift_as.publish_feedback(feedback)
        elif self.rg_state == "place":
            feedback = ReactivePlaceFeedback()
            feedback.manipulation_phase.phase = phase
            self._place_as.publish_feedback(feedback)
        
        #if recording in the grasp playpen, pause at MOVING_TO_GRASP or PLACING
        #for the recorder to finish broadcasting
        if self.pause_for_recording and (phase == ManipulationPhase.MOVING_TO_GRASP \
                or phase == ManipulationPhase.PLACING):
            time.sleep(3)



##Class for the reactive grasp actions and services
class ReactiveGraspActionServer(object):
    _grasp_result = ReactiveGraspResult()
    _lift_result = ReactiveLiftResult()
    _place_result = ReactivePlaceResult()

    def __init__(self, which_arm): #which_arm is 'r' or 'l'
        self.which_arm = which_arm

        self._node_name = which_arm+'_reactive_grasp'
        if which_arm == 'r':
            self._action_name = 'reactive_grasp/right'
            self._lift_action_name = 'reactive_lift/right'
            self._approach_action_name = 'reactive_approach/right'
            self._place_action_name = 'reactive_place/right'
        else:
            self._action_name = 'reactive_grasp/left'
            self._lift_action_name = 'reactive_lift/left'
            self._approach_action_name = 'reactive_approach/left'
            self._place_action_name = 'reactive_place/left'

        #params for reactive approach/grasp
        self.side_step = rospy.get_param("~side_step", .015)
        self.back_step = rospy.get_param("~back_step", .03)
        self.approach_num_tries = rospy.get_param("~approach_num_tries", 5)
        self.goal_pos_thres = rospy.get_param("~goal_pos_thres", .01)

        #params for reactive grasp/grasp adjustment
        self.min_contact_row = rospy.get_param("~min_contact_row", 1)
        self.min_gripper_opening = rospy.get_param("~min_gripper_opening", .0021)
        self.max_gripper_opening = rospy.get_param("~max_gripper_opening", .1) 
        self.grasp_adjust_x_step = rospy.get_param("~grasp_adjust_x_step", .02)
        self.grasp_adjust_z_step = rospy.get_param("~grasp_adjust_z_step", .015)
        self.grasp_adjust_num_tries = rospy.get_param("~grasp_adjust_num_tries", 3)

        #params for reactive grasp 
        self.grasp_num_tries = rospy.get_param("~grasp_num_tries", 2)
        self.forward_step = rospy.get_param("~forward_step", 0)  #select based on top/side
        self.close_force = rospy.get_param("~close_force", 100)
        self.use_slip_controller = rospy.get_param("~use_slip_controller", 0)
        self.use_slip_detection = rospy.get_param("~use_slip_detection", 0)

        #params for reactive place
        self.place_overshoot = rospy.get_param("~place_overshoot", .01)
        self.place_angle_max_diff = rospy.get_param("~place_angle_max_diff", math.pi/10.)

        #params for reactive lift
        self.shake_object = rospy.get_param("~shake_object", 0)

        rospy.loginfo("reactive_grasp_server: using_slip_controller:"+str(self.use_slip_controller))
        rospy.loginfo("reactive_grasp_server: using_slip_detection:"+str(self.use_slip_detection))

        #param for recording in grasp_playpen
        self.pause_for_recording = rospy.get_param("~pause_for_recording", 0)

        if self.pause_for_recording:
            rospy.loginfo("pausing for recording is turned on")

        #start action servers for reactive grasp, lift, and approach
        self._as = actionlib.SimpleActionServer(self._action_name, ReactiveGraspAction, \
                                                    execute_cb=self.reactive_grasp_cb)
        self._lift_as = actionlib.SimpleActionServer(self._lift_action_name, ReactiveLiftAction, \
                                                         execute_cb=self.reactive_lift_cb)
        self._approach_as = actionlib.SimpleActionServer(self._approach_action_name, ReactiveGraspAction, \
                                                             execute_cb = self.reactive_approach_cb)
        self._place_as = actionlib.SimpleActionServer(self._place_action_name, ReactivePlaceAction, \
                                                          execute_cb = self.reactive_place_cb)

        #advertise services for compliant grasp and grasp adjustment
        rospy.Service(self._node_name+'/compliant_close', \
                        Empty, self.compliant_close_callback)
        rospy.Service(self._node_name+'/grasp_adjustment', \
                          Empty, self.grasp_adjustment_callback)

        #initialize controler manager and reactive grasper
        self.cm = controller_manager.ControllerManager(which_arm, \
                               using_slip_controller = self.use_slip_controller, \
                               using_slip_detection = self.use_slip_detection)
        self.rg = ReactiveGrasperWithPreempt(self.cm, self._as, self._lift_as, self._approach_as, self._place_as)

        #shove the flag into the reactive grasper
        self.rg.pause_for_recording = self.pause_for_recording


    ##do a reactive grasp using the fingertip sensors
    #(backs up and moves to the side if a fingertip contacts on the way to the grasp,
    #closes compliantly, tries approach and grasp again if gripper opening is not within bounds)
    def reactive_grasp_cb(self, goal):

        rospy.loginfo("got reactive grasp request")
        try:
            self.rg_state = "rg"
            (trajectory, object_name, table_name, forward_step) = self.init_reactive_grasp(goal)

            #perform the reactive grasp
            self.cm.switch_to_cartesian_mode()
            if goal.max_contact_force > 0:
                self.rg.close_force = goal.max_contact_force
            result = self.rg.reactive_grasp(None, goal.final_grasp_pose, trajectory, \
                              self.side_step, self.back_step, \
                              self.approach_num_tries, self.goal_pos_thres, \
                              self.min_gripper_opening, self.max_gripper_opening, \
                              self.grasp_num_tries, forward_step, self.min_contact_row, \
                              object_name, table_name, \
                              self.grasp_adjust_x_step, self.grasp_adjust_z_step, self.grasp_adjust_num_tries)
            self.rg.check_preempt()

            rospy.loginfo("switching back to joint controllers")
            self.cm.switch_to_joint_mode()
            self.rg_state = "off"

            if result == 0:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
                self._grasp_result.manipulation_result.value = ManipulationResult.SUCCESS
                self._as.set_succeeded(self._grasp_result)
            else:
                #stopped reporting 'failed' just in case we're grasping a super-thin object--it's up to the next-level-up app to decide whether we succeeded
                self.rg.broadcast_phase(ManipulationPhase.FAILED, send_feedback = 0)
                #self._grasp_result.manipulation_result.value = ManipulationResult.FAILED
                self._grasp_result.manipulation_result.value = ManipulationResult.SUCCESS
                #self._as.set_aborted(self._grasp_result)
                self._as.set_succeeded(self._grasp_result)

        except Preempted:
            rospy.loginfo("reactive grasp preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.FAILED       
            self._as.set_preempted(self._grasp_result)

        except reactive_grasp.Aborted:
            rospy.loginfo("reactive grasp saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.ERROR
            self._as.set_aborted(self._grasp_result)


    ##pull relevant parts out of the goal for both reactive grasp and approach
    def init_reactive_grasp(self, goal):

        self.rg.check_preempt()

        self.rg.pressure_listener.set_thresholds()

        self.rg.check_preempt()

        #check the param server for a new closing force and pass it on
        self.close_force = rospy.get_param("~close_force", 100)
        self.rg.close_force = self.close_force
        rospy.loginfo("using a close_force of %d"%self.close_force)

        #pull trajectory angles out of JointTrajectory
        if len(goal.trajectory.points) > 0:
            trajectory = [goal.trajectory.points[i].positions for i in range(len(goal.trajectory.points))]
            print "trajectory:"
            for angles in trajectory:
                print pplist(angles)
        else:
            trajectory = None

        object_name = "points"
        table_name = goal.collision_support_surface_name

        #if forward_step is 0, adjust the forward_step depending on whether the grasp is a top or side grasp
        #if the palm's x-axis's z-component is high enough, it's a top grasp
        if self.forward_step == 0:
            forward_step = .06

            #transform pose to base_link
            bl_pose = change_pose_stamped_frame(self.rg.cm.tf_listener, goal.final_grasp_pose, 'base_link')
            palm_mat = pose_to_mat(bl_pose.pose)  
            if palm_mat[2,0] < -math.cos(math.pi/3):
                rospy.loginfo("palm_mat[2,0] = %0.3f, top grasp"%palm_mat[2,0])

                #if it's a top grasp, lower the forward_step
                forward_step = .03
            else:
                rospy.loginfo("side grasp")
        else:
            forward_step = self.forward_step

        self.rg.check_preempt()

        return (trajectory, object_name, table_name, forward_step)


    ##do a reactive approach using the fingertip sensors
    #(backs up and moves to the side if a fingertip contacts on the way to the grasp)
    def reactive_approach_cb(self, goal):

        rospy.loginfo("got reactive approach request")
        try:
            self.rg_state = "approach"
            (trajectory, object_name, table_name, forward_step) = self.init_reactive_grasp(goal)

            #compute the approach dir using the current wrist pose
            current_wrist_pose = self.rg.cm.get_current_wrist_pose_stamped()
            approach_dir = self.rg.compute_approach_dir(current_wrist_pose, goal.final_grasp_pose)

            #perform the reactive approach
            self.cm.switch_to_cartesian_mode()
            result = self.rg.reactive_approach(approach_dir, goal.final_grasp_pose, trajectory, \
                              self.side_step, self.back_step, \
                              self.approach_num_tries, self.goal_pos_thres)
            self.rg_state = "off"
            self.rg.check_preempt()

            rospy.loginfo("switching back to joint controllers")
            self.cm.switch_to_joint_mode()

            if result == 0:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
                self._grasp_result.manipulation_result.value = ManipulationResult.SUCCESS
                self._approach_as.set_succeeded(self._grasp_result)
            else:
                self.rg.broadcast_phase(ManipulationPhase.FAILED, send_feedback = 0)
                self._grasp_result.manipulation_result.value = ManipulationResult.FAILED
                self._approach_as.set_aborted(self._grasp_result)

        except Preempted:
            rospy.loginfo("reactive grasp preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.FAILED       
            self._approach_as.set_preempted(self._grasp_result)

        except reactive_grasp.Aborted:
            rospy.loginfo("reactive grasp saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._grasp_result.manipulation_result.value = ManipulationResult.ERROR
            self._approach_as.set_aborted(self._grasp_result)


    ##do a reactive lift using the fingertip sensors (uses slip-servoing to exert just enough force to grasp and lift)
    def reactive_lift_cb(self, goal):

        try:
            rospy.loginfo("got reactive lift request")
            self.cm.switch_to_cartesian_mode()

            #lift and check if the object is still within the hand
            self.rg_state = "lift"
            success = self.rg.lift_carefully(goal.lift, self.min_gripper_opening, self.max_gripper_opening)
            
            #optional shake (used to test the quality of the grasp)
            if success and self.shake_object:

                #slow shake
                success = self.rg.shake_object(None, self.min_gripper_opening,
                                               self.max_gripper_opening,
                                               wrist_roll_shake_time = 1.5,
                                               wrist_flex_shake_time = 1.5,
                                               arm_pan_shake_time = 0.25)
#                 if success:

#                     #fast shake
#                     success = self.rg.shake_object(None, self.min_gripper_opening,
#                                                    self.max_gripper_opening,
#                                                    wrist_roll_shake_time = 0.5,
#                                                    wrist_flex_shake_time = 0.5,
#                                                    arm_pan_shake_time = 0.1)

            self.rg_state = "off"

            rospy.loginfo("switching back to joint controllers")
            self.cm.switch_to_joint_mode()

            if success:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
                self._lift_result.manipulation_result.value = ManipulationResult.SUCCESS
                self._lift_as.set_succeeded(self._lift_result)
            else:
                self.rg.broadcast_phase(ManipulationPhase.FAILED, send_feedback = 0)
                self._lift_result.manipulation_result.value = ManipulationResult.FAILED
                self._lift_as.set_aborted(self._lift_result)
            
        except Preempted:
            rospy.loginfo("reactive lift preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._lift_result.manipulation_result.value = ManipulationResult.FAILED       
            self._lift_as.set_preempted(self._lift_result)

        except reactive_grasp.Aborted:
            rospy.loginfo("reactive lift saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._lift_result.manipulation_result.value = ManipulationResult.ERROR
            self._lift_as.set_aborted(self._lift_result)


    ##do a reactive place using the fingertip sensors and accelerometer 
    #(uses slip controller to stop and open when the object hits the table)
    #if slip controller is not running, just goes past the goal with the Cartesian controller
    def reactive_place_cb(self, goal):
        
        try:
            rospy.loginfo("got reactive place request")
            self.cm.switch_to_cartesian_mode()

            #place and open when the object hits the table
            self.rg_state = "place"
            result = self.rg.place_carefully(goal.final_place_pose, self.place_overshoot, \
                                 self.min_gripper_opening, self.max_gripper_opening, \
                                 self.place_angle_max_diff)

            self._place_result.manipulation_result.value = ManipulationResult.SUCCESS
            if result == 1:
                self.rg.broadcast_phase(ManipulationPhase.SUCCEEDED, send_feedback = 0)
            self._place_as.set_succeeded(self._place_result)

        except Preempted:
            rospy.loginfo("reactive place preempted, returning")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._place_result.manipulation_result.value = ManipulationResult.FAILED
            self._place_as.set_preempted(self._place_result)
        
        except reactive_grasp.Aborted:
            rospy.loginfo("reactive place saw a serious error!  Aborting")
            self.rg.broadcast_phase(ManipulationPhase.ABORTED, send_feedback = 0)
            self._place_result.manipulation_result.value = ManipulationResult.ERROR
            self._place_as.set_aborted(self._place_result)


    ##do a compliant close using the fingertip sensors (stops at the first detected 
    #contact and moves the arm so that the contact stays in place)
    def compliant_close_callback(self, req):
        rospy.loginfo("executing compliant grasp--switching to Cartesian controllers")
        self.cm.switch_to_cartesian_mode()

        self.rg.compliant_close()

        rospy.loginfo("switching back to joint controllers")
        self.cm.switch_to_joint_mode()
        
        return EmptyResponse()
    

    ##do a grasp adjustment using the fingertip sensors (tries to move the hand to
    #obtain non-tip and non-edge contacts)
    def grasp_adjustment_callback(self, req):
        rospy.loginfo("executing grasp adjustment--switching to Cartesian controllers")
        self.cm.switch_to_cartesian_mode()

        #start by opening the hand, restting the fingertips, and doing a compliant close 
        #to get proper fingertip readings
        self.rg.open_and_reset_fingertips()
        (initial_l_readings, initial_r_readings) = self.rg.compliant_close()

        #get the current gripper pose as the 'desired grasp pose'
        current_wrist_pose = self.cm.get_current_wrist_pose_stamped()

        #run grasp adjustment
        (adjust_result, current_goal_pose) = self.rg.adjust_grasp(current_wrist_pose, \
                    initial_l_readings, initial_r_readings, \
                    z_step = self.grasp_adjust_z_step, x_step = self.grasp_adjust_x_step, \
                    min_gripper_opening = self.min_gripper_opening, \
                    max_gripper_opening = self.max_gripper_opening, \
                    min_contact_row = self.min_contact_row, 
                    num_tries = self.grasp_adjust_num_tries)

        rospy.loginfo("switching back to joint controllers")
        self.cm.switch_to_joint_mode()

        return EmptyResponse()
        

if __name__ == "__main__":
    
    if len(sys.argv) < 2 or sys.argv[1] != 'r' and sys.argv[1] != 'l':
        rospy.logerr("usage: reactive_grasp_server.py which_arm (which_arm is r or l)")
        sys.exit(1)

    which_arm = sys.argv[1]

    node_name = which_arm+'_reactive_grasp'
    rospy.init_node(node_name, anonymous=True)

    reactive_grasp_services = ReactiveGraspActionServer(which_arm)
    rospy.loginfo("Reactive grasp action ready")

    rospy.spin()
