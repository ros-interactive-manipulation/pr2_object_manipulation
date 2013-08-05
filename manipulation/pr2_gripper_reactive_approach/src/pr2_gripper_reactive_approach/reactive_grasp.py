#!/usr/bin/env python
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

## @package reactive_grasp
#Reactive grasping based on fingertip readings

from __future__ import division
import roslib
roslib.load_manifest('pr2_gripper_reactive_approach')
import time
import rospy
import os
import controller_manager
import sys
from arm_navigation_msgs.msg import OrderedCollisionOperations, CollisionOperation
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from object_manipulation_msgs.msg import GripperTranslation, ManipulationPhase
import tf
import scipy
import math
import pdb
import hand_sensor_listeners
from object_manipulator.convert_functions import *
import copy
from actionlib_msgs.msg import GoalStatus

##pause for input
def keypause():
  print "press enter to continue"
  input = raw_input()
  return input

##abort exception
class Aborted(Exception): pass

##reactive/guarded movement and grasping
class ReactiveGrasper:

  def __init__(self, cm):

    self.cm = cm
    self.whicharm = self.cm.whicharm
    print "initializing %s fingertip sensor listener"%self.whicharm
    self.pressure_listener = hand_sensor_listeners.FingertipSensorListener(self.whicharm)
    #self.palm_listener = hand_sensor_listeners.PalmSensorListener()
    print "done"

    #are we using slip detection?
    self.using_slip_detection = self.cm.using_slip_detection

    #force to use when closing
    self.close_force = 50
    if self.using_slip_detection:
      self.close_force = 10

    #has the hand been closed hard recently?
    self._closed_hard = 0

    #collision name of support surface
    self._table_name = "table"

    #for taking photos of each step
    self._photo = 0

    #broadcast the current phase of the manipulation
    self._phase_pub = rospy.Publisher('/reactive_manipulation_phase', ManipulationPhase)

    #dictionary for ManipulationPhase
    self.manipulation_phase_dict = {}
    for element in dir(ManipulationPhase):
        if element[0].isupper():
            self.manipulation_phase_dict[eval('ManipulationPhase.'+element)] = element

    #dictionary of return values for reactive approach
    self.reactive_approach_result_dict = {"success":0, "within goal threshold":1, "both fingers touching":2, "ran out of tries":3, "touching table":4, "saw palm contact":5, "grasp infeasible":6}

    #dictionary of return values for reactive_grasp
    self.reactive_grasp_result_dict = {"success":0, "ran out of grasp tries":1, "ran out of approach tries":2, \
                                         "aborted":3, "grasp infeasible":4}

    rospy.loginfo("done with ReactiveGrasper init for the %s arm"%self.whicharm)


  ##broadcast the current manipulation phase (of type ManipulationPhase)
  def broadcast_phase(self, phase):
    rospy.loginfo("broadcasting reactive phase %s"%self.manipulation_phase_dict[phase])
    self._phase_pub.publish(phase)
    

  ##check for a preempt (overload for your application)
  def check_preempt(self):
    pass


  ##throw an abort exception 
  def throw_exception(self):
    raise Aborted


  ##check the state of find_gripper_contact and try to collect the first positive thresholded fingertip readings
  #only looks for front contacts
  def get_fingertip_readings_during_find_contact(self, contacts_desired, timeout = 10.):
    saved_l_pressure = [0.]*22
    saved_r_pressure = [0.]*22
    start_time = rospy.get_rostime()
    left_touching = right_touching = left_force = right_force = 0.

    while not rospy.is_shutdown():
      state = self.cm.get_find_gripper_contact_state()

      #action finished
      if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:

        #get the end result, if successful
        if state == GoalStatus.SUCCEEDED:
          (left_touching, right_touching, left_force, right_force) = self.cm.get_find_gripper_contact_result()
          rospy.loginfo("left_touching: %d, left_force: %0.3f, right_touching: %d, right_force: %0.3f"\
                        %(left_touching, left_force, right_touching, right_force))
        else:
          rospy.loginfo("GoalStatus was not SUCCEEDED; state=%d"%state)
        break

      #timeout reached
      if rospy.get_rostime() - start_time > rospy.Duration(timeout):
        rospy.loginfo("timeout reached on find_gripper_contact")
        break

      #check the current thresholded readings and save the first ones seen for each fingertip
      (l_pressure, r_pressure) = self.pressure_listener.get_thresholded_readings()
      if any(l_pressure[7:22]) and not any(saved_l_pressure[7:22]):
        rospy.loginfo("saving l_pressure:"+pplist(l_pressure))
        saved_l_pressure = l_pressure
      if any(r_pressure[7:22]) and not any(saved_r_pressure[7:22]):
        rospy.loginfo("saving r_pressure:"+pplist(r_pressure))
        saved_r_pressure = r_pressure

      time.sleep(.02)

    #our desired fingertip readings haven't tripped yet; wait a bit longer and see if some readings show up
    if contacts_desired == "either" and (not any(saved_l_pressure[7:22]) and not any(saved_r_pressure[7:22])) or \
          contacts_desired == "both" and (not any(saved_l_pressure[7:22]) or not any(saved_r_pressure[7:22])):
      start_time = rospy.get_rostime()
      while not rospy.is_shutdown():
        
        #timeout reached
        if rospy.get_rostime() - start_time > rospy.Duration(1.0):
          rospy.loginfo("timed out on extra second of waiting for readings")
          break

        #check the current thresholded readings and save the first ones seen for each fingertip
        (l_pressure, r_pressure) = self.pressure_listener.get_thresholded_readings()
        if any(l_pressure[7:22]) and not any(saved_l_pressure[7:22]):
          rospy.loginfo("saving l_pressure:"+pplist(l_pressure))
          saved_l_pressure = l_pressure
        if any(r_pressure[7:22]) and not any(saved_r_pressure[7:22]):
          rospy.loginfo("saving r_pressure:"+pplist(r_pressure))
          saved_r_pressure = r_pressure
        if contacts_desired == "either" and (any(saved_l_pressure[7:22]) or any(saved_r_pressure[7:22])):
          break
        if contacts_desired == "both" and (any(saved_l_pressure[7:22]) and any(saved_r_pressure[7:22])):
          break
        time.sleep(.02)

    #if there's still a discrepancy in contacts, figure out which front contact is closest 
    #to being triggered and set that to 1
    (rel_l_pressure, rel_r_pressure) = self.pressure_listener.get_relative_readings()
    if left_touching and not any(saved_l_pressure[7:22]):
      most_likely_on = 7+rel_l_pressure[7:22].index(max(rel_l_pressure[7:22]))
      saved_l_pressure[most_likely_on] = 1
      rospy.loginfo("left_touching but not any saved_l_pressure, fixing")
    if right_touching and not any(saved_r_pressure[7:22]):
      most_likely_on = 7+rel_r_pressure[7:22].index(max(rel_r_pressure[7:22]))
      saved_r_pressure[most_likely_on] = 1
      rospy.loginfo("right_touching but not any saved_r_pressure, fixing")

    return (saved_l_pressure, saved_r_pressure, left_touching, right_touching, left_force, right_force)


  ##close compliantly (controller manager should be in Cartesian mode)
  #(if one finger touches first, counter-move the arm to keep that finger in-place while closing)
  def compliant_close(self):
    self.check_preempt()

    self.broadcast_phase(ManipulationPhase.CLOSING)
    self.cm.check_controllers_ok('cartesian')

    use_slip_controller_to_close = self.using_slip_detection
    #use_slip_controller_to_close = 0

    step_time = .05
    left_touching = right_touching = 0

    #ask the gripper to close until first contact
    if use_slip_controller_to_close:
      rospy.loginfo("finding first gripper contact")
      self.cm.find_gripper_contact(contacts_desired = "either", zero_fingertips = 1, blocking = 0, timeout = 10.)
      (l_pressure, r_pressure, left_touching, right_touching, left_force, right_force) = \
          self.get_fingertip_readings_during_find_contact(contacts_desired = "either")
    else:
      #close slowly until first front contact (or until the gripper gets to 1 cm open)
      desired_pos = self.cm.get_current_gripper_opening()
      while(desired_pos > .01):
        self.cm.command_gripper(desired_pos, self.close_force)
        (l_pressure, r_pressure) = self.pressure_listener.get_thresholded_readings()
        if any(l_pressure):
          left_touching = 1
          break
        if any(r_pressure):
          right_touching = 1
          break
        desired_pos -= .001
        time.sleep(step_time)

    self.check_preempt()

    #if we saw both contacts at once, skip this part
    if not (left_touching and right_touching):

      #if one contact was seen, change the Cartesian control frame to the contacting finger
      reloaded_controllers = 0
      if left_touching or right_touching:

        if not use_slip_controller_to_close:
          #stop the gripper in a hurry
          desired_pos += .002
          self.cm.command_gripper(desired_pos, 200)

        #move the cartesian arm control frame to the appropriate fingertip frame
        if self.cm.use_trajectory_cartesian:
          print "saw first contact!  Reloading Cartesian controllers"
          if left_touching:
            self.cm.cart_params.tip_frame = self.whicharm+'_gripper_l_finger_tip_link'
          else:
            self.cm.cart_params.tip_frame = self.whicharm+'_gripper_r_finger_tip_link'
          self.cm.cart_params.set_params_to_defaults(set_tip_frame = 0, set_params_on_server = 1)
          self.cm.reload_cartesian_controllers(set_params = 0)
          reloaded_controllers = 1
          print "done reloading Cartesian controllers"

        if self._photo:
          keypause()

      self.check_preempt()

      #continue closing the gripper slowly until both sensors contact, or until the gripper's closed
      if use_slip_controller_to_close:
        rospy.loginfo("finding second gripper contact")
        contacts_desired = "both"
        self.cm.find_gripper_contact(contacts_desired, zero_fingertips = 0, blocking = 0, timeout = 10.)      
        (l_pressure, r_pressure, left_touching, right_touching, left_force, right_force) = \
            self.get_fingertip_readings_during_find_contact(contacts_desired = "both")

      else:
        left_touching = right_touching = 0
        while(desired_pos > 0):
          self.cm.command_gripper(desired_pos, self.close_force)
          (l_pressure, r_pressure) = self.pressure_listener.get_thresholded_readings()
          if any(l_pressure):
            left_touching = 1
          if any(r_pressure):
            right_touching = 1
          if left_touching and right_touching:
            print "saw both left and right contacts, stopping"
            break
          desired_pos -= .001
          time.sleep(step_time)

      if self._photo:
        keypause()

      self.check_preempt()

      #put the cartesian control frame back to the normal gripper frame
      if reloaded_controllers:
        print "putting control frame back"
        self.cm.cart_params.set_params_to_gentle(set_tip_frame = 1, set_params_on_server = 1)
        self.cm.reload_cartesian_controllers(set_params = 0)
        print "done reloading Cartesian controllers"

      self.check_preempt()

    return (l_pressure, r_pressure)


  ##move to a Cartesian pose goal
  def move_cartesian_step(self, pose, timeout = 10.0, settling_time = 3.0, blocking = 0, pos_thres = .0025, rot_thres = .05):
    self.check_preempt()

    if type(pose) == list:
      pose = create_pose_stamped(pose, 'base_link')
    self.cm.move_cartesian(pose, blocking = blocking, \
                           pos_thres = pos_thres, rot_thres = rot_thres, \
                           timeout = rospy.Duration(timeout), \
                           settling_time = rospy.Duration(settling_time))


  ##check for tip/side/back/palm contact
  def check_guarded_move_contacts(self):
    
    left_touching = right_touching = palm_touching = 0

    #regions are (tip, plus_z_side, neg_z_side, front, back)
    (l_regions_touching, r_regions_touching) = self.pressure_listener.regions_touching()
    if self.palm_touching():
      palm_touching = 1
      rospy.loginfo("saw palm sensor, stopping")
    if any(l_regions_touching[0:3]) or l_regions_touching[4]:
      left_touching = 1
      rospy.loginfo("saw left fingertip tip or side, stopping")
    if any(r_regions_touching[0:3] or r_regions_touching[4]):
      right_touching = 1
      rospy.loginfo("saw right fingertip tip or side, stopping")
    return (left_touching, right_touching, palm_touching)
  

  ##move the wrist to a desired Cartesian pose while watching the fingertip sensors
  #settling_time is how long to wait after the controllers think we're there
  def guarded_move_cartesian(self, pose_stamped, timeout = 3.0, settling_time = 0.5, \
                               pos_thres = .0025, rot_thres = .05, overshoot_dist = 0., \
                               overshoot_angle = 0., stuck_dist = 0, stuck_angle = 0):

    self.check_preempt()

    #send the goal to the Cartesian controllers
    rospy.loginfo("sending goal to Cartesian controllers")
    self.cm.move_cartesian(pose_stamped, blocking = 0, pos_thres = pos_thres, rot_thres = rot_thres, \
                          timeout = rospy.Duration(timeout), settling_time = rospy.Duration(settling_time), \
                          overshoot_dist = overshoot_dist, overshoot_angle = overshoot_angle)

    #watch the fingertip/palm sensors until the controllers are done and then some
    start_time = rospy.get_rostime()
    done_time = None
    (last_pos, last_rot) = self.cm.return_cartesian_pose()
    last_pos_array = [last_pos]*5
    last_rot_array = [last_rot]*5
    r = rospy.Rate(1./self.cm.timestep)
    while not rospy.is_shutdown():

      self.check_preempt()
      now = rospy.get_rostime()
      (current_pos, current_rot) = self.cm.return_cartesian_pose()

      #stop if you hit a tip, side, back, or palm
      (left_touching, right_touching, palm_touching) = self.check_guarded_move_contacts()

      #saw a contact, freeze the arm
      if left_touching or right_touching or palm_touching:
        rospy.loginfo("saw contact")
        if self.cm.use_trajectory_cartesian:
          self.cm.switch_to_joint_mode()
          self.cm.freeze_arm()
        else:
          self.cm.set_desired_cartesian_to_current()
        break

      #check if we're actually there
      if self.cm.check_cartesian_really_done(pose_stamped, pos_thres, rot_thres, current_pos, current_rot):
        rospy.loginfo("actually got there")
        break

      #check if we're stuck
      if stuck_dist or stuck_angle:
        (pos_change, angle_change) = self.cm.dist_from_current_pos_and_rot(last_pos_array[0], last_rot_array[0])
        for ind in range(len(last_pos_array)-1):
          last_pos_array[ind] = last_pos_array[ind+1]
          last_rot_array[ind] = last_rot_array[ind+1]
        last_pos_array[-1] = current_pos
        last_rot_array[-1] = current_rot
        if now - start_time > rospy.Duration(2.0) and\
              abs(pos_change) <= stuck_dist*self.cm.timestep and \
              abs(angle_change) <= stuck_angle*self.cm.timestep:
          rospy.loginfo("Cartesian controller is stuck, stopping.  pos_change=%f, angle_change=%f", \
                          pos_change, angle_change)
          break

      #check if the controllers think we're done
      if not done_time and self.cm.check_cartesian_done():
        rospy.loginfo("check_cartesian_done returned 1")
        done_time = rospy.get_rostime()

      #done settling
      if self.cm.use_trajectory_cartesian and done_time and \
            rospy.get_rostime() - done_time > rospy.Duration(settling_time):
        rospy.loginfo("done settling")
        break

      #timed out
      if timeout != 0. and rospy.get_rostime() - start_time > rospy.Duration(timeout):
        rospy.loginfo("timed out")
        break

      r.sleep()

    self.cm.stop_cartesian_commands()
    #return whether the left and right fingers were touching
    return (left_touching, right_touching, palm_touching)


  ##move the wrist to a desired Cartesian pose using IK and watching the fingertip sensors
  def guarded_move_ik(self, pose_stamped, max_joint_vel = .05):

    self.check_preempt()

    #send the goal to the interpolated IK controllers (ignore collisions in IK)
    success = self.cm.command_interpolated_ik(pose_stamped, collision_aware = 0, step_size = .005, max_joint_vel = max_joint_vel)

    #if can't move using IK (out of reach), try to move using the Cartesian controllers instead
    if not success:
      (left_touching, right_touching, palm_touching) = self.guarded_move_cartesian(pose_stamped, 10.0, 5.0)
      return (left_touching, right_touching, palm_touching)

    #watch the fingertip sensors until the controllers are done and then some
    while(1):

      self.check_preempt()

      #stop if you hit a tip, side, or palm
      (left_touching, right_touching, palm_touching) = self.check_guarded_move_contacts()

      #saw a contact, freeze the arm
      if left_touching or right_touching or palm_touching:
        self.cm.freeze_arm()
        break

      #check if the controllers say they're done
      if self.cm.check_joint_trajectory_done():
        break

    #return whether the left and right fingers were touching
    return (left_touching, right_touching, palm_touching)


  #vector norm of a list
  def vect_norm(self, vect):
    return sum([x**2 for x in vect])**.5

  #normalize a vector
  def normalize_vect(self, vect):
    return list(scipy.array(vect)/self.vect_norm(vect))

  #dot product of two vectors (as lists)
  def vect_dot(self, vect1, vect2):
    return sum([x*y for (x,y) in zip(vect1, vect2)])

  #projection of vect1 onto vect2
  def vect_proj(self, vect1, vect2):
    norm_vect2 = self.normalize_vect(vect2)
    mag = self.vect_dot(vect1, norm_vect2)
    proj = [mag*x for x in norm_vect2]
    return proj

  #make vect1 orthogonal to vect2 while retaining the magnitude of vect1
  def make_orthogonal(self, vect1, vect2):
    vect1_mag = self.vect_norm(vect1)
    projected_comp = self.vect_proj(vect1, vect2)
    orthogonal_comp = [x-y for (x,y) in zip(vect1, projected_comp)]
    orthogonal_comp_mag = self.vect_norm(orthogonal_comp)
    orthogonal_vect = [vect1_mag * x / orthogonal_comp_mag for x in orthogonal_comp]
    return orthogonal_vect


  ##convert a relative vector in frame to a pose in the base_link frame
  #if start_pose is not specified, uses current pose of the wrist
  #if orthogonal_to_vect and orthogonal_to_vect_frame are specified, first convert vector to be orthogonal to orthogonal_to_vect 
  def return_rel_pose(self, vector, frame, start_pose = None, orthogonal_to_vect = None, orthogonal_to_vect_frame = 'base_link'):

    #if start_pose is not specified, use the current Cartesian pose of the wrist
    if start_pose == None:
      (start_trans, start_rot) = self.cm.return_cartesian_pose('base_link')
    else:
      start_pose.header.stamp = rospy.Time(0)
      (start_trans, start_rot) = pose_stamped_to_lists(self.cm.tf_listener, start_pose, 'base_link')

    #convert the vector in frame to base_link frame
    if frame != 'base_link':
      vector3_stamped = create_vector3_stamped(vector, frame)
      base_link_vector = vector3_stamped_to_list(self.cm.tf_listener, vector3_stamped, 'base_link')    
    else:
      base_link_vector = vector

#     print "start_trans:", self.cm.pplist(start_trans), "start_rot:", self.cm.pplist(start_rot)
#     print "frame:", frame
#     print "base_link_vector:", self.cm.pplist(base_link_vector)

    #if orthogonal_to_vect and orthogonal_to_vect_frame are specified, make the vector orthogonal to that vector
    if orthogonal_to_vect != None:

      #first convert it to the base_link frame if it's not already
      if orthogonal_to_vect_frame != 'base_link':
        ortho_vector3_stamped = create_vector3_stamped(orthogonal_to_vect, orthogonal_to_vect_frame)
        ortho_base_link_vector = vector3_stamped_to_list(self.cm.tf_listener, ortho_vector3_stamped, 'base_link')    
      else:
        ortho_base_link_vector = orthogonal_to_vect

      #now project the desired vector onto the orthogonal vector and subtract out that component
      base_link_vector = self.make_orthogonal(base_link_vector, ortho_base_link_vector)

    #add the desired vector to the position
    new_trans = [x+y for (x,y) in zip(start_trans, base_link_vector)]

    #create a new poseStamped
    pose_stamped = create_pose_stamped(new_trans+start_rot)

#     if orthogonal_to_vect != None:
#       print "orthogonal_to_vect:", self.cm.pplist(orthogonal_to_vect)
#       print "base_link_vector after being made orthogonal:", self.cm.pplist(base_link_vector)
#     print "new_trans:", self.cm.pplist(new_trans)
    #print "resulting pose_stamped:\n", pose_stamped

    return pose_stamped


  ##check if we have gotten close enough to our goal along approach_vect (according to goal_pos_thres)
  def check_goal_reached(self, approach_vect, goal_pos, goal_pos_thres):
    (current_pos, current_rot) = self.cm.return_cartesian_pose()

    #project the current position and the goal position onto the (already-normalized) approach vector 
    current_proj = sum([x*y for (x,y) in zip(current_pos, approach_vect)])
    goal_proj = sum([x*y for (x,y) in zip(goal_pos, approach_vect)])

    posdiff = goal_proj-current_proj  #if we overshot due to the controllers, that's fine too
    #print "current_proj = %5.3f, goal_proj = %5.3f, posdiff:%5.3f"%(current_proj, goal_proj, posdiff)
    #print "current_pos:", self.cm.pplist(current_pos)

    if posdiff < goal_pos_thres:
      return 1

    return 0


  ##compute (unit) approach direction in base_link frame from an approach_pose and grasp_pose (returns scipy array)
  def compute_approach_dir(self, approach_pose, grasp_pose):
    approach_vect = self.compute_approach_vect(approach_pose, grasp_pose)
    approach_dir = approach_vect/scipy.dot(approach_vect, approach_vect)**.5
    return approach_dir


  ##compute approach vector in base_link frame from an approach_pose and grasp_pose (returns scipy array)
  def compute_approach_vect(self, approach_pose, grasp_pose):
    (approach_pos, approach_rot) = pose_stamped_to_lists(self.cm.tf_listener, approach_pose, 'base_link')
    (grasp_pos, grasp_rot) = pose_stamped_to_lists(self.cm.tf_listener, grasp_pose, 'base_link')
    approach_vect = scipy.array(grasp_pos) - scipy.array(approach_pos)
    return approach_vect


  ##are either of the palm sensors touching? (palm sensors removed; stubs left in just in case)
  def palm_touching(self):
    #palm_state = self.palm_listener.get_state()
    #return any(palm_state)
    return 0


  ##reactive approach (stop if unexpected fingertip contact and go around)
  #assumes we are already at the approach pose, going to grasp_pose along the direction approach_dir
  #grasp_pose should be in base_link frame
  #and have found and checked the joint-angle path
  #side_step is how far to move right or left when you hit something
  #num_tries is how many bump-and-moves to try before quitting
  def reactive_approach(self, approach_dir, grasp_pose, joint_path = None, 
                        side_step = .015, back_step = .03, \
                        num_tries = 10, goal_pos_thres = 0.01):

    self.check_preempt()

    self.broadcast_phase(ManipulationPhase.MOVING_TO_GRASP)

    ever_saw_left = ever_saw_right = 0
    #self.cm.reload_joint_controllers(.2)

    (left_touching, right_touching, palm_touching) = self.check_guarded_move_contacts()

    if not left_touching and not right_touching and not palm_touching:
      if 0:
      #if joint_path != None:
        #use the joint trajectory action to move through joint_path slowly
        #while checking the fingertip/palm sensors
        self.cm.command_joint_trajectory(joint_path, 0.1)
        while not self.cm.check_joint_trajectory_done():
          (left_touching, right_touching, palm_touching) = self.check_guarded_move_contacts()
          #saw a contact, freeze the arm
          if left_touching or right_touching or palm_touching:
            self.cm.freeze_arm()
            break
      else:
        #use the Cartesian controllers to move through the approach path
        #(left_touching, right_touching, palm_touching) = self.guarded_move_cartesian(grasp_pose, 10.0, 5.0)
        (left_touching, right_touching, palm_touching) = self.guarded_move_ik(grasp_pose)
      
    #made it all the way through without touching anything, return
    if not left_touching and not right_touching and not palm_touching:
      rospy.loginfo("approach move finished with no contacts")
      return self.reactive_approach_result_dict["success"]

    #wait a bit to settle, then check if we are close enough to the goal 
    #even though we touched something
    time.sleep(1.0)
    (goal_pos, goal_rot) = pose_stamped_to_lists(self.cm.tf_listener, grasp_pose, 'base_link')
    if self.check_goal_reached(approach_dir, goal_pos, goal_pos_thres):
      rospy.loginfo("made it to within goal threshold")
      return self.reactive_approach_result_dict["within goal threshold"]

    #saw palm contact!  Return
    if palm_touching:
      rospy.loginfo("saw palm contact")
      return self.reactive_approach_result_dict["saw palm contact"]

    #saw both contacts at once!  Return
    if left_touching and right_touching:
      rospy.loginfo("both fingers touching")
      return self.reactive_approach_result_dict["both fingers touching"]

    #colliding with the table!  Return
    if (left_touching or right_touching) and self.check_fingers_in_table():
      rospy.loginfo("touching the table!  Returning")
      return self.reactive_approach_result_dict["touching table"]

    #saw only one contact, try to go around
    rospy.loginfo("saw one contact")
    tries = 0
    new_goal = grasp_pose
    while(tries < num_tries):

      rospy.loginfo("approach step %d"%tries)

      if self._photo:
        keypause()

      #back off a bit along the approach vector
      back_off_vect = -approach_dir*back_step
      pose = self.return_rel_pose(back_off_vect, 'base_link')
      self.cm.move_cartesian_ik(pose)
      #self.move_cartesian_step(pose, blocking = 1)

      self.check_preempt()

      #reset the fingertip sensors (hopefully we're no longer touching)
      self.pressure_listener.set_thresholds(short = 1)

      self.check_preempt()

      if self._photo:
        keypause()

      #left finger was touching; go to the left by side_step
      if left_touching:
        rospy.loginfo("side-stepping to the left")
        side_step_vect = [0., side_step, 0.]

      #right finger touching; go to the right by side_step
      if right_touching:
        rospy.loginfo("side-stepping to the right")
        side_step_vect = [0., -side_step, 0.]

      #execute the side step in guarded fashion
      pose = self.return_rel_pose(side_step_vect, self.whicharm+'_wrist_roll_link', orthogonal_to_vect = approach_dir)
      (left_touching, right_touching, palm_touching) = self.guarded_move_ik(pose)
      #(left_touching, right_touching, palm_touching) = self.guarded_move_cartesian(pose, 3.0, 1.0)
      new_goal = self.return_rel_pose(side_step_vect, self.whicharm+'_wrist_roll_link', new_goal)

      self.check_preempt()

      if self._photo:
        keypause()

      #if we didn't hit while moving sideways, 
      #move to our new (offset) goal in guarded fashion
      if not left_touching and not right_touching:
        (goal_pos, goal_rot) = pose_stamped_to_lists(self.cm.tf_listener, new_goal, 'base_link')
        rospy.loginfo("going to new goal, pos: "+self.cm.pplist(goal_pos)+" rot: "+self.cm.pplist(goal_rot))
        (left_touching, right_touching, palm_touching) = self.guarded_move_ik(new_goal)
        #(left_touching, right_touching, palm_touching) = self.guarded_move_cartesian(new_goal, 5.0, 3.0)    

        self.check_preempt()

        if self._photo:
          keypause()

        #made it all the way through without touching anything, return
        if not left_touching and not right_touching and not palm_touching:
          rospy.loginfo("made it all the way without touching anything")
          return self.reactive_approach_result_dict["success"]

        #wait a bit to settle, then check if we are close enough to the goal 
        #even though we touched something
        time.sleep(1.0)
        if self.check_goal_reached(approach_dir, goal_pos, goal_pos_thres):
          rospy.loginfo("within goal threshold")
          return self.reactive_approach_result_dict["within goal threshold"]

      else:
        rospy.loginfo("hit something while side-stepping")

      #saw palm contact!  Return
      if palm_touching:
        rospy.loginfo("saw palm contact")
        return self.reactive_approach_result_dict["saw palm contact"]

      #seeing both contacts now!  Return
      if left_touching and right_touching:
        rospy.loginfo("seeing both contacts!  Returning")
        return self.reactive_approach_result_dict["both fingers touching"]

      #colliding with the table!  Return
      if (left_touching or right_touching) and self.check_fingers_in_table():
        rospy.loginfo("touching the table!  Returning")
        return self.reactive_approach_result_dict["touching table"]

      #update the history
      ever_saw_left = ever_saw_left or left_touching
      ever_saw_right = ever_saw_right or right_touching

      #saw both contacts at some point!  Halve the side-step size
      if ever_saw_left and ever_saw_right:
        side_step /= 2.
        rospy.loginfo("saw both contacts, halving the side-step size to %5.3f"%side_step)
        ever_saw_right = right_touching
        ever_saw_left = left_touching

      tries += 1

    rospy.loginfo("ran out of tries")
    return self.reactive_approach_result_dict["ran out of tries"]


  ##check for marginal grasps 
  #if we see only the edge sensors on both fingers, move in the z-axis direction 
  #if we see only the tip sensors, or front sensors not far enough in (and no palm sensors are pressed), move forward
  #if we see only one edge on one finger and the opposite edge on the other finger, rotate the hand (disabled)
  def check_if_contacts_need_adjusting(self, min_contact_row, l_readings = None, r_readings = None, check_table = 1):
    
    (l_readings, r_readings, l_regions_touching, r_regions_touching, l_rows_touching, r_rows_touching, \
       l_columns_touching, r_columns_touching) = self.pressure_listener.get_sensor_status(l_readings, r_readings)
    self.pressure_listener.print_sensor_status(l_readings, r_readings)

    front_touching = 1
    adjust_x = 0
    adjust_z = 0
    adjust_rot = 0
    fingers_in_table = 0

    #check if our fingers are touching the table (if so, the grasp is probably fine, just return)
    if check_table and self.check_fingers_in_table():
      rospy.loginfo("fingers near table, grasp is probably fine despite not having any front contacts")
      return (1, 0, 0, 0)

    #check if we have any contacts on the front of either sensor
    if not any(l_rows_touching) and not any(r_rows_touching):

      #just return that there aren't any front elements touching
      rospy.loginfo("no front elements touching")
      return (0, 0, 0, 0)


    #check if we need to move along the x-axis (forward)
    if min_contact_row >= 0:
      if self.palm_touching():
        rospy.loginfo("palm touching, can't move forward")
      else:
        r_contact_row = -1
        if any(r_rows_touching):
          r_contact_row = 4-r_rows_touching.index(1)
        l_contact_row = -1
        if any(l_rows_touching):
          l_contact_row = 4-l_rows_touching.index(1)
        rospy.loginfo("l contact row is %d, r contact row is %d"%(l_contact_row, r_contact_row))
        if l_contact_row < min_contact_row and r_contact_row < min_contact_row:
          rospy.loginfo("left and right contacts both not far enough in, need to move forward")
          adjust_x = 1
        if not adjust_x:
          rospy.loginfo("contacts are fine in x-direction")


    #check if we need to move along the z-axis (sideways)
    #center touching on at least one finger
    if l_columns_touching[1] or r_columns_touching[1]:
      rospy.loginfo("at least one center column touching, no need to adjust in z-direction")

    #need to move in +z direction
    elif (l_columns_touching[0] or r_columns_touching[0]) and not (l_columns_touching[2] or r_columns_touching[2]):
      adjust_z = 1

    #need to move in -z direction
    elif (l_columns_touching[2] or r_columns_touching[2]) and not (l_columns_touching[0] or r_columns_touching[0]):
      adjust_z = -1

    #contact on both sides in z-direction
    else:
      rospy.loginfo("contacts not all on one side, no need to adjust in z-direction")


      #check if we need to rotate the hand about the x-axis
      #need to rotate about -x
      if l_columns_touching[0] and r_columns_touching[2] and not (l_columns_touching[2] or r_columns_touching[0]):
        adjust_rot = -1

      #need to rotate about +x
      elif l_columns_touching[2] and r_columns_touching[0] and not(l_columns_touching[0] or r_columns_touching[2]):
        adjust_rot = 1

      else:
        rospy.loginfo("no need to rotate about x")
        
    rospy.loginfo("front_touching:%d, adjust_x:%d, adjust_z:%d, adjust_rot:%d"%(front_touching, adjust_x, adjust_z, adjust_rot))
    return (front_touching, adjust_x, adjust_z, adjust_rot)


  ##pretty-print scipy matrix to string
  def ppmat(self, mat):
    str = ''
    for i in range(mat.shape[0]):
      for j in range(mat.shape[1]):
        str += '%5.3f\t'%mat[i,j]
      str += '\n'
    return str


  ##check the gripper opening to see if it's within our goal gripper opening range
  def check_gripper_opening(self, min_gripper_opening = .01, max_gripper_opening = .1):
    gripper_opening = self.cm.get_current_gripper_opening()
    rospy.loginfo("min_gripper_opening:"+str(min_gripper_opening))
    if min_gripper_opening < gripper_opening and gripper_opening < max_gripper_opening:
      rospy.loginfo("gripper opening was acceptable: %5.4f"%gripper_opening)
      return 1
      
    rospy.loginfo("gripper opening unacceptable: %5.4f"%gripper_opening)
    return 0
  

  ##lift while checking for slip (assumes we already closed around the object)
  #gripper_translation is a desired GripperTranslation for the lift
  #slip servoing should continue after the lift (for moving the object through free space), until the gripper opens or closes
  #returns 1 if the object is still in the hand at the end of the lift, 0 otherwise
  def lift_carefully(self, gripper_translation, min_gripper_opening = .0021, max_gripper_opening = .1, hardness_gain = 0.060):

    #convert the gripper_translation to a desired PoseStamped for the wrist
    base_link_translation = change_vector3_stamped_frame(self.cm.tf_listener, gripper_translation.direction, 'base_link')

    #add it to the current pose of the wrist
    start_wrist_pose = self.cm.get_current_wrist_pose_stamped()

    #if using the slip controller, first open and close to determine how hard to grasp the object,
    #and start the slip servo controller going (blocks until open and close is done)
    if self.using_slip_detection:
      self.cm.start_gripper_grab(hardness_gain)

    self.broadcast_phase(ManipulationPhase.LIFTING)

    #now move to the final desired position
    desired_wrist_pose = copy.deepcopy(start_wrist_pose)

    #temporary, while deepcopy of rospy.Time is broken
    desired_wrist_pose.header.stamp = rospy.Time(start_wrist_pose.header.stamp.secs)
    desired_wrist_pose.pose.position.x += base_link_translation.vector.x * gripper_translation.desired_distance
    desired_wrist_pose.pose.position.y += base_link_translation.vector.y * gripper_translation.desired_distance
    desired_wrist_pose.pose.position.z += base_link_translation.vector.z * gripper_translation.desired_distance
    self.move_cartesian_step(desired_wrist_pose, blocking = 1)

    #check to see if the object is still in the hand
    success = self.check_gripper_opening(min_gripper_opening, max_gripper_opening)

    return success


  ##shake the object by rotating the wrist roll and flex joints
  #and the arm pan joint at a particular set of arm angles
  def shake_object(self, arm_angles = None, min_gripper_opening = .0021, max_gripper_opening = .1,
                   wrist_roll_shake_time = 0.5, wrist_flex_shake_time = 0.5, arm_pan_shake_time = 0.25):
    
    #move to standard arm angles for this arm
    wrist_flex = -1.03
    wrist_roll = -.5 
    if arm_angles == None:
      if self.whicharm == 'r':
        arm_angles = [-0.28, -0.21, -0.78, -1.02, -1.02, wrist_flex, wrist_roll]
      else:
        arm_angles = [0.28, -0.21, 0.78, -1.02, 1.02, wrist_flex, wrist_roll]

    rospy.loginfo("moving to shake arm angles")
    self.broadcast_phase(ManipulationPhase.MOVING_WITH_OBJECT)
    self.cm.command_joint_trajectory([arm_angles], blocking = 1)

    #allow time for the object to fall out
    self.broadcast_phase(ManipulationPhase.HOLDING_OBJECT)
    time.sleep(4)

    #check if the object is still in the gripper; if not, return failed
    success = self.check_gripper_opening(min_gripper_opening, max_gripper_opening)
    if not success:
      return 0

    #rotate wrist roll joint (+,-,+,-)
    shake_amplitude = 1.75*1.57
    roll_pos_angles = arm_angles[:]
    roll_pos_angles[6] += shake_amplitude
    roll_neg_angles = arm_angles[:]
    roll_neg_angles[6] -= shake_amplitude

    roll_trajectory = [roll_pos_angles, roll_neg_angles, roll_pos_angles, roll_neg_angles, arm_angles]
    times = [wrist_roll_shake_time*i+0.05 for i in range(len(roll_trajectory))]
    vels = [[0.0]*7 for i in range(len(roll_trajectory))]

    rospy.loginfo("starting roll shake")
    self.broadcast_phase(ManipulationPhase.SHAKING)
    self.cm.command_joint_trajectory(roll_trajectory, blocking = 1, times_and_vels = (times, vels))

    #allow time for the object to fall out
    time.sleep(4)

    #check if the object is still in the gripper; if not, return failed
    success = self.check_gripper_opening(min_gripper_opening, max_gripper_opening)
    if not success:
      return 0

    #rotate wrist flex joint
    flex_pos_angles = arm_angles[:]
    flex_pos_angles[5] += .78
    flex_neg_angles = arm_angles[:]
    flex_neg_angles[5] -= .78
    
    flex_trajectory = [flex_pos_angles, flex_neg_angles, flex_pos_angles, flex_neg_angles, arm_angles]
    times = [wrist_flex_shake_time*i+0.05 for i in range(len(flex_trajectory))]
    vels = [[0.0]*7 for i in range(len(flex_trajectory))]

    rospy.loginfo("starting flex shake")
    self.broadcast_phase(ManipulationPhase.SHAKING)
    self.cm.command_joint_trajectory(flex_trajectory, blocking = 1, times_and_vels = (times, vels))

    #allow time for the object to fall out
    time.sleep(4)

    #check if the object is still in the gripper; if not, return failed
    success = self.check_gripper_opening(min_gripper_opening, max_gripper_opening)
    if not success:
      return 0

    #rotate arm pan joint
    pan_pos_angles = arm_angles[:]
    pan_neg_angles = arm_angles[:]
    shake_amplitude = 0.125
    if self.whicharm == 'r':
      pan_pos_angles[0] += shake_amplitude
      pan_neg_angles[0] -= shake_amplitude
    else:
      pan_pos_angles[0] -= shake_amplitude
      pan_neg_angles[0] += shake_amplitude
    pan_trajectory = [pan_pos_angles, pan_neg_angles, pan_pos_angles, pan_neg_angles, arm_angles]
    times = [arm_pan_shake_time*i+0.05 for i in range(len(pan_trajectory))]
    vels = [[0.0]*7 for i in range(len(pan_trajectory))]

    rospy.loginfo("starting pan shake")
    self.broadcast_phase(ManipulationPhase.SHAKING)
    self.cm.command_joint_trajectory(pan_trajectory, blocking = 1, times_and_vels = (times, vels))

    #allow time for the object to fall out
    time.sleep(4)

    #check if the object is still in the gripper; if not, return failed
    success = self.check_gripper_opening(min_gripper_opening, max_gripper_opening)
    if not success:
      return 0
    
    return 1


  ##place while checking for the gripper to report that the object has hit the table
  #final_place_pose is a PoseStamped for the gripper in its desired place pose
  #place_overshoot is the distance in m to go past the final_place_pose
  def place_carefully(self, final_place_pose, place_overshoot, min_gripper_opening = .0021, \
                        max_gripper_opening = .1, place_angle_max_diff = 1.05):
   
    self.broadcast_phase(ManipulationPhase.PLACING)

    #check if there's still something in the gripper to place
    if not self.check_gripper_opening(min_gripper_opening, max_gripper_opening):
      self.broadcast_phase(ManipulationPhase.FAILED)
      self.cm.switch_to_joint_mode()
      return 0

    #get the current pose of the wrist
    (start_wrist_pos, start_wrist_rot) = self.cm.return_cartesian_pose()
 
    #set the actual goal to just past the final place goal by place_overshoot
    (place_pos, place_rot) = pose_stamped_to_lists(self.cm.tf_listener, final_place_pose, 'base_link')
    place_vect = scipy.array(place_pos) - scipy.array(start_wrist_pos)
    place_vect_mag = (place_vect*place_vect).sum()
    overshoot_vect = place_vect / place_vect_mag * (place_vect_mag + place_overshoot)
    overshoot_pos = overshoot_vect + place_pos
    overshoot_pose = create_pose_stamped(overshoot_pos.tolist() + place_rot, 'base_link')

    #change the Cartesian control parameters to be more gentle
    self.cm.cart_params.set_params_to_gentle(set_tip_frame = 0, set_params_on_server = 1)

    #tell the gripper controller to go into place mode
    self.check_preempt()
    if self.using_slip_detection:
      self.cm.start_gripper_event_detector(blocking = 0)

    #move to the actual goal using the Cartesian controllers
    self.move_cartesian_step(overshoot_pose, blocking = 0)

    #look for the trigger saying that the object hit the table
    start_time = rospy.get_rostime()
    timeout = 5.
    start_angles = self.cm.get_current_arm_angles()
    while not rospy.is_shutdown():
      self.check_preempt()

      #check the state of the gripper place action
      if self.using_slip_detection:
        state = self.cm.get_gripper_event_detector_state()

        #action finished (trigger seen)
        if state not in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
          rospy.loginfo("place carefully saw the trigger, stopping the arm")
          break

      #check if the arm angles have changed too much
      current_angles = self.cm.get_current_arm_angles()
      if any([abs(current - start) > place_angle_max_diff for (current, start) in zip(current_angles, start_angles)]):
        rospy.loginfo("arm angle change too great during place, stopping the arm")
        break

      #timed out
      if rospy.get_rostime() - start_time > rospy.Duration(timeout):
        rospy.loginfo("place carefully timed out before seeing the trigger, stopping the arm")
        break

      time.sleep(0.1)

    #freeze the arm as soon as the trigger happens, or after the timeout 
    if self.cm.use_trajectory_cartesian:
      self.cm.switch_to_joint_mode()
      self.cm.freeze_arm()
    else:
      self.cm.set_desired_cartesian_to_current()

    #open the gripper
    self.broadcast_phase(ManipulationPhase.OPENING)
    self.cm.command_gripper(.1, -1, 1)

    #change the controller params back
    self.cm.cart_params.set_params_to_defaults(set_tip_frame = 0, set_params_on_server = 1)

    return 1


  ##close with a force of self.close_force, and record that the fingertips need a long-reset
  #unless we're using the slip controller, in which case open and close to estimate stiffness
  def close_hard(self, force = None):

    if not self.using_slip_detection:
      self.broadcast_phase(ManipulationPhase.CLOSING)
      tries = 0
      self._closed_hard = 1
      if force == None:
        force = self.close_force
      self.cm.command_gripper(0, force, 1)


  ##open the gripper, wait for the fingertips to release, and reset the thresholds if we're not touching anything
  def open_and_reset_fingertips(self, reset = 0):

    #open the gripper
    rospy.loginfo("opening gripper")
    self.cm.command_gripper(.1, -1, 1)

    self.check_preempt()

    #if we pressed hard when closing, have to wait awhile for the fingertip sensors to release
    if self._closed_hard:
      rospy.loginfo("waiting for the fingertips to release")
      time.sleep(2)
      self._closed_hard = 0
    else:
      time.sleep(1)

    self.check_preempt()

    if reset:
      #assume the fingertips are not touching anything and reset the thresholds
      self.pressure_listener.set_thresholds(short = 1)

    self.check_preempt()


  ##check to see if the fingers are colliding with the table in gripper_pose according to the collision model
  #defaults to the current gripper pose
  def check_fingers_in_table(self, gripper_pose = None):
    
    #if we don't have tip contact, we're probably not touching the table
    (l_regions_touching, r_regions_touching) = self.pressure_listener.regions_touching()
    #print "l_regions_touching:", l_regions_touching
    #print "r_regions_touching:", r_regions_touching
    if not (l_regions_touching[0] or r_regions_touching[0]):
      return 0

    if gripper_pose == None:
      gripper_pose = self.cm.get_current_wrist_pose_stamped()
    current_angles = self.cm.get_current_arm_angles()
    collision_oper1 = CollisionOperation(object1 = CollisionOperation.COLLISION_SET_ALL, \
                                        object2 = CollisionOperation.COLLISION_SET_ALL, \
                                        operation = CollisionOperation.DISABLE)
    collision_oper2 = CollisionOperation(object1 = self.whicharm+"_gripper_r_finger_tip_link", \
                                        object2 = self._table_name, \
                                        operation = CollisionOperation.ENABLE)
    collision_oper3 = CollisionOperation(object1 = self.whicharm+"_gripper_l_finger_tip_link", \
                                        object2 = self._table_name, \
                                        operation = CollisionOperation.ENABLE)                                        

    # ordered_collision_operations = OrderedCollisionOperations([collision_oper1, collision_oper2, collision_oper3])
    # (pos, error_code) = self.cm.ik_utilities.run_ik(gripper_pose, current_angles, self.cm.ik_utilities.link_name, \
    #                                                   collision_aware = 1, \
    #                                                   ordered_collision_operations = ordered_collision_operations)
    # if error_code == "IK_LINK_IN_COLLISION":
    #   rospy.loginfo("fingers are near the table")
    #   return 1
    # rospy.loginfo("fingers not near the table")
    return 0
  
  ##adjust the grasp in an attempt to center the object according to the contacts, if we appear to be in a marginal grasp
  #assumes the fingers start closed around the object
  def adjust_grasp(self, grasp_pose, initial_l_readings, initial_r_readings, z_step = .015, x_step = .015, min_gripper_opening = .0021, max_gripper_opening = .1, min_contact_row = 1, num_tries = 4):

    self.check_preempt()

    #check if the gripper opening is acceptable; if not, give up
    if not self.check_gripper_opening(min_gripper_opening, max_gripper_opening):
      return ("gripper opening unacceptable", grasp_pose)

    #check if the contacts need adjusting, and if so, in which directions
    (front_touching, adjust_x, adjust_z, adjust_rot) = self.check_if_contacts_need_adjusting(min_contact_row, initial_l_readings, initial_r_readings)

    #try to adjust the grasp a maximum of num_tries times
    tries = 0
    current_grasp_pose = grasp_pose
    self._closed_hard = 0
    while(1):

      self.check_preempt()

      #if you're out of adjust tries, but there are front contacts, just close hard and see if the grasp is okay anyway
      if tries >= num_tries:
        
        #if front_touching:
          rospy.loginfo("ran out of tries while adjusting the grasp, trying to close hard anyway")
          self.close_hard()

          #check if the gripper opening is acceptable and return
          if not self.check_gripper_opening(min_gripper_opening, max_gripper_opening):
            return ("gripper opening unacceptable", current_grasp_pose)
          return ("grasp okay", current_grasp_pose)
        #else:
        #  return ("ran out of tries", current_grasp_pose)

      rospy.loginfo("adjust_grasp tries: "+str(tries))

      #adjust the hand position in the appropriate directions (don't bother adjusting just rotation)
      if adjust_x or adjust_z:

        self.broadcast_phase(ManipulationPhase.ADJUSTING_GRASP)

        #find the new, adjusted grasp pose
        adjust_mat = scipy.matrix(scipy.identity(4))
        adjust_vect = scipy.matrix([0,0,0]).T
        if adjust_z:
          adjust_mat[2, 3] = adjust_z * z_step
        if adjust_x:
          adjust_mat[0, 3] = x_step

        new_grasp_pose = transform_pose_stamped(current_grasp_pose, adjust_mat)

        self.check_preempt()
    
        #open the gripper to adjust the grasp
        rospy.loginfo("opening gripper to adjust hand")
        self.open_and_reset_fingertips(reset = 1)
 
        self.check_preempt()
        if self._photo:
          keypause()
 
        #update the current grasp pose
        current_grasp_pose = new_grasp_pose

        #move the hand to the new grasp pose
        (left_touching, right_touching, palm_touching) = self.guarded_move_ik(current_grasp_pose)

        self.check_preempt()
        if self._photo:
          keypause()

        #close the gripper compliantly
        (l_readings, r_readings) = self.compliant_close()

        self.check_preempt()
        if self._photo:
          keypause()

        #update the new grasp pose to the current position
        current_grasp_pose = self.cm.get_current_wrist_pose_stamped()

        #check if the gripper opening is acceptable; if not, give up
        if not self.check_gripper_opening(min_gripper_opening, max_gripper_opening):
          return ("gripper opening unacceptable", current_grasp_pose)

        #get the new fingertip sensor readings and check if the new contacts are okay
        (front_touching, adjust_x, adjust_z, adjust_rot) = self.check_if_contacts_need_adjusting(min_contact_row, l_readings, r_readings)

      #is the grasp okay now? (ignore a possible rotation adjustment)
      grasp_okay = 1
      if not front_touching or adjust_x or adjust_z:
        grasp_okay = 0

      #grasp is okay, close hard
      if grasp_okay:
        self.check_preempt()  

        rospy.loginfo("contacts are fine")
        self.close_hard()

        if self._photo:
          keypause()

        #check if the gripper opening is acceptable; if not, give up
        if not self.check_gripper_opening(min_gripper_opening, max_gripper_opening):
          return ("gripper opening unacceptable", current_grasp_pose)

        #get the new fingertip sensor readings and check if the new contacts are okay
        (front_touching, adjust_x, adjust_z, adjust_rot) = self.check_if_contacts_need_adjusting(min_contact_row)
        grasp_okay = 1
        if not front_touching or adjust_x or adjust_z:
          grasp_okay = 0

      #if the grasp is still fine, stop
      if grasp_okay:
        rospy.loginfo("contacts are fine")
        result = "grasp okay"
        break
    
      tries += 1

    return (result, current_grasp_pose)


  ##do a reactive approach followed by a compliant close if the approach is successful
  #then move forward more and try again along the approach if the gripper opening is not as expected
  #succeeds if the gripper is closed with an opening between min_gripper_opening and max_gripper_opening
  #tries the approach-and-grasp grasp_num_tries times (including the first approach)
  #forward step is how far to move along the approach direction (past the goal) for the next approach-and-grasp try
  #min_contact_row is how far in from the tip we require contact on the front of each sensor (farther in is okay), with values from 0-4 (-1 to ignore)
  #possible return values: 0=success, 1=ran out of grasp tries, 2=ran out of approach tries, 3=aborted
  #approach_pose, grasp_pose, joint_path, side_step, back_step, approach_num_tries, goal_pos_thres are inputs to the approach
  def reactive_grasp(self, approach_pose, grasp_pose, joint_path = None, side_step = .015, back_step = .03, \
                       approach_num_tries = 10, goal_pos_thres = 0.01, min_gripper_opening = 0.0021, max_gripper_opening = 0.1, \
                       grasp_num_tries = 2, forward_step = 0.03, min_contact_row = 1, object_name = "points", table_name = "table", grasp_adjust_x_step = .02, grasp_adjust_z_step = .015, grasp_adjust_num_tries = 3):

    self._table_name = table_name

    self.check_preempt()

    #if approach_pose is not specified, use the current pose of the hand
    if approach_pose == None:
      approach_pose = self.cm.get_current_wrist_pose_stamped()

    #reset the fingertip thresholds and palm sensor state (assumes we aren't touching anything right now)
    self.pressure_listener.set_thresholds()
    #self.palm_listener.reset_state()

    self.check_preempt()

    #if approach and/or grasp are not in base_link frame, convert them to base_link frame
    if approach_pose.header.frame_id != 'base_link':
      self.cm.tf_listener.waitForTransform('base_link', approach_pose.header.frame_id, rospy.Time.now(), rospy.Duration(5))
      try:
        approach_pose = self.cm.tf_listener.transformPose('base_link', approach_pose)
      except:
        rospy.logerr("tf transform was not there!")
        self.throw_exception()
        return self.reactive_grasp_result_dict["aborted"]

    if grasp_pose.header.frame_id != 'base_link':
      self.cm.tf_listener.waitForTransform('base_link', grasp_pose.header.frame_id, rospy.Time.now(), rospy.Duration(5))
      try:
        grasp_pose = self.cm.tf_listener.transformPose('base_link', grasp_pose)
      except:
        rospy.logerr("tf transform was not there!")
        self.throw_exception()
        return self.reactive_grasp_result_dict["aborted"]
      
    #compute (unit) approach direction in base_link frame
    approach_dir = self.compute_approach_dir(approach_pose, grasp_pose)

    num_tries = 0
    current_goal_pose = grasp_pose
    while 1:

      if approach_num_tries:
        #try the approach, unless the reactive approach is disabled (approach_num_tries is 0)
        approach_result = self.reactive_approach(approach_dir, current_goal_pose, joint_path, \
                                                   side_step, back_step, \
                                                   approach_num_tries, goal_pos_thres)

        #quit if the approach ran out of tries (couldn't get around the object) (disabled; just close anyway)
        #if approach_result == self.reactive_approach_result_dict["ran out of tries"]:
        #  return self.reactive_grasp_result_dict["ran out of approach tries"]

        if approach_result == self.reactive_approach_result_dict["grasp infeasible"]:
          return self.reactive_grasp_result_dict["grasp infeasible"]

      else:

        #reactive approach is disabled; just move to the goal
        self.broadcast_phase(ManipulationPhase.MOVING_TO_GRASP)
        rospy.loginfo("reactive approach is disabled, moving to the goal")
        self.move_cartesian_step(grasp_pose, timeout = 10.0, settling_time = 5.0, blocking = 1)

      #if both fingers were touching, or we got to the grasp goal, or we're touching the table, try to close the fingers
      rospy.loginfo("starting compliant_close")
      (l_readings, r_readings) = self.compliant_close()

      #update the new goal pose to the current position
      current_goal_pose = self.cm.get_current_wrist_pose_stamped()

      #check if the object is centered/adjust incrementally if necessary
      (adjust_result, current_goal_pose) = self.adjust_grasp(current_goal_pose, l_readings, r_readings, z_step = grasp_adjust_z_step, \
                     x_step = grasp_adjust_x_step, min_gripper_opening = min_gripper_opening, max_gripper_opening = max_gripper_opening, \
                     min_contact_row = min_contact_row, num_tries = grasp_adjust_num_tries)
      rospy.loginfo("adjust_result: "+adjust_result)

      #if the grasp looks good, declare success and stop
      if adjust_result == "grasp okay":
        return self.reactive_grasp_result_dict["success"]

      #if the gripper opening is unacceptable, but the palm sensors are touching something, also declare success and stop
      if self.palm_touching():
        return self.reactive_grasp_result_dict["success"]

      #if the gripper opening is unacceptable or there were no front contacts, and there are no palm contacts, take a major step forward and try approaching again until we run out of tries
      num_tries += 1
      if num_tries < grasp_num_tries:

        #move the goal forward along the approach direction by forward_step
        move_forward_vect = approach_dir*forward_step
        current_goal_pose = self.return_rel_pose(move_forward_vect, 'base_link', current_goal_pose)

        joint_path = None
          
        #print the new goal
        (goal_pos, goal_rot) = pose_stamped_to_lists(self.cm.tf_listener, current_goal_pose, 'base_link')
        rospy.loginfo("trying approach again with new goal, pos: "+self.cm.pplist(goal_pos)+" rot: "+self.cm.pplist(goal_rot))

        if self._photo:
          keypause()

        #open the gripper back up
        self.open_and_reset_fingertips(reset = 1)

        if self._photo:
          keypause()
        

      else:
        rospy.loginfo("ran out of grasp tries!")
        return self.reactive_grasp_result_dict["ran out of grasp tries"]
    
      


#sample code for using the above functions directly
if __name__ == "__main__":

  rospy.init_node('grasp_demo', anonymous = True)

  print "initializing controller manager"
  cm = controller_manager.ControllerManager('r')

  print "opening gripper"
  cm.command_gripper(.1, -1, 1)
  print "done"

  print "initializing reactive grasper"
  rg = ReactiveGrasper(cm)
  print "done initialization"

  cm.start_joint_controllers()
  cm.start_gripper_controller()

  #top approach/grasp
  #table_height = .55  #simulated table
  #table_height = .7366  #table in middle of green room
  table_height = .7239 #round table
  tip_dist_to_table = .12
  wrist_height = table_height + tip_dist_to_table + .02

  topapproachpos = [.52, -.05, wrist_height+.1]
  topapproachquat = [-0.5, 0.5, 0.5, 0.5]  #from the top
  topgrasppos = topapproachpos[:]
  topgrasppos[2] -= .08
  top_approach_pose = create_pose_stamped(topapproachpos+topapproachquat)
  top_grasp_pose = create_pose_stamped(topgrasppos+topapproachquat)

  #side approach/grasp
  sideangles = [-0.447, -0.297, -2.229, -0.719, 0.734, -1.489, 1.355]
  side_tip_dist_to_table = .14  
  start_angles = sideangles
  tiltangle = math.pi/18.
  sideapproachmat = scipy.array([[0., -1., 0., 0.],  
                                 [math.cos(tiltangle), 0., math.sin(tiltangle), 0.],
                                 [-math.sin(tiltangle), 0., math.cos(tiltangle), 0.],
                                 [0., 0., 0., 1.]])
  sideapproachpos = [.63, -.3, table_height-.035+side_tip_dist_to_table]
  sideapproachquat = list(tf.transformations.quaternion_from_matrix(sideapproachmat))
  sidegrasppos = sideapproachpos[:]
  sidegrasppos[1] += .09

  side_approach_pose = create_pose_stamped(sideapproachpos+sideapproachquat)
  side_grasp_pose = create_pose_stamped(sidegrasppos+sideapproachquat)

  #compute approach vector
  top_approach_vect = rg.compute_approach_vect(top_approach_pose, top_grasp_pose)
  side_approach_vect = rg.compute_approach_vect(side_approach_pose, side_grasp_pose)

  mode = 'none'
  #update currentgoal to the new current position
  (pos, rot) = cm.return_cartesian_pose()
  currentgoal = pos+rot

  small_step = .1

  while(not rospy.is_shutdown()):
    print "enter top to go to the top approach position" 
    print "side to go to the side approach position"
    print "rg to run the full reactive grasp"
    print "ra to approach once reactively"
    print "cc to close compliantly"
    print "c to close, o to open"
    print "u to go up, d to go down, r to go right, l to go left"
    print "a to go away from the robot, t to go toward the robot"
    print "j to go to a set of nearish-side-grasp angles"
    print "reset to reset the fingertip thresholds"
    print "f to print the current finger sensor values"
    print "s to stop"
    c = keypause()

    if c == 'top':
      print "moving to top approach position"
      step_size = .01
      max_joint_vel = .05
      if mode == 'side' or mode == 'none':
        step_size = .03
        max_joint_vel = .15
      cm.command_interpolated_ik(top_approach_pose, collision_aware = 0, step_size = step_size, max_joint_vel = max_joint_vel)
      while not cm.check_joint_trajectory_done() and not rospy.is_shutdown():
        time.sleep(.1)
      print "done"
      
      #update currentgoal to the new current position
      (pos, rot) = cm.return_cartesian_pose()
      currentgoal = pos+rot
      mode = 'top'

    elif c == 'side':
      print "moving to side approach position"
      step_size = .01
      max_joint_vel = .05
      if mode == 'top' or mode == 'none':
        step_size = .03
        max_joint_vel = .15
      cm.command_interpolated_ik(side_approach_pose, collision_aware = 0, step_size = step_size, max_joint_vel = max_joint_vel)
      while not cm.check_joint_trajectory_done():
        time.sleep(.1)
      print "done"

      #update currentgoal to the new current position
      (pos, rot) = cm.return_cartesian_pose()
      currentgoal = pos+rot
      mode = 'side'

    elif c == 'j':
      print "moving to near-side-grasp angles"
      cm.command_joint(sideangles)
      #   while not cm.check_joint_trajectory_done():
      #     time.sleep(.1)
      #   print "done"

    elif c == 'rg' or c == 'ra':
      print "performing reactive grasp relative to current pose"
      current_pose = rg.cm.get_current_wrist_pose_stamped()
      if mode == 'top':
        new_grasp_pose = rg.return_rel_pose(top_approach_vect, 'base_link')
        grasp_num_tries = 2
        forward_step = .02
      elif mode == 'side':
        new_grasp_pose = rg.return_rel_pose(side_approach_vect, 'base_link')
        grasp_num_tries = 2
        forward_step = .06
      else:
        print "go to either top or side approach first"
        continue

      #check that the approach is reachable, find the arm angle trajectory
      current_angles = cm.get_current_arm_angles()
      (trajectory, error_codes) = cm.ik_utilities.check_cartesian_path(current_pose, new_grasp_pose, current_angles, collision_aware = 0)

      #trajectory bad, use the Cartesian controllers
      if any(error_codes):
        rospy.loginfo("can't execute an interpolated IK trajectory to the grasp pose, using Cartesian controllers instead")
        if c == 'ra':

          #compute (unit) approach direction in base_link frame
          approach_dir = rg.compute_approach_dir(current_pose, new_grasp_pose)

          rg.reactive_approach(approach_dir, new_grasp_pose)
        else:
          result = rg.reactive_grasp(current_pose, new_grasp_pose, forward_step = forward_step, grasp_num_tries = grasp_num_tries)

      #trajectory good, run the reactive approach
      else:
        if c == 'ra':
          #compute (unit) approach direction in base_link frame
          approach_dir = rg.compute_approach_dir(current_pose, new_grasp_pose)
          print "approach_dir:", approach_dir

          rg.reactive_approach(approach_dir, new_grasp_pose, trajectory)
        else:
          result = rg.reactive_grasp(current_pose, new_grasp_pose, trajectory, forward_step = forward_step, grasp_num_tries = grasp_num_tries)

      #update currentgoal to the new current position
      (pos, rot) = cm.return_cartesian_pose()
      currentgoal = pos+rot

      #if reactive grasp was successful, lift
      if c == 'rg' and result == 0:
        currentgoal[2] += .1
        rg.move_cartesian_step(currentgoal, blocking = 1)

    elif c == 'cc':
      print "compliantly closing gripper"
      rg.compliant_close()

      #update currentgoal to the new current position
      (pos, rot) = cm.return_cartesian_pose()
      currentgoal = pos+rot

    elif c == 'c':
      print "closing gripper"
      rg.close_hard()
      #cm.command_gripper(0, 100.0, 1)
    elif c == 'o':
      print "opening gripper"
      rg.open_and_reset_fingertips(reset = 1)
      #cm.command_gripper(.1, -1, 1)
    elif c == 'u':
      print "going up"
      currentgoal[2] += .1
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 'us':
      print "going up a small amount"
      currentgoal[2] += .02
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 'd':
      print "going down"
      currentgoal[2] -= .05
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 'ds':
      print "going down a small amount"
      currentgoal[2] -= .02
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 'r':
      print "moving right"
      currentgoal[1] -= small_step
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 'l':
      print "moving left"
      currentgoal[1] += small_step
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 'a':
      print "moving away from robot"
      currentgoal[0] += small_step
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 't':
      print "moving toward the robot"
      currentgoal[0] -= small_step
      rg.move_cartesian_step(currentgoal, blocking = 1)
    elif c == 'reset':
      print "resetting the fingertip thresholds"
      rg.pressure_listener.set_thresholds()
    elif c == 'f':
      print "printing the finger sensor values"
      rg.pressure_listener.print_sensor_status()
    elif c == 's':
      print "quitting"
      break

  print "stopping Cartesian controllers"
  cm.stop_controllers(stop_cartesian = 1)

