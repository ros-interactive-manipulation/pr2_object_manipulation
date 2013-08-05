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


## @package joint_states_listener
#Spins off a thread to listen for joint_states messages and 
#provides the same information (or subsets of) as a Python function

from __future__ import division
import roslib
roslib.load_manifest('pr2_gripper_reactive_approach')
import rospy
from sensor_msgs.msg import JointState
import threading
import time
import sys

##holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self, run_server = 0):
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.setDaemon(True)
        self.thread.start()

    '''
    Service removed due to not having a home for the following service type:
    ReturnJointStates.srv
    string[] name
    ---
    uint32[] found
    float64[] position
    float64[] velocity
    float64[] effort
    '''
#         if run_server:
#             s = rospy.Service('return_joint_states', ReturnJointStates, self.return_joint_states_service)
        

    ##thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    ##callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        self.lock.release()


    ##returns (found, position, velocity, effort) for the joint joint_name 
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        #no messages yet
        if self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        #return info for this joint
        self.lock.acquire()
        if joint_name in self.name:
            index = self.name.index(joint_name)
            position = self.position[index]
            velocity = self.velocity[index]
            effort = self.effort[index]

        #unless it's not found
        else:
            rospy.logerr("Joint %s not found!", (joint_name,))
            self.lock.release()
            return (0, 0., 0., 0.)
        self.lock.release()
        return (1, position, velocity, effort)


#     #server callback: returns arrays of position, velocity, and effort
#     #for a list of joints specified by name
#     def return_joint_states_service(self, req):
#         (joints_found, positions, velocities, efforts) = self.return_joint_states(req.name)
#         return ReturnJointStatesResponse(joints_found, positions, velocities, efforts)


    ##return the positions, velocities, and efforts for a list of joint names
    def return_joint_states(self, joint_list):
        joints_found = []
        positions = []
        velocities = []
        efforts = []
        for joint_name in joint_list:
            (found, position, velocity, effort) = self.return_joint_state(joint_name)
            joints_found.append(found)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
        return (joints_found, positions, velocities, efforts)


#keep printing the current arm angles
if __name__ == "__main__":

    #which arm to print
    if len(sys.argv) < 2 or sys.argv[1] != 'r' and sys.argv[1] != 'l':
        rospy.logerr("usage: joint_states_listener.py which_arm (which_arm is r or l)")
        sys.exit(1)
    which_arm = sys.argv[1]

    #pretty-print list to string
    def pplist(list):
        return ' '.join(['%5.3f'%x for x in list])

    rospy.init_node(which_arm+'_joint_states_listener')

    latest_joint_states = LatestJointStates(run_server = 0)
    
    joint_names = ["_shoulder_pan_joint",
                   "_shoulder_lift_joint",
                   "_upper_arm_roll_joint",
                   "_elbow_flex_joint",
                   "_forearm_roll_joint",
                   "_wrist_flex_joint",
                   "_wrist_roll_joint"]
    r_joint_names = ["r" + joint_name for joint_name in joint_names]
    l_joint_names = ["l" + joint_name for joint_name in joint_names]
    while not rospy.is_shutdown():
        time.sleep(0.5)

        if which_arm == 'r':
            (joints_found, current_r_arm_angs, vels, efforts) = \
                latest_joint_states.return_joint_states(r_joint_names)
            rospy.loginfo("current right arm angles:"+pplist(current_r_arm_angs))
        else:
            (joints_found, current_l_arm_angs, vels, efforts) = \
                latest_joint_states.return_joint_states(l_joint_names)
            rospy.loginfo("current left arm angles:"+pplist(current_l_arm_angs))

    #rospy.loginfo("joints_states_listener server started, waiting for queries")
    #rospy.spin()
