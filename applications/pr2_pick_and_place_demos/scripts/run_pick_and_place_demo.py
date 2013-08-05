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

#Pick and place demo: picks objects up from one side of the table and moves 
#them to the other, then switches sides

import roslib
roslib.load_manifest('pr2_pick_and_place_demos')
import rospy
import actionlib
from pr2_pick_and_place_demos.pick_and_place_demo import *
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal

if __name__ == '__main__':

    rospy.init_node('pick_and_place_demo', anonymous=False)

    use_slip_controller = rospy.get_param('/reactive_grasp_node_right/use_slip_controller', 0)
    use_slip_detection = rospy.get_param('/reactive_grasp_node_right/use_slip_detection', 0)
    just_start = rospy.get_param("~just_start", True)

    pick_and_place_demo = PickAndPlaceDemo(use_slip_controller = use_slip_controller, \
                                               use_slip_detection = use_slip_detection)
    if just_start:
        #lift the torso all the way up
        torso_action_client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction);
        rospy.loginfo("run_pick_and_place_demo: waiting for torso action server")
        torso_action_client.wait_for_server()
        rospy.loginfo("run_pick_and_place_demo: torso action server found")
        goal = SingleJointPositionGoal()
        goal.position = 0.295
        rospy.loginfo("sending command to lift torso")
        torso_action_client.send_goal(goal)
        torso_action_client.wait_for_result(rospy.Duration(30))

        #start the pick and place demo
        rospy.loginfo("starting pick and place demo")
        pick_and_place_demo.start_autonomous_thread(True)
        rospy.spin()
    else:
        rospy.loginfo("starting pick and place keyboard interface")
        pick_and_place_demo.keyboard_interface()
