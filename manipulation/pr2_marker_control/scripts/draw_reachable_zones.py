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
Draws circles around the pr2 shoulders to show distances to the wrist either continuously or when asked to
"""

import roslib
roslib.load_manifest('pr2_marker_control')
import rospy
import object_manipulator.draw_functions
import scipy
from std_msgs.msg import Empty
import sys

#draw horizontal circles at shoulder-level
def draw_circles(duration):
    draw_functions.draw_rviz_circle(scipy.matrix(scipy.identity(4)), 0.82, frame = 'l_shoulder_pan_link', 
                                    ns = 'left_wrist', id = 0, duration = duration, color = [0,1,1], opaque = 1, 
                                    frame_locked = True)
    draw_functions.draw_rviz_circle(scipy.matrix(scipy.identity(4)), 0.82, frame = 'r_shoulder_pan_link', 
                                    ns = 'right_wrist', id = 0, duration = duration, color = [1,0,1], opaque = 1, 
                                    frame_locked = True)

#draw continuously
def draw_continuous():
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        draw_circles(0.)
        rate.sleep()

#callback to draw when you see a ping message
def callback(msg):
    print "drawing reachable zones"
    draw_circles(20.)

if __name__ == '__main__':
    rospy.init_node('draw_reachable_zones')
    marker_topic = "grasp_markers"
    draw_functions = object_manipulator.draw_functions.DrawFunctions(marker_topic, latch=False)

    if len(sys.argv) >= 2 and sys.argv[1] == "c":
        print 'drawing reachable zones continuously'
        draw_continuous()
    else:
        rospy.Subscriber("draw_reachable_zones/ping", Empty, callback)
        print "subscribed to draw_reachable_zones/ping"
        rospy.spin()

