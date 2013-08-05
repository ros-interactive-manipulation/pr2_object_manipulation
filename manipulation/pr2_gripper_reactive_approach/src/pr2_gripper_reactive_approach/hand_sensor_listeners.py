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

## @package hand_sensor_listeners
#Listen to and keep track of fingertip sensor readings

from __future__ import division
import roslib
roslib.load_manifest('pr2_gripper_reactive_approach')
import rospy
import time
import scipy
import threading
import copy
import pdb
import math
from pr2_msgs.msg import PressureState

##listens to PressureState messages 
class FingertipSensorListener:

  def __init__(self, gripper): #gripper is 'r' or 'l'
    self.gripper = gripper
    self.lock = threading.Lock()

    #raw values
    self.l_readings = [0]*22
    self.r_readings = [0]*22

    #thresholded values
    self.l_thres = [0]*22
    self.r_thres = [0]*22

    #thresholds to use (updated shortly in set_thresholds)
    self.l_threshold = [1]*22
    self.r_threshold = [1]*22

    #threshold parameters
    self.side_threshold_buffer = 125
    self.front_threshold_buffer = 225
    self.threshold_buffers = [self.side_threshold_buffer]*7 + [self.front_threshold_buffer]*15

    #subscribe to PressureState messages for this gripper
    rospy.Subscriber("pressure/"+gripper+"_gripper_motor", PressureState, self.sensor_callback)

    #grab some readings and set the thresholds
    time.sleep(2.0)
    self.set_thresholds(short = 1)

    #uncomment to test how sensitive the thresholds are
    #self.test_sensor_thresholds()


  ##callback: store the PressureState messages as they come in
  def sensor_callback(self, pressurestate):

    self.lock.acquire()    
    self.r_readings = pressurestate.r_finger_tip[:]
    self.l_readings = pressurestate.l_finger_tip[:]
    self.lock.release()

  ##get copies of the thresholded readings
  def get_thresholded_readings(self):
    self.lock.acquire()

    #using simple threshold: zero out entries less than self.threshold
    l_thres = [(x-t if x >= t else 0.) for \
                      (x,t) in zip(self.l_readings, self.l_threshold)]
    r_thres = [(x-t if x >= t else 0.) for \
                      (x,t) in zip(self.r_readings, self.r_threshold)]
    self.lock.release()

    return (l_thres, r_thres)


  ##get copies of the raw readings
  def get_readings(self):
    self.lock.acquire()
    l_readings_copy = self.l_readings[:]
    r_readings_copy = self.r_readings[:]
    self.lock.release()

    return (l_readings_copy, r_readings_copy)


  ##get the readings relative to the thresholds (no zeroing of values that are below)
  def get_relative_readings(self):
    self.lock.acquire()
    l_rel = [x-t for (x,t) in zip(self.l_readings, self.l_threshold)]
    r_rel = [x-t for (x,t) in zip(self.r_readings, self.r_threshold)]
    self.lock.release()
    
    return (l_rel, r_rel)


  ##run this when you're sure you're not touching anything, to set the noise threshold
  def set_thresholds(self, short = 0):

    #gather data for awhile and use the max value to set the thresholds
    l_readings_max = [0.]*22
    r_readings_max = [0.]*22

    num_readings = 50
    if short:
      num_readings = 10

    for i in range(num_readings):
      (l_readings, r_readings) = self.get_readings()
      l_readings_max = [max(x,y) for (x,y) in zip(l_readings_max, l_readings)]
      r_readings_max = [max(x,y) for (x,y) in zip(r_readings_max, r_readings)]
      time.sleep(.05)

    #only increase the thresholds, if doing a short reset
    self.lock.acquire()
    if short:
      self.l_threshold = [max(new+buffer, current_thres) for (new, current_thres, buffer) in zip(l_readings_max, self.l_threshold, self.threshold_buffers)]
      self.r_threshold = [max(new+buffer, current_thres) for (new, current_thres, buffer) in zip(r_readings_max, self.r_threshold, self.threshold_buffers)]
    else:
      self.l_threshold = [x + buffer for (x, buffer) in zip(l_readings_max, self.threshold_buffers)]
      self.r_threshold = [x + buffer for (x, buffer) in zip(r_readings_max, self.threshold_buffers)]
    self.lock.release()

    rospy.loginfo("left tip thresholds:"+str(self.l_threshold))
    rospy.loginfo("right tip thresholds:"+str(self.r_threshold))


  ##test sensor sensitivity
  def test_sensor_thresholds(self):
    rospy.loginfo("testing the sensor thresholds")
    while(not rospy.is_shutdown()):
      self.print_sensor_status()
      time.sleep(.1)


  ##pretty-print list to string
  def pplist(self, list):
    return ' '.join(['%0.3f'%x for x in list])


  ##print the current readings, thresholded and categorized
  def print_sensor_status(self, l_readings = None, r_readings = None):
    (l_readings, r_readings, l_regions_touching, r_regions_touching, l_rows_touching, r_rows_touching, \
         l_columns_touching, r_columns_touching) = self.get_sensor_status(l_readings, r_readings)

    if any(l_readings):
        rospy.loginfo("left")
        rospy.loginfo("l_readings: " + self.pplist(l_readings))
        rospy.loginfo("l_regions_touching: " + str(l_regions_touching))
        rospy.loginfo("l_rows_touching: " + str(l_rows_touching))
        rospy.loginfo("l_columns_touching: " + str(l_columns_touching))

    if any(r_readings):
        rospy.loginfo("          right")
        rospy.loginfo("          r_readings: " + self.pplist(r_readings))
        rospy.loginfo("          r_regions_touching: " + str(r_regions_touching))
        rospy.loginfo("          r_rows_touching: " + str(r_rows_touching))
        rospy.loginfo("          r_columns_touching: " + str(r_columns_touching))


  ##get readings, thresholded and categorized
  def get_sensor_status(self, l_readings = None, r_readings = None):
    if l_readings == None or r_readings == None:
      (l_readings, r_readings) = self.get_thresholded_readings()
    (l_regions_touching, r_regions_touching) = self.regions_touching(l_readings, r_readings)
    (l_rows_touching, r_rows_touching) = self.front_rows_touching(l_readings, r_readings)
    (l_columns_touching, r_columns_touching) = self.front_columns_touching(l_readings, r_readings)
    return (l_readings, r_readings, l_regions_touching, r_regions_touching, l_rows_touching, r_rows_touching, \
              l_columns_touching, r_columns_touching)


  ##report whether various regions are touching (tip, plus_z_side, neg_z_side, front, back) for each sensor
  def regions_touching(self, l_readings = None, r_readings = None):
    l_region_elements = [[2,3,4,5], [1,], [6,], [7,8,9,10,11,12,13,14,15,16,17,18,19,20,21], [0]]
    r_region_elements = [[2,3,4,5], [6,], [1,], [7,8,9,10,11,12,13,14,15,16,17,18,19,20,21], [0]]
 
    if l_readings == None or r_readings == None:
      (l_readings, r_readings) = self.get_thresholded_readings()
    l_regions_touching = []
    for region in l_region_elements:
      l_regions_touching.append(any([l_readings[element] for element in region]))

    r_regions_touching = []
    for region in r_region_elements:
      r_regions_touching.append(any([r_readings[element] for element in region]))
 
    return (l_regions_touching, r_regions_touching)


  ##report whether each row of 3 elements on each sensor's front (from base to tip) is in contact
  def front_rows_touching(self, l_readings = None, r_readings = None):
    if l_readings == None or r_readings == None:
      (l_readings, r_readings) = self.get_thresholded_readings()
    l_rows_touching = [0]*5
    r_rows_touching = [0]*5

    for i in range(5):
      if any(l_readings[i*3+7:(i+1)*3+7]):
        l_rows_touching[i] = 1
      if any(r_readings[i*3+7:(i+1)*3+7]):
        r_rows_touching[i] = 1
    
    l_rows_touching.reverse()
    r_rows_touching.reverse()

#     rospy.loginfo("l_rows_touching: "+str(l_rows_touching))
#     rospy.loginfo("r_rows_touching: "+str(r_rows_touching))

    return (l_rows_touching, r_rows_touching)
    
   
  ##report whether each column of 5 elements on each sensor's front (from +z side to -z side) is in contact
  def front_columns_touching(self, l_readings = None, r_readings = None):
    if l_readings == None or r_readings == None:
      (l_readings, r_readings) = self.get_thresholded_readings()
    l_columns_touching = [0]*3
    r_columns_touching = [0]*3

    for i in range(3):
      l_column_elements = [x*3+7+i for x in range(5)]
      r_column_elements = [x*3+9-i for x in range(5)]
      if scipy.array(l_readings)[l_column_elements].any():
        l_columns_touching[i] = 1
      if scipy.array(r_readings)[r_column_elements].any():
        r_columns_touching[i] = 1

#     rospy.loginfo("l_columns_touching: "+str(l_columns_touching))
#     rospy.loginfo("r_columns_touching: "+str(r_columns_touching))

    return (l_columns_touching, r_columns_touching)


    
if __name__ == "__main__":
  rospy.init_node('hand_sensor_listeners', anonymous=True)

  listener = FingertipSensorListener('r')
  #listener = FingertipSensorListener('l')
  
  listener.test_sensor_thresholds()
