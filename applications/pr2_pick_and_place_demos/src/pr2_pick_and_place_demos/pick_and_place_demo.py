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

## @package pick_and_place_demo
#Pick and place demo: picks objects up from one side of the table and moves 
#them to the other, then switches sides

import roslib
roslib.load_manifest('pr2_pick_and_place_demos')
import rospy
from pr2_pick_and_place_demos.pick_and_place_manager import *


##class for running the pick and place demo, inherits from PickAndPlaceManager
class PickAndPlaceDemo(PickAndPlaceManager):
    
    
    def __init__(self, use_slip_controller = 0, use_slip_detection = 0):

        #run the standard init
        PickAndPlaceManager.__init__(self, use_slip_controller, use_slip_detection)
        
        #which side we're getting objects from (right = 0, left = 1, placing is on the other side)
        self.pick_up_side = 0
        self.put_down_side = 1

        #(x,y) dimensions of each table area (side) that we want to place/pick up from
        self.table_side_dims = [.3, .3]

        #middle line separating right and left (base_link y)
        self.middle_line = 0.

        #center of each table side in '/base_link' frame (0=right, 1=left, height and dist from table added later)
        self.table_center_offsets = [[self.table_side_dims[1]/2, -(self.table_side_dims[0]/2+.02)+self.middle_line, 0],
                              [self.table_side_dims[1]/2, self.table_side_dims[0]/2+.02+self.middle_line, 0]]

        #temporary table info until we get a table detection
        #centers of the table halves
        self.table_centers = copy.deepcopy(self.table_center_offsets)
        for i in range(2):
            self.table_centers[i][0] += self.table_front_edge_x + .05
            self.table_centers[i][2] += self.table_height

        #update where to point the head and draw the table centers
        self.update_head_point_locs()

        #set the place rectangle to the initial put-down side
        self.set_table_place_rectangle(self.put_down_side)

        #are we trying to preempt the autonomous thread?
        self.preempting = 0

        #lock for self.preempting and variable to keep track of whether we're in the autonomous thread
        self.lock = threading.Lock()
        self.autonomous_thread_running = 0

        #is the autonomous demo going to run in constrained mode or not?
        self.constrained = False


    ##update where to point the head based on the table centers
    def update_head_point_locs(self):

        #places to point the head to
        self.head_point_locs = copy.deepcopy(self.table_centers)
        self.head_point_locs[0][0] += .1  #forward of center
        self.head_point_locs[1][0] += .1
        #in a bit toward center (add more or less if the center is shifted)
        self.head_point_locs[0][1] += .05-self.middle_line/2.1  
        self.head_point_locs[1][1] -= .05+self.middle_line/1.2


    ##extended to update the table side regions
    def update_table_info(self, adjust_place_rectangle = 0):

        #update self.table_front_edge_x and self.table_height
        PickAndPlaceManager.update_table_info(self, adjust_place_rectangle)

        #centers of the table halves
        self.table_centers = copy.deepcopy(self.table_center_offsets)
        for i in range(2):
            self.table_centers[i][0] += self.table_front_edge_x + .05
            self.table_centers[i][2] += self.table_height

        #update where to point the head and draw the table centers
        self.update_head_point_locs()


    ##overridden to exit from the autonomous thread
    def throw_exception(self):                
        self.exit_autonomous_thread()

    
    ##move objects from one side of the table to the other
    def move_objects_to_other_side(self):

        #only try each object max_object_tries times
        max_object_tries = 5
        object_tries = 0
        while(not rospy.is_shutdown() and object_tries < max_object_tries):
            rospy.loginfo("object_tries:%d", object_tries)

            self.check_preempted()

            #head point location is the pick-up side
            head_point = copy.deepcopy(self.head_point_locs[self.pick_up_side])
            head_point[0] += scipy.rand()*.05
            head_point[1] += scipy.rand()*.05

            #try the arm corresponding to the pick-up side first, then the other arm (if available)
            if self.pick_up_side == 0 and self.arms_to_use == "both":
                arms_to_try = [0,1]
            elif self.pick_up_side == 1 and self.arms_to_use == "both":
                arms_to_try = [1,0]
            elif self.arms_to_use == "left":
                arms_to_try = [1]
            elif self.arms_to_use == "right":
                arms_to_try = [0]

            #detect and pick up an object
            (result, arm_used) = self.detect_and_pick_up_object(head_point, arms_to_try = arms_to_try, constrained = self.constrained)

            #no objects left!  Quit
            if result == 'no objects left':
                return 1
            
            #grasp failed.  Increment the number of object_tries and try again
            if result == 'grasp failed':
                object_tries += 1
                continue

            self.check_preempted()

#             #if the object is the bowl, try to transfer it to the other arm before putting it down
#             if self.held_objects[arm_used] and self.held_objects[arm_used].object.type == 1 \
#                 and self.held_objects[arm_used].object.model_pose.model_id == 18699:
#                 rospy.loginfo("detected bowl, trying to transfer it to the other hand before putting it down")
#                 success = self.transfer_object_to_other_hand(arm_used, self.held_objects[arm_used])
#                 if success:
#                     arm_used = 1-arm_used

            #success!  Reset the number of object_tries 
            object_tries = 0

            #place it on the other side
            self.put_down_object(arm_used, use_place_override = 1, constrained = self.constrained)

        #ran out of tries
        rospy.logerr("ran out of tries to move an object!")
        return 0


    ##filter/count how many objects in detected_objects are on whichside
    def count_objects(self, whichside = None):
        if whichside == None:
            whichside = self.pick_up_side

        object_count = 0
        kept_objects = []
        for object in self.detected_objects:
            dist = object.pose.pose.position.x-(self.table_centers[0][0]+self.table_side_dims[0]/2.)
            if whichside == 0 and object.pose.pose.position.y > self.middle_line + 0.01:
                rospy.loginfo("counting right: ignoring object not on the right side, y pos: %0.3f"%object.pose.pose.position.y)
                continue
            elif whichside == 1 and object.pose.pose.position.y < self.middle_line - 0.01:
                rospy.loginfo("counting left: ignoring object not on the left side, y pos: %0.3f"%object.pose.pose.position.y)
                continue            
            elif dist > .1:
                rospy.loginfo("ignoring object too far away, dist=%5.3f"%dist)
                continue
            else:
                rospy.loginfo("object dist from back edge is %5.3f"%dist)
            kept_objects.append(object)
        self.detected_objects = kept_objects

        return len(self.detected_objects)


    ##overridden temporarily to get rid of the single table object when there's nothing on the table
    def call_tabletop_detection(self, update_table = 0, clear_attached_objects = 1, \
                            replace_table_in_collision_map = 1, update_place_rectangle = 0, \
                            num_models = 0):
        (detected_objects, table) = PickAndPlaceManager.call_tabletop_detection(self, \
                                                 update_table, clear_attached_objects, \
                                                 replace_table_in_collision_map, update_place_rectangle, num_models)
        
        #remove table object
        if len(self.detected_objects) == 1 and self.detected_objects[0].box_dims[0] > .2 and self.detected_objects[0].box_dims[1] > .2:
            self.remove_object(self.detected_objects[0].collision_name)
            self.detected_objects = []
            rospy.loginfo("removed table object")

        return (self.detected_objects, table)



    ##choose the side of the table with more objects to be the side to pick them up from
    def pick_sides(self):

        #figure out which side has more objects
        object_counts = [0]*2
        for i in range(2):

            self.check_preempted()

            self.point_head(self.head_point_locs[i], 'base_link')

            self.check_preempted()

            self.call_tabletop_detection(update_table = 0, clear_attached_objects = 0)
            object_counts[i] = self.count_objects(i)

        rospy.loginfo("saw %d objects on the right and %d objects on the left"%(object_counts[0], object_counts[1]))

        #set the side with more objects to be the side to pick up from
        if object_counts[0] > object_counts[1]:
            rospy.loginfo("setting pick_up_side to the right side, put_down_side to the left")
            self.pick_up_side = 0
            self.put_down_side = 1
        else:
            rospy.loginfo("setting pick_up_side to the left side, put_down_side to the right")
            self.pick_up_side = 1
            self.put_down_side = 0

        #set the place rectangle to the put-down side
        self.set_table_place_rectangle(self.put_down_side)


    ##switch pick-up and put-down sides
    def switch_sides(self):

        #switch the flags
        self.put_down_side = self.pick_up_side
        self.pick_up_side = 1-self.put_down_side
        rospy.loginfo("switching sides!  pick-up side is now %s, put-down is %s"%(self.arm_dict[self.pick_up_side],\
                                                                                      self.arm_dict[self.put_down_side]))
        
        #update the place rectangle
        self.set_table_place_rectangle(self.put_down_side)


    ##set the place rectangle to one side of the table
    def set_table_place_rectangle(self, whichside):

        rect_pose_mat = scipy.identity(4)
        rect_pose_mat[0:3, 3] = scipy.matrix(self.table_centers[whichside])
        rect_pose_stamped = stamp_pose(mat_to_pose(rect_pose_mat), 'base_link')
        self.set_place_area(rect_pose_stamped, self.table_side_dims)


    ##demo to run continuously in the autonomous thread
    def run_demo(self):

        #move the arms to the side (out of the way of detection)
        for arm in self.arms_to_use_list:
            self.move_arm_to_side(arm)
            self.check_preempted()

        #find the table
        self.find_table()        
        self.check_preempted()

        #figure out which side the objects are mostly on
        self.pick_sides()
        self.check_preempted()

        #run continuously until shut down
        while not rospy.is_shutdown():

            rospy.loginfo("starting to move objects from %s side to %s side"%(self.arm_dict[self.pick_up_side], \
                                                                                  self.arm_dict[self.put_down_side]))
            result = self.move_objects_to_other_side()
            self.check_preempted()

            self.switch_sides()


    ##check if we want to preempt, and if so, stop and notify the user that we've done so
    def check_preempted(self, inside_thread = 1):
        preempting = 0
        paused = 0
        self.lock.acquire()
        preempting = self.preempting
        self.lock.release()

        #preempt requested, exit thread
        if preempting == 1 and inside_thread:
            rospy.loginfo("saw the preempt")
            self.reset_preempted()
            self.autonomous_thread_running = 0
            sys.exit(0)

        #paused, if inside thread, wait for main thread to reset the preempt
        if preempting == -1 and inside_thread:
            rospy.loginfo("pause received inside thread, press enter to continue or q to exit")
            while not rospy.is_shutdown():
                preempting = self.check_preempted(inside_thread = 0)
                if not preempting:
                    break
                elif preempting == 1:
                    rospy.loginfo("quit requested received inside thread, exiting")
                    self.reset_preempted()
                    self.autonomous_thread_running = 0
                    sys.exit(0)    
                time.sleep(.1)
            return 0

        return preempting


    ##reset the preempt notification 
    def reset_preempted(self):
        rospy.loginfo("resetting the preempt")
        self.lock.acquire()
        self.preempting = 0
        self.lock.release()


    ##tell the autonomous thread to start running the demo continuously
    def start_autonomous_thread(self,constrained = False):

        #start a thread to run the demo continously in a override-able way
        self.reset_preempted()
        self.constrained = constrained
        self.thread = threading.Thread(target=self.run_demo)
        self.autonomous_thread_running = 1
        self.thread.setDaemon(True)
        self.thread.start()


    ##stop the autonomous thread by telling it to stop as soon as possible
    def stop_autonomous_thread(self):

        #tell the autonomous thread to stop as soon as possible
        self.lock.acquire()
        self.preempting = 1
        self.lock.release()
        
        #wait for the preempt to finish
        rospy.loginfo("preempt sent, waiting for something to finish")
        while not rospy.is_shutdown():
            if not self.check_preempted(inside_thread = 0):
                break
            time.sleep(.1)
        rospy.loginfo("autonomous thread ended")


    ##pause the autonomous thread as soon as possible
    def pause_autonomous_thread(self):

        #tell the autonomous thread to pause as soon as possible
        self.lock.acquire()
        self.preempting = -1
        self.lock.release()
        
        #wait for the pause to finish
        rospy.loginfo("pause sent, waiting for something to finish (press enter to continue, q to quit)")
        c = raw_input()
        if c == 'q':
            self.stop_autonomous_thread()
            return 1
        self.reset_preempted()
        rospy.loginfo("pause done")
        return 0
    

    ##exit from within the autonomous thread, if running 
    def exit_autonomous_thread(self):
        if self.autonomous_thread_running:
            print "saw a bad error, exiting the autonomous thread (hit q to go back to the keyboard interface)"
            self.autonomous_thread_running = 0
            sys.exit(0)
        else:
            rospy.logerr("saw a bad error!!  Not within autonomous thread; be careful, things may be very broken.")


    ##instructions for extensions to the keyboard interface specific to the demo
    def print_keyboard_extensions(self):
        print "\'start\' to start the autonomous demo"
        print "s to switch pick-up and put-down sides"
        print "hs to point the head at either side"


    ##extensions to the keyboard interface specific to the demo
    def keyboard_extensions(self, input):

        #run the autonomous thread
        if input == 'start':
            self.start_autonomous_thread(False)    

            while not rospy.is_shutdown():
                print "type s to preempt the autonomous demo, p to pause"
                input = raw_input()
                if input.strip() == 's':
                    self.stop_autonomous_thread()
                    break
                elif input.strip() == 'p':
                    exit = self.pause_autonomous_thread()                        
                    if exit:
                        break
                elif input.strip() == 'q':
                    break
                time.sleep(0.2)

        #switch the pick-up and put-down sides
        elif input == 's':
            self.switch_sides()

        #point the head at either the right or left side
        elif input == 'hs':
            print "which side?  r for right and l for left"
            side = self.input_side()
            if side != None and side >= 0:
                self.point_head(self.head_point_locs[side], 'base_link')

        return 0


if __name__ == '__main__':

    use_slip_controller = rospy.get_param('/reactive_grasp_node_right/use_slip_controller', False)
    use_slip_detection = rospy.get_param('/reactive_grasp_node_right/use_slip_detection', False)

    rospy.init_node('pick_and_place_demo', anonymous=True)
    pick_and_place_demo = PickAndPlaceDemo(use_slip_controller = use_slip_controller, 
                                           use_slip_detection = use_slip_detection)
    pick_and_place_demo.keyboard_interface()
