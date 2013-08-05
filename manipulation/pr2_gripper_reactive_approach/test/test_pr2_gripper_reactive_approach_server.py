#!/usr/bin/python
"""test client for pr2_gripper_reactive_approach_server
requires pr2_gripper_reactive_approach_server.py r (right arm) to be running
"""

from __future__ import division
import roslib
roslib.load_manifest('pr2_gripper_reactive_approach')
import rospy
import math
from geometry_msgs.msg import PoseStamped, PointStamped, \
    QuaternionStamped, Pose, Point, Quaternion
from object_manipulation_msgs.msg import ReactiveGraspAction, \
    ReactiveGraspGoal, ReactiveLiftAction, ReactiveLiftGoal, \
    GripperTranslation, ReactivePlaceAction, ReactivePlaceGoal
import actionlib
from trajectory_msgs.msg import JointTrajectory
from pr2_gripper_reactive_approach import controller_manager
from std_srvs.srv import Empty
from object_manipulator.convert_functions import *


#call reactive_grasp to do a reactive grasp using the fingertip sensors
#(backs up and move to the side if the tip contacts on the way to the grasp,
#does a compliant grasp if it gets there, tries the approach and grasp 
#several times if the gripper opening isn't within bounds)
def call_reactive_grasp(ac, grasp_pose, trajectory):

    if trajectory == None:
        trajectory = JointTrajectory()

    goal = ReactiveGraspGoal()
    goal.final_grasp_pose = grasp_pose
    goal.trajectory = trajectory
    goal.collision_support_surface_name = "table"

    ac.send_goal(goal)
    ac.wait_for_result()
    result = ac.get_result()
    print "reactive grasp result:", result

    return result.manipulation_result


#call reactive_grasp to do a reactive approach using the fingertip sensors
#(backs up and move to the side if the tip contacts on the way to the grasp)
def call_reactive_approach(ac, grasp_pose, trajectory):

    if trajectory == None:
        trajectory = JointTrajectory()

    goal = ReactiveGraspGoal()
    goal.final_grasp_pose = grasp_pose
    goal.trajectory = trajectory
    goal.collision_support_surface_name = "table"

    ac.send_goal(goal)
    ac.wait_for_result()
    result = ac.get_result()
    print "reactive approach result:", result

    return result.manipulation_result


#call reactive lift (starts up slip servoing if using the slip controllers, 
#otherwise just a Cartesian move)
def call_reactive_lift(ac, lift):

    goal = ReactiveLiftGoal()
    goal.lift = lift

    ac.send_goal(goal)
    ac.wait_for_result()
    result = ac.get_result()
    print "reactive lift result:", result

    return result


#call reactive place (uses the slip controllers to detect when the object
#hits the table, and opens the gripper)
def call_reactive_place(ac, place_pose):

    goal = ReactivePlaceGoal()
    goal.final_place_pose = place_pose

    ac.send_goal(goal)
    ac.wait_for_result()
    result = ac.get_result()
    print "reactive place result:", result

    return result


#pause for input
def keypause():
    print "press enter to continue"
    input = raw_input()
    return input


#move to a Cartesian pose goal
def move_cartesian_step(cm, pose, timeout = 10.0,
                        settling_time = 3.0, blocking = 0):
    if type(pose) == list:
        pose = create_pose_stamped(pose, 'base_link')
        cm.move_cartesian(pose, blocking = blocking, \
                   pos_thres = .0025, rot_thres = .05, \
                   timeout = rospy.Duration(timeout), \
                   settling_time = rospy.Duration(settling_time))


#used to call compliant close or grasp adjustment
def call_empty_service(service_name, serv):
    try:
        serv()
    except rospy.ServiceException, e:
        rospy.logerr("error when calling %s,%s"%(service_name,e))
        return 0
    return 1


#return the current pos and rot of the wrist for the right arm 
#as a 7-list (pos, quaternion rot)
def return_current_pose_as_list(cm):
    (pos, rot) = cm.return_cartesian_pose()
    return pos+rot


##convert a relative vector in frame to a pose in the base_link frame
#if start_pose is not specified, uses current pose of the wrist
def return_rel_pose(cm, vector, frame, start_pose = None):

    #if start_pose is not specified, use the current Cartesian pose of the wrist
    if start_pose == None:
        (start_trans, start_rot) = cm.return_cartesian_pose('base_link')
    else:
        start_pose.header.stamp = rospy.Time(0)
        (start_trans, start_rot) = pose_stamped_to_lists(cm.tf_listener, start_pose, 'base_link')

    #convert the vector in frame to base_link frame
    if frame != 'base_link':
        vector3_stamped = create_vector3_stamped(vector, frame)
        base_link_vector = vector3_stamped_to_list(cm.tf_listener, vector3_stamped, 'base_link')    
    else:
        base_link_vector = vector

    #add the desired vector to the position
    new_trans = [x+y for (x,y) in zip(start_trans, base_link_vector)]

    #create a new poseStamped
    pose_stamped = create_pose_stamped(new_trans+start_rot)

    return pose_stamped


#run the test
if __name__ == "__main__":

    rospy.init_node('test_reactive_grasp_server', anonymous = True)

    use_slip_detection = rospy.get_param('/reactive_grasp_node_right/use_slip_controller', 0)
    use_slip_controller = rospy.get_param('/reactive_grasp_node_right/use_slip_detection', 0)
    rospy.loginfo("use_slip_controller:"+str(use_slip_controller))
    rospy.loginfo("use_slip_detection:"+str(use_slip_detection))

    rospy.loginfo("waiting for compliant_close and grasp_adjustment services")
    rospy.wait_for_service("r_reactive_grasp/compliant_close")
    rospy.wait_for_service("r_reactive_grasp/grasp_adjustment")
    rospy.loginfo("service found")

    rg_ac = actionlib.SimpleActionClient("reactive_grasp/right", \
                                             ReactiveGraspAction)
    ra_ac = actionlib.SimpleActionClient("reactive_approach/right", \
                                             ReactiveGraspAction)
    rl_ac = actionlib.SimpleActionClient("reactive_lift/right", \
                                             ReactiveLiftAction)
    rp_ac = actionlib.SimpleActionClient("reactive_place/right", \
                                             ReactivePlaceAction)
    cc_srv = rospy.ServiceProxy("r_reactive_grasp/compliant_close", Empty)
    ga_srv = rospy.ServiceProxy("r_reactive_grasp/grasp_adjustment", Empty)

    rospy.loginfo("waiting for reactive grasp action")
    rg_ac.wait_for_server()
    rospy.loginfo("reactive grasp action found")

    rospy.loginfo("waiting for reactive approach action")
    ra_ac.wait_for_server()
    rospy.loginfo("reactive approach action found")

    rospy.loginfo("waiting for reactive lift action")
    rl_ac.wait_for_server()
    rospy.loginfo("reactive lift action found")

    rospy.loginfo("waiting for reactive place action")
    rp_ac.wait_for_server()
    rospy.loginfo("reactive place action found")

    cm = controller_manager.ControllerManager('r', \
                             using_slip_controller = use_slip_controller, \
                             using_slip_detection = use_slip_detection)

    #joint names for the right arm
    joint_names = ["r_shoulder_pan_joint",
                   "r_shoulder_lift_joint",
                   "r_upper_arm_roll_joint",
                   "r_elbow_flex_joint",
                   "r_forearm_roll_joint",
                   "r_wrist_flex_joint",
                   "r_wrist_roll_joint"]

    #reactive approach from above 
    #table_height = .55  #simulated table
    table_height = .7239 #round table
    tip_dist_to_table = .165
    wrist_height = table_height + tip_dist_to_table + .02

    approachpos = [.52, -.05, wrist_height+.1]
    approachquat = [0.5, 0.5, -0.5, 0.5]  #from the top
    sideangles = [-0.447, -0.297, -2.229, -0.719, 0.734, -1.489, -1.787]
    side_tip_dist_to_table = .06 
    sideapproachpos = [.63, -.3, table_height-.035+side_tip_dist_to_table]
    sideapproachquat = [.707, .707, 0, 0]
    current_goal = approachpos[:] + approachquat[:]
    small_step = .02
    approach_type = 'top'

    while(not rospy.is_shutdown()):

        #update current goal to current pose
        current_goal = return_current_pose_as_list(cm)

        #pregrasp_pose = create_pose_stamped(current_goal)
        if approach_type == 'top':
            pregrasp_pose = create_pose_stamped(approachpos+approachquat)
        else:
            pregrasp_pose = create_pose_stamped(sideapproachpos+sideapproachquat)

        print "enter:"
        print "side to switch to side approach, top to switch to top approach"
        print "appi to go to the approach position with interpolated IK"
        print "appc to go to the approach position with Cartesian controllers"
        print "appm to go to the approach position with move_arm"
        print "rg to grasp reactively, ra to just approach reactively, gl to grasp and lift"
        print "cc to do a compliant close"
        print "rl to do a reactive lift"
        print "rp to do a reactive place"
        print "ga to do a grasp adjustment"
        print "c to close, o to open"
        print "u to go up 10 cm, d to go down 10 cm"
        print "small steps: us to go up, ds to go down, r to go right, l to go left,"
        print "     a to go away from the robot, t to go toward the robot"
        print "j to go to a set of nearish-side-grasp angles"
        print "s to stop"
        c = keypause()
        if c == 'side':
            approach_type = 'side'
        elif c == 'top':
            approach_type = 'top'
        elif c == 'appi':
            #Go to the approach position using the interpolated 
            #IK controllers (collision-aware)
            cm.command_interpolated_ik(pregrasp_pose)
        elif c == 'appc':
            #Go to the approach position using the Cartesian controllers
            cm.command_cartesian(pregrasp_pose)
        elif c == 'appm':
            #Go to the approach position using move_arm
            cm.move_arm_pose(pregrasp_pose, blocking = 1)
        elif c == 'j':
            print "moving to near-side-grasp angles"
            cm.command_joint(sideangles)

        elif c == 'rg' or c == 'ra' or c == 'gl':

            #set the goal to be 10 cm along the current gripper x-axis
            grasp_pose = return_rel_pose(cm, [.1, 0, 0], 'r_wrist_roll_link')

            # grasp_goal = current_goal[:]
            # grasp_goal[2] -= .1
            # grasp_pose = create_pose_stamped(grasp_goal)

            start_angles = [0.]*7
            trajectory = None
            if c == 'rg' or c == 'gl':
                print "calling reactive grasp"
                result = call_reactive_grasp(rg_ac, grasp_pose, trajectory)
            else:
                print "calling reactive approach"
                result = call_reactive_approach(ra_ac, grasp_pose, trajectory)
            print "result:", result

            if c == 'gl':
                if result.value == 1:
                    print "calling reactive lift"
                    lift = GripperTranslation()
                    lift.direction = create_vector3_stamped([0,0,1])
                    lift.desired_distance = .1
                    lift.min_distance = 0.
                    result = call_reactive_lift(rl_ac, lift)
                else:
                    print "grasp didn't return success, opening gripper"
                    cm.command_gripper(.087, -1)

        elif c == 'rl':
            print "calling reactive lift"
            lift = GripperTranslation()
            lift.direction = create_vector3_stamped([0,0,1])
            lift.desired_distance = .1
            lift.min_distance = 0.
            result = call_reactive_lift(rl_ac, lift)

        elif c == 'rp':
            print "calling reactive place"
            place_goal = current_goal[:]
            place_goal[2] -= .1
            place_pose = create_pose_stamped(place_goal)
            result = call_reactive_place(rp_ac, place_pose)

        elif c == 'cc':
            print "calling compliant close"
            call_empty_service("compliant close", cc_srv)
        elif c == 'ga':
            print "calling grasp adjustment"
            call_empty_service("grasp adjustment", ga_srv)
        elif c == 'c':
            print "closing gripper"
            cm.command_gripper(0, 40.0)
        elif c == 'o':
            print "opening gripper"
            cm.command_gripper(.087, -1)
        elif c == 'u':
            print "going up"
            current_goal[2] += .1
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 'us':
            print "going up a small amount"
            current_goal[2] += small_step
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 'd':
            print "going down"
            current_goal[2] -= .05
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 'ds':
            print "going down a small amount"
            current_goal[2] -= small_step
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 'r':
            print "moving right"
            current_goal[1] -= small_step
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 'l':
            print "moving left"
            current_goal[1] += small_step
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 'a':
            print "moving away from robot"
            current_goal[0] += small_step
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 't':
            print "moving toward the robot"
            current_goal[0] -= small_step
            move_cartesian_step(cm, current_goal, blocking = 1)
        elif c == 's':
            print "quitting"
            break

        #update current goal to current pose
        current_goal = return_current_pose_as_list(cm)
