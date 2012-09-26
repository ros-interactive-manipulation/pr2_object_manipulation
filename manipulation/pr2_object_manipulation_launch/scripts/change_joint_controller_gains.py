#! /usr/bin/env python

import roslib; roslib.load_manifest('pr2_object_manipulation_launch')
import rospy

from pr2_mechanism_msgs.srv import SwitchController, LoadController, UnloadController, ListControllers

#load the params for the joint controllers onto the param server
class JointParamChanger:

    def __init__(self, whicharm): #whicharm is 'r' or 'l'
        self.whicharm = whicharm
        self.default_p = [2400., 1200., 1000., 700., 300., 300., 300.]
        self.default_d = [18., 10., 6., 4., 6., 4., 4.]
        self.default_i = [800., 700., 600., 450., 300., 300., 300.]
        self.default_i_clamp = [4., 4., 4., 4., 2., 2., 2.]
        joint_names = ['_shoulder_pan_joint', '_shoulder_lift_joint',
                         '_upper_arm_roll_joint', '_elbow_flex_joint',
                         '_forearm_roll_joint', '_wrist_flex_joint',
                         '_wrist_roll_joint']
        self.controller_name = whicharm+"_arm_controller"
        self.joint_names = [whicharm+joint_name for joint_name in joint_names]

        rospy.loginfo("waiting for pr2_controller_manager services")
        rospy.wait_for_service('pr2_controller_manager/load_controller')
        rospy.wait_for_service('pr2_controller_manager/unload_controller')
        rospy.wait_for_service('pr2_controller_manager/switch_controller')

        self.load_controller_service = \
            rospy.ServiceProxy('pr2_controller_manager/load_controller', LoadController)
        self.unload_controller_service = \
            rospy.ServiceProxy('pr2_controller_manager/unload_controller', UnloadController)
        self.switch_controller_service = \
            rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
        self.joint_controller = self.whicharm+'_arm_controller'


    #set gains to some fraction of the default
    def set_gains_fraction(self, fraction):
        for i in range(7):
            rospy.set_param(self.controller_name+'/gains/%s/p'%self.joint_names[i], self.default_p[i]*fraction)
            rospy.set_param(self.controller_name+'/gains/%s/d'%self.joint_names[i], self.default_d[i]*fraction)
            rospy.set_param(self.controller_name+'/gains/%s/i'%self.joint_names[i], self.default_i[i]*fraction)
            rospy.set_param(self.controller_name+'/gains/%s/i_clamp'%self.joint_names[i], self.default_i_clamp[i]*fraction)
            

    #unload and reload the joint controllers with the params on the server
    def reload_joint_controllers(self, fraction = 1.):
        self.set_gains_fraction(fraction)

        #unload the joint controller
        self.unload_joint_controllers()
  
        #re-load the joint controllers
        self.load_joint_controllers()

        #start the joint controllers
        self.start_joint_controllers()


    #load the joint controller with the current set of params on the param server
    def load_joint_controllers(self):
        success = self.load_controller_service(self.joint_controller)
        if not success:
            rospy.logerr("error in loading joint controller!")
        else:
            rospy.loginfo("loaded joint controller")


    #unload the joint controller 
    def unload_joint_controllers(self):
        self.stop_joint_controllers()

        success = self.unload_controller_service(self.joint_controller)
        if not success:
            rospy.logerr("error in unloading joint controller!")
        else:
            rospy.loginfo("unloaded joint controller")


    #stop controllers that are currently running
    def stop_joint_controllers(self):
        success = self.switch_controller_service([], [self.joint_controller,], 2)
        if success:
            rospy.loginfo("stopped joint controller successfully")    
        else:
            rospy.logerr("stopping joint controller failed")  


    #just start the joint controllers (need to be loaded already)
    def start_joint_controllers(self):
        success = self.switch_controller_service([self.joint_controller,], [], 2)
        if success:
            rospy.loginfo("started joint controller successfully")
        else:
            rospy.logerr("starting joint controller failed")


if __name__ == "__main__":
    r_joint_params = JointParamChanger('r')
    l_joint_params = JointParamChanger('l')

    r_joint_params.reload_joint_controllers()
    l_joint_params.reload_joint_controllers()



