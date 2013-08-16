import roslib; roslib.load_manifest('pr2_gripper_reactive_approach')
import rospy
import pdb
from object_manipulation_msgs.msg import CartesianGains
from std_msgs.msg import Float64MultiArray

##load the params for the joint controllers onto the param server
class JointParams:

  def __init__(self, whicharm): #whicharm is 'r' or 'l'
    self.whicharm = whicharm
    self.default_p = [800., 800., 200., 100., 500., 400., 400.]
    self.default_d = [10., 10., 3., 3., 6., 4., 4.]
    joint_names = ['_shoulder_pan_joint', '_shoulder_lift_joint',
                     '_upper_arm_roll_joint', '_elbow_flex_joint',
                     '_forearm_roll_joint', '_wrist_flex_joint',
                     '_wrist_roll_joint']
    self.controller_name = whicharm+"_arm_controller"
    self.joint_names = [whicharm+joint_name for joint_name in joint_names]

  ##set gains to some fraction of the default
  def set_gains_fraction(self, fraction):
    for i in range(7):
      rospy.set_param(self.controller_name+'/gains/%s/p'%self.joint_names[i], self.default_p[i]*fraction)
      rospy.set_param(self.controller_name+'/gains/%s/d'%self.joint_names[i], self.default_d[i]*fraction)


##load the params for the Jtranspose Cartesian controllers (jt_task_controller) onto the param server
class JTCartesianTaskParams:

  def __init__(self, whicharm, task_controller = 0): #whicharm is 'r' or 'l'
    self.whicharm = whicharm
    self.rot_d = [1.2]*3
    self.rot_p = [80.0]*3
    self.trans_d = [15.0]*3
    self.trans_p = [800.0]*3
    self.cname = self.whicharm+'_cart'
    self.gains_topic = self.cname+'/gains'
    self.tip_frame = self.whicharm+"_wrist_roll_link"
    self.base_frame = "/base_link"
    self.gains_frame = "/base_link"
    self.task_controller = task_controller
    print "task_controller: ", self.task_controller

    #publisher for setting gains
    if self.task_controller:
      self.set_gains_pub = rospy.Publisher(self.gains_topic, CartesianGains)
    else:
      self.set_gains_pub = rospy.Publisher(self.gains_topic, Float64MultiArray)

    #set the default values on the parameter server and send the default gains
    self.set_params()
    self.set_gains()


  ##set the parameter values back to their defaults
  def set_params_to_defaults(self, set_tip_frame = 1, set_params_on_server = 1, set_gains = 1):
    self.rot_d = [1.2]*3
    self.rot_p = [80.0]*3
    self.trans_d = [15.0]*3
    self.trans_p = [800.0]*3
    if set_tip_frame:
      self.tip_frame = self.whicharm+"_wrist_roll_link"
    if set_params_on_server:
      self.set_params()
    elif set_gains:
      self.set_gains()


  ##set the gains and max vels and accs lower, so the arm moves more gently/slowly
  def set_params_to_gentle(self, set_tip_frame = 1, set_params_on_server = 1, set_gains = 1):
    self.rot_d = [0.6]*3
    self.rot_p = [40.0]*3
    self.trans_d = [7.5]*3
    self.trans_p = [400.0]*3
    if set_tip_frame:
      self.tip_frame = self.whicharm+'_wrist_roll_link'
      rospy.loginfo("setting tip frame back to wrist_roll_link")
    if set_params_on_server:
      self.set_params()
    elif set_gains:
      self.set_gains()


  ##set the current parameter values on the parameter server
  def set_params(self):
    #disabled until controller's changing of tip frame is fixed (https://code.ros.org/trac/wg-ros-pkg/ticket/5134)
    #rospy.loginfo("setting Cartesian controller parameters")
    #rospy.set_param(self.cname+'/root_name', self.base_frame)
    #rospy.set_param(self.cname+'/tip_name', self.tip_frame)
    self.set_gains()

    
  ##set the current controller gains by calling the setGains service
  def set_gains(self, trans_p = None, trans_d = None, rot_p = None, rot_d = None, frame_id = None):
    if self.task_controller:
      gains = CartesianGains()
      if frame_id != None:
        gains.header.frame_id = frame_id
      else:
        gains.header.frame_id = self.gains_frame
    else:
      gains = Float64MultiArray()
    gains_array = [0]*12
    if trans_p != None:
        gains_array[0:3] = trans_p
    else:
        gains_array[0:3] = self.trans_p
    if rot_p != None:
        gains_array[3:6] = rot_p
    else:
        gains_array[3:6] = self.rot_p
    if trans_d != None:
        gains_array[6:9] = trans_d
    else:
        gains_array[6:9] = self.trans_d
    if rot_d != None:
        gains_array[9:12] = rot_d
    else:
        gains_array[9:12] = self.rot_d
    if self.task_controller:
      gains.gains = gains_array
    else:
      gains.data = gains_array
    self.set_gains_pub.publish(gains)



##load the params for the Jtranspose Cartesian controllers (simple_Jtranspose_controller) onto the param server
class JTCartesianParams:

  def __init__(self, whicharm): #whicharm is 'r' or 'l'
    self.whicharm = whicharm
    self.max_vel_trans=.05 
    self.max_vel_rot=.1
    self.max_acc_trans=0.2
    self.max_acc_rot=0.3
    self.pose_fb_trans_p=400.
    self.pose_fb_trans_d=6.
    self.pose_fb_rot_p=40.
    self.pose_fb_rot_d=0.

    self.tcname = self.whicharm+'_arm_cartesian_trajectory_controller'
    self.pcname = self.whicharm+'_arm_cartesian_pose_controller'
    self.tip_frame = self.whicharm+"_wrist_roll_link"
    self.base_frame = "base_link"

    #set the default values on the parameter server
    self.set_params()


  ##set the parameter values back to their defaults
  def set_params_to_defaults(self, set_tip_frame = 1, set_params_on_server = 1):
    self.max_vel_trans=.15 
    self.max_vel_rot=.3
    self.max_acc_trans=0.4
    self.max_acc_rot=0.6
    self.pose_fb_trans_p=2000.
    self.pose_fb_trans_d=30.
    self.pose_fb_rot_p=200.
    self.pose_fb_rot_d=0.
    if set_tip_frame:
      self.tip_frame = self.whicharm+"_wrist_roll_link"
    if set_params_on_server:
      self.set_params()


  ##set the gains and max vels and accs lower, so the arm moves more gently/slowly
  def set_params_to_gentle(self, set_tip_frame = 1, set_params_on_server = 1):
    self.max_vel_trans=.05 
    self.max_vel_rot=.1
    self.max_acc_trans=0.2
    self.max_acc_rot=0.3
    self.pose_fb_trans_p=400.
    self.pose_fb_trans_d=6.
    self.pose_fb_rot_p=40.
    self.pose_fb_rot_d=0.
    if set_tip_frame:
      self.tip_frame = self.whicharm+'_wrist_roll_link'
      rospy.loginfo("setting tip frame back to wrist_roll_link")
    if set_params_on_server:
      self.set_params()


  ##set the current parameter values on the parameter server
  def set_params(self):
    rospy.loginfo("setting Cartesian controller parameters")
    rospy.set_param(self.tcname+'/type', "pr2_manipulation_controllers/CartesianTrajectoryController")
    rospy.set_param(self.tcname+'/root_name', self.base_frame)
    rospy.set_param(self.tcname+'/tip_name', self.tip_frame)
    rospy.set_param(self.tcname+'/output', self.pcname)

    rospy.set_param(self.pcname+'/type', "robot_mechanism_controllers/CartesianPoseController")
    rospy.set_param(self.pcname+'/root_name', self.base_frame)
    rospy.set_param(self.pcname+'/tip_name', self.tip_frame)
    rospy.set_param(self.pcname+'/output', self.whicharm+'_arm_cartesian_twist_controller')

    rospy.set_param(self.tcname+'/max_vel_trans', self.max_vel_trans)
    rospy.set_param(self.tcname+'/max_vel_rot', self.max_vel_rot)
    rospy.set_param(self.tcname+'/max_acc_trans', self.max_acc_trans)
    rospy.set_param(self.tcname+'/max_acc_rot', self.max_acc_rot)
    
    rospy.set_param(self.pcname+'/fb_trans/p', self.pose_fb_trans_p)
    rospy.set_param(self.pcname+'/fb_trans_d', self.pose_fb_trans_d)
    rospy.set_param(self.pcname+'/fb_rot/p', self.pose_fb_rot_p)
    rospy.set_param(self.pcname+'/fb_rot/d', self.pose_fb_rot_d)

    #tip_frame = rospy.get_param(self.tcname+'/tip_name')
