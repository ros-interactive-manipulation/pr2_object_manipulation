/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <pr2_manipulation_controllers/JTTaskControllerState.h>
#include <object_manipulation_msgs/CartesianGains.h>

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <control_toolbox/pid.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_listener.h>

#include <rosrt/rosrt.h>

namespace pr2_manipulation_controllers {

template <int Joints>
struct Kin
{
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;

  Kin(const KDL::Chain &kdl_chain) :
    fk_solver_(kdl_chain), jac_solver_(kdl_chain),
    kdl_q(Joints), kdl_J(Joints)
  {
  }
  ~Kin()
  {
  }

  void fk(const JointVec &q, Eigen::Affine3d &x)
  {
    kdl_q.data = q;
    KDL::Frame kdl_x;
    fk_solver_.JntToCart(kdl_q, kdl_x);
    tf::transformKDLToEigen(kdl_x, x);
  }
  void jac(const JointVec &q, Jacobian &J)
  {
    kdl_q.data = q;
    jac_solver_.JntToJac(kdl_q, kdl_J);
    J = kdl_J.data;
  }

  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainJntToJacSolver jac_solver_;
  KDL::JntArray kdl_q;
  KDL::Jacobian kdl_J;
};

class JTTaskController : public pr2_controller_interface::Controller
{
public:
  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
  enum { Joints = 7 };
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, 1> CartVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;
  typedef pr2_manipulation_controllers::JTTaskControllerState StateMsg;
public:
  JTTaskController();
  ~JTTaskController();

  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();

  Eigen::Affine3d x_desi_, x_desi_filtered_;
  CartVec wrench_desi_;

  ros::NodeHandle node_;
  ros::Subscriber sub_gains_;
  ros::Subscriber sub_posture_;
  ros::Subscriber sub_pose_;
  tf::TransformListener tf_;

  rosrt::Publisher<StateMsg> pub_state_;
  rosrt::Publisher<geometry_msgs::PoseStamped> pub_x_, pub_x_desi_;
  rosrt::Publisher<geometry_msgs::Twist> pub_xd_, pub_xd_desi_;
  rosrt::Publisher<geometry_msgs::Twist> pub_x_err_, pub_wrench_;
  rosrt::Publisher<std_msgs::Float64MultiArray> pub_tau_;

  std::string root_name_, tip_name_;
  ros::Time last_time_;
  int loop_count_;
  pr2_mechanism_model::RobotState *robot_state_;

  pr2_mechanism_model::Chain chain_;
  boost::scoped_ptr<Kin<Joints> > kin_;
  
  Eigen::Matrix<double,6,1> Kp, Kd;  //aleeper
  
  // Todo Rename this! The rotation matrix for tool->gain frame
  Eigen::Matrix<double,6,6> St;  //aleeper
  bool use_tip_frame_; // aleeper
  double pose_command_filter_;
  double vel_saturation_trans_, vel_saturation_rot_;
  JointVec saturation_;
  JointVec joint_dd_ff_;
  double joint_vel_filter_;
  double jacobian_inverse_damping_;
  JointVec q_posture_;
  double k_posture_;
  bool use_posture_;

  // Minimum resolutions
  double res_force_, res_position_;
  double res_torque_, res_orientation_;

  Eigen::Affine3d last_pose_;
  CartVec last_wrench_;
  double last_stiffness_, last_compliance_;
  double last_Dx_, last_Df_;


  JointVec qdot_filtered_;

/*  void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() >= 6)
      for (size_t i = 0; i < 6; ++i)
        Kp[i] = msg->data[i];
    if (msg->data.size() == 12)
      for (size_t i = 0; i < 6; ++i)
        Kd[i] = msg->data[6+i];

    ROS_INFO("New gains: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
             Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
  }
*/
  void setGains(const object_manipulation_msgs::CartesianGains::ConstPtr &msg)
  {

    //ROS_INFO_STREAM("Received CartesianGains msg: " << *msg);
    //ROS_INFO("root: [%s] tip: [%s]", root_name_.c_str(), tip_name_.c_str()); 
    
    // Store gains...
    if (msg->gains.size() >= 6)
      for (size_t i = 0; i < 6; ++i)
        Kp[i] = msg->gains[i];
    if (msg->gains.size() == 12)
      for (size_t i = 0; i < 6; ++i)
        Kd[i] = msg->gains[6+i];

    // Store frame information
    if(!msg->header.frame_id.compare(root_name_))
    {
      use_tip_frame_ = false;
      ROS_INFO("New gains in root frame [%s]: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
               root_name_.c_str(), Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
      St.setIdentity();
    }
    else if(!msg->header.frame_id.compare(tip_name_))
    {
      use_tip_frame_ = true;
      ROS_INFO("New gains in tip frame [%s]: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
               tip_name_.c_str(), Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
      
    }
    else
    {
      use_tip_frame_ = false;
      
      geometry_msgs::PoseStamped in_root;
      in_root.pose.orientation.w = 1.0;
      in_root.header.frame_id = msg->header.frame_id;

      try {
        tf_.waitForTransform(root_name_, msg->header.frame_id, msg->header.stamp, ros::Duration(0.1));
        tf_.transformPose(root_name_, in_root, in_root);
      }
      catch (const tf::TransformException &ex)
      {
        ROS_ERROR("Failed to transform: %s", ex.what());
        return;
      }
      
      Eigen::Affine3d t;
      
      tf::poseMsgToEigen(in_root.pose, t);

      St << 
          t(0,0),t(0,1),t(0,2),0,0,0,
          t(1,0),t(1,1),t(1,2),0,0,0,
          t(2,0),t(2,1),t(2,2),0,0,0,
          0,0,0,t(0,0),t(0,1),t(0,2),
          0,0,0,t(1,0),t(1,1),t(1,2),
          0,0,0,t(2,0),t(2,1),t(2,2);
    
      St.transposeInPlace();
  
      ROS_INFO("New gains in arbitrary frame [%s]: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
             msg->header.frame_id.c_str(), Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
    }
  }

  void commandPosture(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() == 0) {
      use_posture_ = false;
      ROS_INFO("Posture turned off");
    }
    else if ((int)msg->data.size() != q_posture_.size()) {
      ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
      return;
    }
    else
    {
      use_posture_ = true;
      for (int j = 0; j < Joints; ++j)
        q_posture_[j] = msg->data[j];
    }
  }

  void commandPose(const geometry_msgs::PoseStamped::ConstPtr &command)
  {
    geometry_msgs::PoseStamped in_root;
    try {
      tf_.waitForTransform(root_name_, command->header.frame_id, command->header.stamp, ros::Duration(0.1));
      tf_.transformPose(root_name_, *command, in_root);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("Failed to transform: %s", ex.what());
      return;
    }

    tf::poseMsgToEigen(in_root.pose, x_desi_);
  }
};


JTTaskController::JTTaskController()
  : robot_state_(NULL), use_posture_(false)
{}

JTTaskController::~JTTaskController()
{
  sub_gains_.shutdown();
  sub_posture_.shutdown();
  sub_pose_.shutdown();
}


bool JTTaskController::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &n)
{
  rosrt::init();
  node_ = n;

  ROS_INFO_STREAM("JTTask controller compiled at " << __TIME__ );
  // get name of root and tip from the parameter server
  // std::string tip_name; // aleeper: Should use the class member instead!
  if (!node_.getParam("root_name", root_name_)){
    ROS_ERROR("JTTaskController: No root name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tip_name", tip_name_)){
    ROS_ERROR("JTTaskController: No tip name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  // Chain of joints
  if (!chain_.init(robot_state_, root_name_, tip_name_))
    return false;
  if (!chain_.allCalibrated())
  {
    ROS_ERROR("Not all joints in the chain are calibrated (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }


  // Kinematics
  KDL::Chain kdl_chain;
  chain_.toKDL(kdl_chain);
  kin_.reset(new Kin<Joints>(kdl_chain));

  // Cartesian gains
  double kp_trans, kd_trans, kp_rot, kd_rot;
  if (!node_.getParam("cart_gains/trans/p", kp_trans) ||
      !node_.getParam("cart_gains/trans/d", kd_trans))
  {
    ROS_ERROR("P and D translational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("cart_gains/rot/p", kp_rot) ||
      !node_.getParam("cart_gains/rot/d", kd_rot))
  {
    ROS_ERROR("P and D rotational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  Kp << kp_trans, kp_trans, kp_trans,  kp_rot, kp_rot, kp_rot;
  Kd << kd_trans, kd_trans, kd_trans,  kd_rot, kd_rot, kd_rot;

  // aleeper
  use_tip_frame_ = false;
  if (!node_.getParam("use_tip_frame", use_tip_frame_)){
    ROS_WARN("JTTaskController: use_tip_frame was not specified, assuming 'false': %s)",
              node_.getNamespace().c_str());
  }
  St.setIdentity();

  node_.param("pose_command_filter", pose_command_filter_, 1.0);

  // Velocity saturation
  node_.param("vel_saturation_trans", vel_saturation_trans_, 0.0);
  node_.param("vel_saturation_rot", vel_saturation_rot_, 0.0);

  node_.param("jacobian_inverse_damping", jacobian_inverse_damping_, 0.0);
  node_.param("joint_vel_filter", joint_vel_filter_, 1.0);

  // Joint gains
  for (int i = 0; i < Joints; ++i)
    node_.param("joint_feedforward/" + chain_.getJoint(i)->joint_->name, joint_dd_ff_[i], 0.0);
  for (int i = 0; i < Joints; ++i)
    node_.param("joint_max_effort/" + chain_.getJoint(i)->joint_->name, saturation_[i], 0.0);

  // Posture gains
  node_.param("k_posture", k_posture_, 1.0);

  node_.param("resolution/force", res_force_, 0.01);
  node_.param("resolution/position", res_position_, 0.001);
  node_.param("resolution/torque", res_torque_, 0.01);
  node_.param("resolution/orientation", res_orientation_, 0.001);


  sub_gains_ = node_.subscribe("gains", 5, &JTTaskController::setGains, this);
  sub_posture_ = node_.subscribe("command_posture", 5, &JTTaskController::commandPosture, this);
  sub_pose_ = node_.subscribe("command_pose", 1, &JTTaskController::commandPose, this);

  StateMsg state_template;
  state_template.header.frame_id = root_name_;
  state_template.x.header.frame_id = root_name_;
  state_template.x_desi.header.frame_id = root_name_;
  state_template.x_desi_filtered.header.frame_id = root_name_;
  state_template.tau_pose.resize(Joints);
  state_template.tau_posture.resize(Joints);
  state_template.tau.resize(Joints);
  state_template.J.layout.dim.resize(2);
  state_template.J.data.resize(6*Joints);
  state_template.N.layout.dim.resize(2);
  state_template.N.data.resize(Joints*Joints);
  pub_state_.initialize(node_.advertise<StateMsg>("state", 10), 10, state_template);

  geometry_msgs::PoseStamped pose_template;
  pose_template.header.frame_id = root_name_;
  pub_x_.initialize(node_.advertise<geometry_msgs::PoseStamped>("state/x", 10),
                    10, pose_template);
  pub_x_desi_.initialize(node_.advertise<geometry_msgs::PoseStamped>("state/x_desi", 10),
                         10, pose_template);
  pub_x_err_.initialize(node_.advertise<geometry_msgs::Twist>("state/x_err", 10),
                        10, geometry_msgs::Twist());
  pub_xd_.initialize(node_.advertise<geometry_msgs::Twist>("state/xd", 10),
                     10, geometry_msgs::Twist());
  pub_xd_desi_.initialize(node_.advertise<geometry_msgs::Twist>("state/xd_desi", 10),
                          10, geometry_msgs::Twist());
  pub_wrench_.initialize(node_.advertise<geometry_msgs::Twist>("state/wrench", 10),
                         10, geometry_msgs::Twist());

  std_msgs::Float64MultiArray joints_template;
  joints_template.layout.dim.resize(1);
  joints_template.layout.dim[0].size = Joints;
  joints_template.layout.dim[0].stride = 1;
  joints_template.data.resize(7);
  pub_tau_.initialize(node_.advertise<std_msgs::Float64MultiArray>("state/tau", 10),
                      10, joints_template);

  return true;
}

void JTTaskController::starting()
{
  //Kp << 800.0, 800.0, 800.0,   80.0, 80.0, 80.0;
  //Kd << 12.0, 12.0, 12.0,   0.0, 0.0, 0.0;

  JointVec q;
  chain_.getPositions(q);
  kin_->fk(q, x_desi_);
  x_desi_filtered_ = x_desi_;
  last_pose_ = x_desi_;
  q_posture_ = q;
  qdot_filtered_.setZero();
  last_wrench_.setZero();

  last_stiffness_ = 0;
  last_compliance_ = 0;
  last_Dx_ = 0;
  last_Df_ = 0;

  loop_count_ = 0;
}


static void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
  err.head<3>() = xact.translation() - xdes.translation();
  err.tail<3>()   = 0.5 * (xdes.linear().col(0).cross(xact.linear().col(0)) +
                          xdes.linear().col(1).cross(xact.linear().col(1)) +
                          xdes.linear().col(2).cross(xact.linear().col(2)));
}

void JTTaskController::update()
{
  // get time
  ros::Time time = robot_state_->getTime();
  ros::Duration dt = time - last_time_;
  last_time_ = time;
  ++loop_count_;

  // ======== Measures current arm state

  JointVec q;
  chain_.getPositions(q);

  Eigen::Affine3d x;
  kin_->fk(q, x);

  Jacobian J;
  kin_->jac(q, J);


  JointVec qdot_raw;
  chain_.getVelocities(qdot_raw);
  for (int i = 0; i < Joints; ++i)
    qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw[i] - qdot_filtered_[i]);
  JointVec qdot = qdot_filtered_;
  CartVec xdot = J * qdot;

  // ======== Controls to the current pose setpoint

  // Compute x_desi_filtered_ based on pose_command_filter time constant
  {
    Eigen::Vector3d p0(x_desi_filtered_.translation());
    Eigen::Vector3d p1(x_desi_.translation());
    Eigen::Quaterniond q0(x_desi_filtered_.linear());
    Eigen::Quaterniond q1(x_desi_.linear());
    q0.normalize();
    q1.normalize();

    tf::Quaternion tf_q0(q0.x(), q0.y(), q0.z(), q0.w());
    tf::Quaternion tf_q1(q1.x(), q1.y(), q1.z(), q1.w());
    tf::Quaternion tf_q = tf_q0.slerp(tf_q1, pose_command_filter_);

    Eigen::Vector3d p = p0 + pose_command_filter_ * (p1 - p0);
    //Eigen::Quaterniond q = q0.slerp(pose_command_filter_, q1);
    Eigen::Quaterniond q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
    //x_desi_filtered_ = q * Eigen::Translation3d(p);
    x_desi_filtered_ = Eigen::Translation3d(p) * q;
  }
  CartVec x_err;
  //computePoseError(x, x_desi_, x_err);
  computePoseError(x, x_desi_filtered_, x_err);

  // If updating every cycle, set matrix based on end-effector pose (transform)
  // Use rotation matrix components of x (Affine3d) to rotate linear and rotational
  // components of wrench
  if(use_tip_frame_)
  { 
      St << 
          x(0,0),x(0,1),x(0,2),0,0,0,
          x(1,0),x(1,1),x(1,2),0,0,0,
          x(2,0),x(2,1),x(2,2),0,0,0,
          0,0,0,x(0,0),x(0,1),x(0,2),
          0,0,0,x(1,0),x(1,1),x(1,2),
          0,0,0,x(2,0),x(2,1),x(2,2);
      St.transposeInPlace();
  }

  // HERE WE CONVERT CALCULATIONS TO THE FRAME IN WHICH GAINS ARE SPECIFIED!
  //xdot_desi is not derivative of x_desi_filtered_--it is a velocity controller on the position error
  //F = -k_d*xdot - k_p*(x - x_desi)
  //xdot_desi = -k_p/k_d(x - x_desi)

  // This can/should be refactored to look like R' * K * R.
  CartVec xdot_desi = -1.0 * (Kp.array() / Kd.array()) * (St * x_err).array() ;  // aleeper


  // Caps the cartesian velocity
  // We can consider capping force/torque, but there are multiple places to
  // cap and proxy things.
  if (vel_saturation_trans_ > 0.0) // negative value disables cap
  {
    if (fabs(xdot_desi.head<3>().norm()) > vel_saturation_trans_)
      xdot_desi.head<3>() *= (vel_saturation_trans_ / xdot_desi.head<3>().norm());
  }
  if (vel_saturation_rot_ > 0.0) // negative value disables cap
  {
    if (fabs(xdot_desi.tail<3>().norm()) > vel_saturation_rot_)
      xdot_desi.tail<3>() *= (vel_saturation_rot_ / xdot_desi.tail<3>().norm());
  }

  CartVec F = Kd.array() * (xdot_desi.array() - (St * xdot).array());  // aleeper

  // HERE WE CONVERT BACK TO THE ROOT FRAME SINCE THE JACOBIAN IS IN ROOT FRAME.
  JointVec tau_pose = J.transpose() * St.transpose() * F;

  // ======== J psuedo-inverse and Nullspace computation

  // Computes pseudo-inverse of J
  Eigen::Matrix<double,6,6> I6; I6.setIdentity();
  Eigen::Matrix<double,6,6> JJt = J * J.transpose();
  Eigen::Matrix<double,6,6> JJt_inv;
  JJt_inv = JJt.inverse();
  Eigen::Matrix<double,6,6> JJt_damped = J * J.transpose() + jacobian_inverse_damping_ * I6;
  Eigen::Matrix<double,6,6> JJt_inv_damped;
  JJt_inv_damped = JJt_damped.inverse();
  Eigen::Matrix<double,Joints,6> J_pinv = J.transpose() * JJt_inv_damped;

  // Computes the nullspace of J
  Eigen::Matrix<double,Joints,Joints> I;
  I.setIdentity();
  Eigen::Matrix<double,Joints,Joints> N = I - J_pinv * J;

  // ======== Posture control

  // Computes the desired joint torques for achieving the posture
  JointVec tau_posture;
  tau_posture.setZero();
  if (use_posture_)
  {
    JointVec posture_err = q_posture_ - q;
    for (size_t j = 0; j < Joints; ++j)
    {
      if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
        posture_err[j] = angles::normalize_angle(posture_err[j]);
    }

    for (size_t j = 0; j < Joints; ++j) {
      if (fabs(q_posture_[j] - 9999) < 1e-5)
        posture_err[j] = 0.0;
    }

    JointVec qdd_posture = k_posture_ * posture_err;
    tau_posture = joint_dd_ff_.array() * (N * qdd_posture).array();
  }

  JointVec tau = tau_pose + tau_posture;

  // ======== Torque Saturation
  double sat_scaling = 1.0;
  for (int i = 0; i < Joints; ++i) {
    if (saturation_[i] > 0.0)
      sat_scaling = std::min(sat_scaling, fabs(saturation_[i] / tau[i]));
  }
  JointVec tau_sat = sat_scaling * tau;

  // ======== Apply torques to arm
  chain_.addEfforts(tau_sat);

  // ================ END OF CONTROLLER ================

  // ======== Environment stiffness

  CartVec df = F - last_wrench_;
  CartVec dx;
  computePoseError(last_pose_, x, dx);

  // Just in the Z direction for now

  double Df, Dx;
  if (fabs(dx[2]) >= res_position_)
    Df = df[2] * res_position_ / fabs(dx[2]);
  else
    Df = (1. - fabs(dx[2])/res_position_) * last_Df_ + df[2];
  if (fabs(df[2]) >= res_force_)
    Dx = dx[2] * res_force_ / fabs(df[2]);
  else
    Dx = (1. - fabs(df[2])/res_force_) * last_Dx_ + dx[2];
  last_Df_ = Df;
  last_Dx_ = Dx;

  double stiffness, compliance;
  if (fabs(dx[2]) >= res_position_)
    stiffness = fabs(df[2]) / fabs(dx[2]);
  else
    stiffness = (1 - fabs(dx[2])/res_position_) * last_stiffness_ + fabs(df[2]) / res_position_;
  if (fabs(df[2]) >= res_force_)
    compliance = fabs(dx[2]) / fabs(df[2]);
  else
    compliance = (1 - fabs(df[2])/res_force_) * last_compliance_ + fabs(dx[2]) / res_force_;

  last_pose_ = x;
  last_wrench_ = F;
  last_stiffness_ = stiffness;
  last_compliance_ = compliance;

  if (loop_count_ % 10 == 0)
  {
    geometry_msgs::PoseStamped::Ptr pose_msg;
    geometry_msgs::Twist::Ptr twist_msg;
    std_msgs::Float64MultiArray::Ptr q_msg;

    if (pose_msg = pub_x_.allocate()) {  // X
      pose_msg->header.stamp = time;
      tf::poseEigenToMsg(x, pose_msg->pose);
      pub_x_.publish(pose_msg);
    }

    if (pose_msg = pub_x_desi_.allocate()) {  // X desi
      pose_msg->header.stamp = time;
      tf::poseEigenToMsg(x_desi_, pose_msg->pose);
      pub_x_desi_.publish(pose_msg);
    }

    if (twist_msg = pub_x_err_.allocate()) {  // X err
      tf::twistEigenToMsg(x_err, *twist_msg);
      pub_x_err_.publish(twist_msg);
    }

    if (twist_msg = pub_xd_.allocate()) {  // Xdot
      tf::twistEigenToMsg(xdot, *twist_msg);
      pub_xd_.publish(twist_msg);
    }

    if (twist_msg = pub_xd_desi_.allocate()) {  // Xdot desi
      tf::twistEigenToMsg(xdot_desi, *twist_msg);
      pub_xd_desi_.publish(twist_msg);
    }

    if (twist_msg = pub_wrench_.allocate()) {  // F
      tf::twistEigenToMsg(F, *twist_msg);
      pub_wrench_.publish(twist_msg);
    }

    if (q_msg = pub_tau_.allocate()) {  // tau
      for (size_t i = 0; i < Joints; ++i)
        q_msg->data[i] = tau[i];
      pub_tau_.publish(q_msg);
    }

    StateMsg::Ptr state_msg;
    if (state_msg = pub_state_.allocate()) {
      state_msg->header.stamp = time;
      state_msg->x.header.stamp = time;
      tf::poseEigenToMsg(x, state_msg->x.pose);
      state_msg->x_desi.header.stamp = time;
      tf::poseEigenToMsg(x_desi_, state_msg->x_desi.pose);
      state_msg->x_desi_filtered.header.stamp = time;
      tf::poseEigenToMsg(x_desi_filtered_, state_msg->x_desi_filtered.pose);
      tf::twistEigenToMsg(x_err, state_msg->x_err);
      tf::twistEigenToMsg(xdot, state_msg->xd);
      tf::twistEigenToMsg(xdot_desi, state_msg->xd_desi);
      tf::wrenchEigenToMsg(F, state_msg->F);
      tf::matrixEigenToMsg(J, state_msg->J);
      tf::matrixEigenToMsg(N, state_msg->N);
      for (size_t j = 0; j < Joints; ++j) {
        state_msg->tau_pose[j] = tau_pose[j];
        state_msg->tau_posture[j] = tau_posture[j];
        state_msg->tau[j] = tau[j];
      }
      state_msg->stiffness = stiffness;
      state_msg->compliance = compliance;
      state_msg->Df = Df / res_position_;
      state_msg->Dx = Dx / res_force_;
      state_msg->df = df[2];
      state_msg->dx = dx[2];
      pub_state_.publish(state_msg);
    }
  }
}

} //namespace

PLUGINLIB_EXPORT_CLASS(pr2_manipulation_controllers::JTTaskController,pr2_controller_interface::Controller)

