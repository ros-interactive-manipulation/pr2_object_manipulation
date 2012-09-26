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

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>
#include <rosrt/rosrt.h>
#include <angles/angles.h>
#include <control_toolbox/pid.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_listener.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Core>

#include <geometry_msgs/TwistStamped.h>
#include <pr2_manipulation_controllers/JinvTeleopControllerState.h>

namespace pr2_manipulation_controllers {

template <int JOINTS>
class JinvTeleopController : public pr2_controller_interface::Controller
{
public:
  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  typedef Eigen::Matrix<double, JOINTS, 1> JointVector;
  typedef Eigen::Matrix<double, 6, 1> CartVector;
  typedef pr2_manipulation_controllers::JinvTeleopControllerState StateMsg;
public:

  JinvTeleopController();

  ~JinvTeleopController();

  bool init(pr2_mechanism_model::RobotState *r, ros::NodeHandle &n);

  void starting();

  void update();

private:

  pr2_mechanism_model::RobotState *robot_;

  enum {POSE, TWIST} mode_;

  ros::NodeHandle node_;
  ros::Subscriber sub_command_;
  ros::Subscriber sub_command_pose_;
  ros::Subscriber sub_twist_;
  ros::Subscriber sub_posture_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> pub_x_;
  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> pub_x_desi_;
  realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> pub_x_err_;
  rosrt::Publisher<StateMsg> pub_state_;
  ros::Publisher pub_transformed_twist_;
  ros::Publisher pub_transformed_rot_vel_;
  tf::TransformListener tf_;

  CartVector Kp_x, Kd_x;
  JointVector Kp_j, Kd_j, pclamp_j;
  JointVector saturation_;
  double jacobian_inverse_damping_;
  double pose_command_filter_;

  ros::Time last_command_time_;
  Eigen::Affine3d x_desi_, x_desi_filtered_;
  CartVector xd_desi;
  double xd_trans_limit_;
  double xd_rot_limit_;
  JointVector q_proxy_;

  double k_posture_;
  bool use_posture_;
  JointVector q_posture_;

  ros::Time last_time_;
  int loop_count_;

  std::string root_name_;
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;
  boost::scoped_ptr<KDL::ChainFkSolverVel> fk_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;

  // All local variables of update.  Brought outside so they can be recorded
  JointVector q, qd;
  Eigen::Affine3d x;
  CartVector xd;
  CartVector x_err;
  CartVector xd_ref;
  JointVector qd_pose;
  JointVector tau;

  /*
  void commandCB(const simple_cartesian_controller::CartesianCommand::ConstPtr &command)
  {
    ROS_ERROR("COMMAND");
    // TODO: TF stuff

    tf::poseMsgToEigen(command->pose, x_desi_);
    tf::twistMsgToEigen(command->twist, xd_desi);
    xd_trans_limit_ = command->max_trans_vel;
    xd_rot_limit_ = command->max_rot_vel;
    mode_ = POSE;
  }
  */

  void commandPoseCB(const geometry_msgs::PoseStamped::ConstPtr &command)
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
    xd_desi.setZero();
  }

  void commandTwistCB(const geometry_msgs::TwistStamped::ConstPtr &command)
  {
    tf::StampedTransform transform;
    try {
      tf_.waitForTransform(root_name_, command->header.frame_id, command->header.stamp,
                           ros::Duration(0.1));
      tf_.lookupTransform(root_name_, command->header.frame_id, command->header.stamp, transform);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("Failed to transform: %s", ex.what());
      return;
    }

    tf::Vector3 vel(command->twist.linear.x, command->twist.linear.y, command->twist.linear.z);
    tf::Vector3 rot_vel(command->twist.angular.x, command->twist.angular.y, command->twist.angular.z);

    //vel = transform.getBasis().inverse() * vel;
    //rot_vel = transform.getBasis().inverse() * rot_vel;
    vel = transform.getBasis() * vel;
    rot_vel = transform.getBasis() * rot_vel;

    xd_desi[0] = vel[0];
    xd_desi[1] = vel[1];
    xd_desi[2] = vel[2];
    xd_desi[3] = rot_vel[0];
    xd_desi[4] = rot_vel[1];
    xd_desi[5] = rot_vel[2];

    //ROS_INFO("Transformed twist: %.2f, %.2f, %.2f,  %.2f, %.2f, %.2f",
    //         xd_desi[0], xd_desi[1], xd_desi[2], xd_desi[3], xd_desi[4], xd_desi[5]);

    xd_trans_limit_ = 0.0;
    xd_rot_limit_ = 0.0;

    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = root_name_;
    tf::twistEigenToMsg(xd_desi, msg.twist);
    pub_transformed_twist_.publish(msg);
    geometry_msgs::Vector3Stamped vm;
    vm.header.stamp = ros::Time::now();
    vm.header.frame_id = root_name_;
    vm.vector.x = rot_vel[0];
    vm.vector.y = rot_vel[1];
    vm.vector.z = rot_vel[2];
    pub_transformed_rot_vel_.publish(vm);

    last_command_time_ = ros::Time::now();
    mode_ = TWIST;
  }

  void commandPostureCB(const std_msgs::Float64MultiArray::ConstPtr &msg)
  {
    if (msg->data.size() == 0) {
      use_posture_ = false;
      ROS_INFO("Posture turned off");
    }
    else if ((int)msg->data.size() != JOINTS) {
      ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
      return;
    }
    else
    {
      use_posture_ = true;
      for (int j = 0; j < JOINTS; ++j)
        q_posture_[j] = msg->data[j];
    }
  }
};


template <int JOINTS>
JinvTeleopController<JOINTS>::JinvTeleopController()
{
  loop_count_ = 0;
}

template <int JOINTS>
JinvTeleopController<JOINTS>::~JinvTeleopController()
{
  sub_command_.shutdown();
  sub_command_pose_.shutdown();
  sub_twist_.shutdown();
  sub_posture_.shutdown();
}

template <int JOINTS>
bool JinvTeleopController<JOINTS>::init(pr2_mechanism_model::RobotState *r, ros::NodeHandle &n)
{
  rosrt::init();
  node_ = n;
  robot_ = r;

  // Initialized the chain
  std::string tip_name;
  if (!node_.getParam("root_name", root_name_))
  {
    ROS_ERROR("No \"root_name\" found on parameter server");
    return false;
  }
  if (!node_.getParam("tip_name", tip_name))
  {
    ROS_ERROR("No \"tip_name\" found on parameter server");
    return false;
  }
  if (!chain_.init(robot_, root_name_, tip_name))
    return false;
  chain_.toKDL(kdl_chain_);

  // TODO: check chain size again JOINTS

  // Creates the KDL solvers
  fk_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

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
  Kp_x << kp_trans, kp_trans, kp_trans,  kp_rot, kp_rot, kp_rot;
  Kd_x << kd_trans, kd_trans, kd_trans,  kd_rot, kd_rot, kd_rot;

  // Gets the joint gains
  for (size_t j = 0; j < JOINTS; ++j)
  {
    const std::string prefix = std::string("joints/" + chain_.getJoint(j)->joint_->name);
    node_.param(prefix + "/p", Kp_j[j], 0.0);
    node_.param(prefix + "/d", Kd_j[j], 0.0);
    node_.param(prefix + "/p_clamp", pclamp_j[j], 0.0);
  }

  node_.param("k_posture", k_posture_, 0.0);
  node_.param("jacobian_inverse_damping", jacobian_inverse_damping_, 0.0);
  node_.param("pose_command_filter", pose_command_filter_, 1.0);

  // Advertises state topics
  pub_x_.init(node_, "state/x", 3);
  pub_x_desi_.init(node_, "state/x_desi", 3);
  pub_x_err_.init(node_, "state/x_error", 3);
  pub_transformed_twist_ = node_.advertise<geometry_msgs::TwistStamped>("state/transformed_twist", 3);
  pub_transformed_rot_vel_ = node_.advertise<geometry_msgs::Vector3Stamped>("state/transformed_rot_vel", 3);

  // Subscribe to desired posture
  //sub_command_ = node_.subscribe("command", 1, &JinvTeleopController<JOINTS>::commandCB, this);
  sub_command_pose_ = node_.subscribe("command_pose", 20, &JinvTeleopController<JOINTS>::commandPoseCB, this);
  sub_twist_ = node_.subscribe("command_twist", 1,
                               &JinvTeleopController<JOINTS>::commandTwistCB, this);
  sub_posture_ = node_.subscribe("command_posture", 3,
                                 &JinvTeleopController<JOINTS>::commandPostureCB, this);

  StateMsg state_template;
  state_template.header.frame_id = root_name_;
  state_template.x.header.frame_id = root_name_;
  state_template.x_desi.header.frame_id = root_name_;
  state_template.x_desi_filtered.header.frame_id = root_name_;
  state_template.q_proxy.resize(JOINTS);
  state_template.qd_pose.resize(JOINTS);
  state_template.qd_posture.resize(JOINTS);
  state_template.qd_posture_raw.resize(JOINTS);
  state_template.qd_desi.resize(JOINTS);
  state_template.tau.resize(JOINTS);
  state_template.J.layout.dim.resize(2);
  state_template.J.data.resize(6*JOINTS);
  state_template.N.layout.dim.resize(2);
  state_template.N.data.resize(JOINTS*JOINTS);
  pub_state_.initialize(node_.advertise<StateMsg>("state", 20), 20, state_template);

  /*
  JointVector q, qd;
  Eigen::Affine3d x;
  Eigen::Matrix<double, 6, 1> xd;
  Eigen::Matrix<double, 6, 1> x_err;
  Eigen::Matrix<double, 6, 1> xd_ref;
  JointVector qd_pose;
  JointVector tau;
  */
  return true;
}


static void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
  err.head<3>() = xact.translation() - xdes.translation();
  err.tail<3>()   = 0.5 * (xdes.linear().col(0).cross(xact.linear().col(0)) +
                          xdes.linear().col(1).cross(xact.linear().col(1)) +
                          xdes.linear().col(2).cross(xact.linear().col(2)));
}

template <int JOINTS>
void JinvTeleopController<JOINTS>::starting()
{
  last_time_ = robot_->getTime();

  KDL::JntArrayVel q_qd(JOINTS);
  chain_.getVelocities(q_qd);
  JointVector q(q_qd.q.data), qd(q_qd.qdot.data);

  // Forward kinematics to get current x
  Eigen::Affine3d x;
  KDL::FrameVel x_framevel;
  fk_solver_->JntToCart(q_qd, x_framevel);
  tf::transformKDLToEigen(x_framevel.GetFrame(), x);
  q_proxy_ = q;

  x_desi_ = x;
  x_desi_filtered_ = x;
  xd_desi.setZero();
  use_posture_ = false;
  mode_ = POSE;
}

template <class Derived>
//Eigen::Quaterniond eulerRot(const Eigen::MatrixBase<Derived>& e)
Eigen::Matrix<double,3,3> eulerRot(double dt, const Eigen::MatrixBase<Derived>& e)
{
  double n = e.norm();
  if (fabs(n) < 1e-8)
    return Eigen::Matrix<double,3,3>::Identity();
  return Eigen::AngleAxis<double>(n * dt, e.normalized()).toRotationMatrix();
  /*
  return
    Eigen::AngleAxis<double>(e[2], Eigen::Vector3d(0,0,1)) *
    Eigen::AngleAxis<double>(e[1], Eigen::Vector3d(0,1,0)) *
    Eigen::AngleAxis<double>(e[0], Eigen::Vector3d(1,0,0));
  */
}

template <int JOINTS>
void JinvTeleopController<JOINTS>::update()
{
  ros::Time time = robot_->getTime();
  ros::Duration dt = time - last_time_;

  KDL::JntArrayVel q_qd(JOINTS);
  chain_.getVelocities(q_qd);
  q = q_qd.q.data;
  qd = q_qd.qdot.data;

  KDL::Jacobian kdl_jacobian(JOINTS);
  jac_solver_->JntToJac(q_qd.q, kdl_jacobian);
  Eigen::Matrix<double, 6, JOINTS> J(kdl_jacobian.data);

  // Forward kinematics to get x and xd
  KDL::FrameVel x_framevel;
  fk_solver_->JntToCart(q_qd, x_framevel);
  tf::transformKDLToEigen(x_framevel.GetFrame(), x);
  tf::twistKDLToEigen(x_framevel.GetTwist(), xd);

  // Integrates the desired twist to determine the next position.
  if (mode_ == TWIST)
  {
    // Only apply update if command is less than 2 seconds old,
    // otherwise timeout and switch back to POSE mode.
    // TODO parameterize the timeout duration
    if (time < last_command_time_ + ros::Duration(1.0))
    {
      x_desi_.translation() += xd_desi.head<3>() * dt.toSec();
      x_desi_.linear() = eulerRot(dt.toSec(), xd_desi.tail<3>()) * x_desi_.linear();

      // Clamp
      Eigen::Vector3d err = x_desi_.translation() - x.translation();
      if (err.norm() > 0.1)
      {
        x_desi_.translation() = x.translation() + err.normalized() * 0.1;
      }
    }
    else
    {
      x_desi_ = x;
      mode_ = POSE;
    }
    x_desi_filtered_ = x_desi_;
  }
  else
  {
    // Filters the desired pose

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

  //computePoseError(x, x_desi_, x_err);
  computePoseError(x, x_desi_filtered_, x_err);

  // Computes the reference twist from the pose error and the desired twist.
  xd_ref = xd_desi.array() - (Kp_x.array() / Kd_x.array()) * x_err.array();

  // Applies velocity limits
  // TODO

  // ======== J psuedo-inverse and Nullspace computation

  // Computes pseudo-inverse of J

  Eigen::Matrix<double,6,6> I6; I6.setIdentity();
  Eigen::Matrix<double,6,6> JJt = J * J.transpose();
  Eigen::Matrix<double,6,6> JJt_inv;
  JJt_inv = JJt.inverse();
  Eigen::Matrix<double,JOINTS,6> J_pinv = J.transpose() * JJt_inv;
  Eigen::Matrix<double,6,6> JJt_damped = J * J.transpose() + jacobian_inverse_damping_ * I6;
  Eigen::Matrix<double,6,6> JJt_inv_damped;
  JJt_inv_damped = JJt_damped.inverse();
  Eigen::Matrix<double,JOINTS,6> J_pinv_damped = J.transpose() * JJt_inv_damped;

  // Computes the nullspace of J
  Eigen::Matrix<double,JOINTS,JOINTS> I;
  I.setIdentity();
  Eigen::Matrix<double,JOINTS,JOINTS> N = I - J_pinv * J;

  // ======== Pose Control
  
  // Computes the desired joint velocities for achieving the pose.
  qd_pose = J_pinv_damped * xd_ref;  // J-inverse
  //qd_pose = J.transpose() * xd_ref;  // J-transpose
  JointVector qd_desi = qd_pose;


  // ======== Posture control
  
  // Computes the desired joint velocities for achieving the posture
  JointVector qd_posture;
  JointVector qd_posture_raw;
  if (use_posture_)
  {
    JointVector posture_err = q_posture_ - q;
    for (size_t j = 0; j < JOINTS; ++j)
    {
      if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
        posture_err[j] = angles::normalize_angle(posture_err[j]);
    }

    for (size_t j = 0; j < JOINTS; ++j) {
      if (fabs(q_posture_[j] - 9999) < 1e-5)
        posture_err[j] = 0.0;
    }

    qd_posture_raw = k_posture_ * posture_err;
    qd_posture = N * qd_posture_raw;
    qd_desi += qd_posture;
  }

  // Clamps the proxy
  // aleeper:
  // Doesn't do anything because q_proxy_ and q should never differ
  for (int j = 0; j < JOINTS; ++j)
    q_proxy_[j] = std::max(q[j] - pclamp_j[j], std::min(q_proxy_[j], q[j] + pclamp_j[j]));

  // Convertes the desired joint velocities into joint torques
  tau = Kp_j.array() * (q_proxy_ - q).array() + Kd_j.array() * (qd_desi - qd).array();
  // aleeper: changed to qd_desi from qd... is this right?
  q_proxy_ += qd_desi * dt.toSec();

  // ======== Torque Saturation
  // Copied from JT controller ... do we want this here, or does it even make sense?
  /*
  double sat_scaling = 1.0;
  for (int i = 0; i < Joints; ++i) {
    if (saturation_[i] > 0.0)
      sat_scaling = std::min(sat_scaling, fabs(saturation_[i] / tau[i]));
  }
  JointVec tau_sat = sat_scaling * tau;
  */

  // ======== Apply torques to arm
  chain_.addEfforts(tau);


  // ================ END OF CONTROLLER ================
   
  // ======== Publish state
  if (loop_count_ % 10 == 0)
  {
    if (pub_x_.trylock())
    {
      pub_x_.msg_.header.stamp = time;
      pub_x_.msg_.header.frame_id = root_name_;  // NOT RT SAFE
      tf::poseEigenToMsg(x, pub_x_.msg_.pose);
      pub_x_.unlockAndPublish();
    }

    if (pub_x_desi_.trylock())
    {
      pub_x_desi_.msg_.header.stamp = time;
      pub_x_desi_.msg_.header.frame_id = root_name_;  // NOT RT SAFE
      tf::poseEigenToMsg(x_desi_, pub_x_desi_.msg_.pose);
      pub_x_desi_.unlockAndPublish();
    }
    if (pub_x_err_.trylock())
    {
      pub_x_err_.msg_.header.stamp = time;
      tf::twistEigenToMsg(x_err, pub_x_err_.msg_.twist);
      pub_x_err_.unlockAndPublish();
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
      tf::twistEigenToMsg(xd, state_msg->xd);
      tf::twistEigenToMsg(xd_desi, state_msg->xd_desi);
      tf::matrixEigenToMsg(J, state_msg->J);
      tf::matrixEigenToMsg(N, state_msg->N);
      for (size_t j = 0; j < JOINTS; ++j) {
        state_msg->q_proxy[j] = q_proxy_[j];
        state_msg->qd_pose[j] = qd_pose[j];
        state_msg->qd_posture[j] = qd_posture[j];
        state_msg->qd_posture_raw[j] = qd_posture_raw[j];
        state_msg->qd_desi[j] = qd_desi[j];
        state_msg->tau[j] = tau[j];
      }
      pub_state_.publish(state_msg);
    }
  }

  last_time_ = time;
  ++loop_count_;
}

} // namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(pr2_manipulation_controllers, JinvTeleopController7,
                        pr2_manipulation_controllers::JinvTeleopController<7>, pr2_controller_interface::Controller)
