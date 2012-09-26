/*********************************************************************
 *
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
 *
 *  \author Joe Romano
 *********************************************************************/
//@author  Joe Romano
//@email   joeromano@gmail.com
//@brief   pr2_gripper_sensor_controller.cpp  - controller to read data from
//         the sensors on the pr2 gripper (accelerometer in palm,
//         pressure arrays on the fingers, and encoder in the hand) 
//         and publish higher-level useful processed information about
//         them, as well as perform low-level control tasks to control
//         the gripper based on these signals in real-time.
//         
//         This controller is inteded to be interacted with through its
//         action interface, pr2_gripper_sensor_action.

#include <pr2_controller_interface/controller.h>
#include <pr2_gripper_sensor_controller/gripper_controller.h>
#include <pr2_gripper_sensor_controller/pressure_observer.h>
#include <pr2_gripper_sensor_controller/acceleration_observer.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactData.h>
#include <pr2_gripper_sensor_msgs/PR2GripperFindContactCommand.h>
#include <pr2_gripper_sensor_msgs/PR2GripperForceServoData.h>
#include <pr2_gripper_sensor_msgs/PR2GripperForceServoCommand.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoData.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSlipServoCommand.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorData.h>
#include <pr2_gripper_sensor_msgs/PR2GripperEventDetectorCommand.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSensorRTState.h>
#include <pr2_gripper_sensor_msgs/PR2GripperSensorRawData.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <ros/ros.h>

#include <pr2_controllers_msgs/JointControllerState.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/Float64.h>


namespace pr2_gripper_sensor_controller{

class PR2GripperSensorController: public pr2_controller_interface::Controller
{
private:
  ros::NodeHandle nodeHandle;                   ///< our ROS node handle
  pr2_mechanism_model::RobotState *robotHandle; ///< our robot state handle


  // functions ---------------
  bool initializeHandles(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  bool forceServo();
  bool updateZeros(std_srvs::Empty::Request& req,std_srvs::Empty::Response& resp);
  bool stopMotorOutput(std_srvs::Empty::Request& req,std_srvs::Empty::Response& resp);
  void reinitializeValues();
  bool reloadParams(std_srvs::Empty::Request& req,std_srvs::Empty::Response& resp);
  
  // variables ---------------
  int loop_count_;         ///< loop counter
  int contactCounter;      ///< counter to delay a certain amount of tmie after contact
  bool stable_contact;     ///< state variable to indicate whether our contact has been held enough
  bool placedState;        ///< state flag to indicate when event_detector succceded
  bool update_zeros;       ///< bool to signal whether we are currently updating our zeros or not
  int placeConditions;     ///< storage container for the conditions we want to use in event_detector
  double acc_trigger;      ///< value to use as trigger for acceleration detection (event_detector)
  double slip_trigger;     ///< value to use as trigger for slip detection (event_detector)
  bool stable_force;       ///< flag to indicate a stable force is achieved
  int findContact_delay;   ///< counter to track how long a force is stable for

  // state variables about our controller
  int control_mode;        ///< the current/desired control mode we want the controller state machine to be in
  bool contact_success;    ///< flag to indicate whether the fingers found contact or not
  double servo_force;      ///< the force we want to servo our controller to
  double servo_position;   ///< the position we want to servo our controller to
  double max_effort;       ///< the max effort to allow our controller to use
  double servo_velocity;   ///< the velocity we would like our controller to servo to
  int contacts_desired;   ///< the contact state we want when we search for contact on the fingers
  double deformation_limit;///< the max allowable deformation for our controller
  double fingertip_start_force; ///< the force we want to start seroing to on the fingertips
  double fingertip_force_limit; ///< the limit we do not want to exceed when force servoing
  double force_servo_velocity_tolerance; ///< the tolerance on velocity to consider a force stable
  const pr2_gripper_sensor_msgs::PR2GripperSensorRTState rt_state_def; ///< object that defines states of our controller
  pr2_gripper_sensor_msgs::PR2GripperSensorRawData raw_data;///< an object for raw data publication for debugging
  bool publish_raw_data;   ///< flag to indicate whether we want to publish raw data or not

  ros::Time last_time_;    ///< counter to track the time

  accelerationObserver *myAccelerationObserver;   ///< accelerometer observer object
  gripperController *myGripperController;         ///< gripper action object to control our gripper
  pressureObserver *myPressureObserver;           ///< pressure observer object to do analysis on the incoming pressure sensor data

  
  ///our service publishers
  ros::ServiceServer updateZeros_srv_;
  ros::ServiceServer reloadParams_srv_;
  ros::ServiceServer stopMotorOutput_srv_;

  /// blank request/responses
  std_srvs::Empty::Request empty_req;
  std_srvs::Empty::Response empty_resp;


  /// raw data real-time publisher
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
    pr2_gripper_sensor_msgs::PR2GripperSensorRawData> > raw_data_publisher_ ;

  /// position data real-time publisher
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
    pr2_controllers_msgs::JointControllerState> > controller_state_publisher_ ;
  ros::Subscriber sub_command_;
  void positionCB(const pr2_controllers_msgs::Pr2GripperCommandConstPtr& msg);

  /// find_contact real-time publisher
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
    pr2_gripper_sensor_msgs::PR2GripperFindContactData> > contact_state_publisher_ ;
  ros::Subscriber sub_findcontact_command_;
  void findContactCB(const pr2_gripper_sensor_msgs::PR2GripperFindContactCommandConstPtr& msg);

  /// slip_servo real-time publisher
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
    pr2_gripper_sensor_msgs::PR2GripperSlipServoData> > slip_state_publisher_ ;
  ros::Subscriber sub_slipservo_command_;
  void slipServoCB(const pr2_gripper_sensor_msgs::PR2GripperSlipServoCommandConstPtr& msg);

  /// force_servo real-time publisher
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
    pr2_gripper_sensor_msgs::PR2GripperForceServoData> > force_state_publisher_ ;
  ros::Subscriber sub_forceservo_command_;
  void forceServoCB(const pr2_gripper_sensor_msgs::PR2GripperForceServoCommandConstPtr& msg);

  /// place publisher and listener
  boost::scoped_ptr<
    realtime_tools::RealtimePublisher<
    pr2_gripper_sensor_msgs::PR2GripperEventDetectorData> > event_detector_state_publisher_ ;
  ros::Subscriber sub_event_detector_command_;
  void eventDetectorCB(const pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommandConstPtr& msg);
  
public:
  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  virtual void update();
  virtual void stopping();
  virtual void starting() {}

};
} 
