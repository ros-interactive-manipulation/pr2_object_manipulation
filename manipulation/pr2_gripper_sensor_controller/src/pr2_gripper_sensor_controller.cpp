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
//@brief   Controller to read data from the sensors on the pr2 gripper 
//         (accelerometer in palm, pressure arrays on the fingers, and 
//         encoder in the hand) and publish higher-level useful processed 
//         information about them, as well as perform low-level control 
//         tasks to control the gripper based on these signals in real-time.
//         
//         This controller is inteded to be interacted with through its
//         action interface, pr2_gripper_sensor_action.

#include <pr2_gripper_sensor_controller/pr2_gripper_sensor_controller.h>
#include <pluginlib/class_list_macros.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <std_srvs/Empty.h>


using namespace pr2_gripper_sensor_controller;


/**
 * In this function we grab a handle to the palm accelerometer, each finger
 * pressure array, and the motor/encoder of the gripper.
 *
 * @param *robot
 *   A pointer to our robot state instance
 * @param &n
 *   The node handle instance of the calling node.
 *
 * @return
 *   true if all handles were initialized successfully.
 */
bool PR2GripperSensorController::initializeHandles(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{

  // get a handle to the hardware interface 
  pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
  if(!hardwareInterface)
  {
      ROS_ERROR("Perhaps Something wrong with the hardware interface pointer!!!!");
  }

  // get a handle to our accelerometer 
  std::string accelerometer_name;
  if(!n.getParam("accelerometer_name", accelerometer_name))
  {
      ROS_ERROR("No accelerometer given in namespace: '%s')", n.getNamespace().c_str());
      return false;
  }
  pr2_hardware_interface::Accelerometer* accelerometerHandle = hardwareInterface->getAccelerometer(accelerometer_name);
  
  if(!accelerometerHandle)
  {
      ROS_ERROR("PR2GripperSensorController could not find sensor named '%s'", accelerometer_name.c_str());
      return false;
  }
  myAccelerationObserver = new accelerationObserver(accelerometerHandle);

  // get a handle to our left finger pressure sensors 
  std::string leftFinger_pressureSensor_name;
  if(!n.getParam("left_pressure_sensor_name", leftFinger_pressureSensor_name))
  {
        ROS_ERROR("No accelerometer given in namespace: '%s')",	n.getNamespace().c_str());
        return false;
  }
  pr2_hardware_interface::PressureSensor* leftFinger_pressureSensorHandle = hardwareInterface->getPressureSensor(leftFinger_pressureSensor_name);
  if(!leftFinger_pressureSensorHandle)
  {
      ROS_ERROR("PR2GripperSensorController could not find sensor named '%s'", leftFinger_pressureSensor_name.c_str());
      return false;
  }   

  // get a handle to our right finger pressure sensors 
  std::string rightFinger_pressureSensor_name;  
  if(!n.getParam("right_pressure_sensor_name", rightFinger_pressureSensor_name))
  {
      ROS_ERROR("No accelerometer given in namespace: '%s')",   n.getNamespace().c_str());
      return false;
  }
  pr2_hardware_interface::PressureSensor* rightFinger_pressureSensorHandle = hardwareInterface->getPressureSensor(rightFinger_pressureSensor_name);
  if(!rightFinger_pressureSensorHandle)
  {
      ROS_ERROR("PR2GripperSensorController could not find sensor named '%s'", rightFinger_pressureSensor_name.c_str());
      return false;
  }
  myPressureObserver = new pressureObserver(leftFinger_pressureSensorHandle, rightFinger_pressureSensorHandle);

  // get a handle to our desired joint 
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')", n.getNamespace().c_str());
    return false;
  }
  pr2_mechanism_model::JointState* joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("PR2GripperSensorController could not find joint named '%s'", joint_name.c_str());
    return false;
  }
  
  // instantiate our gripper action object and pass it handles to our joint_state_controller and observer objects
  myGripperController = new gripperController(joint_state_, myPressureObserver, myAccelerationObserver);

  return true;
}


/**
 * A helper function to reload parameters from the param server
 *
 * @param &req
 *   An std_srvs::Empty::Request 
 * @param &resp
 *   An std_srvs::Empty::Response
 *
 * @return
 *   returns true when complete
 */
bool PR2GripperSensorController::reloadParams(std_srvs::Empty::Request& req,std_srvs::Empty::Response& resp)
{
  if(!nodeHandle.getParam("close_speed", servo_velocity))
      ROS_ERROR("No close_speed given in namespace: '%s')", nodeHandle.getNamespace().c_str());
  servo_velocity = -servo_velocity;
  if(servo_velocity > 0)
  {
      ROS_ERROR("Incorrect sign on close_speed (negative speed is impossible), setting to 0!");
      servo_velocity = 0;
  }

  if(!nodeHandle.getParam("max_joint_effort", max_effort))
      ROS_ERROR("No max_joint_effort given in namespace: '%s')", nodeHandle.getNamespace().c_str());
  myGripperController->max_effort = max_effort;

  if(!nodeHandle.getParam("fingertip_force_limit", fingertip_force_limit))
      ROS_ERROR("No fingertip_force_limit given in namespace: '%s')", nodeHandle.getNamespace().c_str());
  fingertip_force_limit = -fingertip_force_limit;
  myGripperController->fingertip_force_limit = fingertip_force_limit;

  if(!nodeHandle.getParam("deformation_limit", deformation_limit))
      ROS_ERROR("No deformation_limit given in namespace: '%s')", nodeHandle.getNamespace().c_str());  
  myGripperController->positionMarker_limit = deformation_limit;

  if(!nodeHandle.getParam("slip_servo_start_force", fingertip_start_force))
      ROS_ERROR("No slip_servo_start_force given in namespace: '%s')", nodeHandle.getNamespace().c_str());  
  fingertip_start_force = -fingertip_start_force;

  if(!nodeHandle.getParam("force_lightest", myPressureObserver->forceLightest))
      ROS_ERROR("No force_lightest given in namespace: '%s')", nodeHandle.getNamespace().c_str());  

  if(!nodeHandle.getParam("hp_force_trigger", myPressureObserver->hpForceTrigger))
      ROS_ERROR("No hpForceTrigger given in namespace: '%s')", nodeHandle.getNamespace().c_str());  

  if(!nodeHandle.getParam("force_servo_force_tolerance", myGripperController->force_servo_force_tolerance))
      ROS_ERROR("No force_servo_force_tolerance given in namespace: '%s')", nodeHandle.getNamespace().c_str());  

  if(!nodeHandle.getParam("force_servo_velocity_tolerance", force_servo_velocity_tolerance))
      ROS_ERROR("No force_servo_velocity_tolerance given in namespace: '%s')", nodeHandle.getNamespace().c_str());  

  if(!nodeHandle.getParam("position_servo_position_tolerance", force_servo_velocity_tolerance))
      ROS_ERROR("No position_servo_position_tolerance given in namespace: '%s')", nodeHandle.getNamespace().c_str());  


  return true;

}



/**
 * ROS real-time controller initialization routine.
 * Establishes action/message/service I/O for real-time controller and inits
 * runtime variables. 
 *
 * @param *robot
 *   A pointer to our robot state instance
 * @param &n
 *   The node handle instance of the calling node.
 *
 * @return
 *   true if all hardware handles were initialized successfully.
 */
bool PR2GripperSensorController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  // make the below true if you want to publish raw data messages (debug mode)
  publish_raw_data = false;

  // store the node and robot handle
  assert(robot);
  nodeHandle = n;
  robotHandle = robot;

  // initialize handles to our sensors and joints
  bool success =  this->initializeHandles(robot, n);
  loop_count_ = 0;

  // initially place our controller into disabled mode
  control_mode = rt_state_def.DISABLED;
  
  // seed the timer
  last_time_ = robot->getTime();

  //publish our services
  updateZeros_srv_ = n.advertiseService("zero_fingertip_sensors", &PR2GripperSensorController::updateZeros, this);
  reloadParams_srv_ = n.advertiseService("reload_params", &PR2GripperSensorController::reloadParams, this);
  stopMotorOutput_srv_ = n.advertiseService("stop_motor_output", &PR2GripperSensorController::stopMotorOutput, this);

  //setup our publisher for raw data information (only if flag is set)
  if(publish_raw_data)
  {
    raw_data_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_gripper_sensor_msgs::PR2GripperSensorRawData>(n, "raw_data", 1));
  }

  //setup our publisher for joint and motor effort state information
  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>(n, "state", 1));

  //setup our publisher for contact state information
  contact_state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_gripper_sensor_msgs::PR2GripperFindContactData>(n, "contact_state", 1));

  //setup our publisher for place state information
  event_detector_state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_gripper_sensor_msgs::PR2GripperEventDetectorData>(n, "event_detector_state", 1));
  
  //setup our publisher for slip state information
  slip_state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_gripper_sensor_msgs::PR2GripperSlipServoData>(n, "slip_servo_state", 1));

  //setup our publisher for force state information
  force_state_publisher_.reset(new realtime_tools::RealtimePublisher<pr2_gripper_sensor_msgs::PR2GripperForceServoData>(n, "force_servo_state", 1));

  // setup our callback to handle position command requests
  sub_command_ = n.subscribe<pr2_controllers_msgs::Pr2GripperCommand>("command", 1, &PR2GripperSensorController::positionCB, this);

  // setup our callback to handle find_contact command requests
  sub_findcontact_command_ = n.subscribe<pr2_gripper_sensor_msgs::PR2GripperFindContactCommand>("find_contact", 1, &PR2GripperSensorController::findContactCB, this);

  // setup our callback to handle event_detector command requests
  sub_event_detector_command_ = n.subscribe<pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommand>("event_detector", 1, &PR2GripperSensorController::eventDetectorCB, this);
  
  // setup our callback to handle slip_servo command requests
  sub_slipservo_command_ = n.subscribe<pr2_gripper_sensor_msgs::PR2GripperSlipServoCommand>("slip_servo", 1, &PR2GripperSensorController::slipServoCB, this);

  // setup our callback to handle force_servo command requests
  sub_forceservo_command_ = n.subscribe<pr2_gripper_sensor_msgs::PR2GripperForceServoCommand>("force_servo", 1, &PR2GripperSensorController::forceServoCB, this);

  // upate the zero offset of our pressure sensors
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response rsp;
  updateZeros(req,rsp);

  // load our params from the param server
  reloadParams(req,rsp);

  return success;

}



/**
 * real-time thread update callback, should be run at 1khz ros rt rate.
 *
 * Listens to various command messages and publishes data back on response 
 * messages. Upon receiving a command message the controller moves into various 
 * different real-time control_modes for for the gripper.
 *
 */
void PR2GripperSensorController::update()
{

  // update our timing information
  ros::Time time = robotHandle->getTime();
  ros::Duration dt = time - last_time_;

  // do some sanity checks to see that we have a valid robot handle
  assert(robotHandle != NULL);

  //----------------------------------------
  // Begin State Machine
  
  //disabled (do nothing) mode
  if(control_mode == rt_state_def.DISABLED){}

  // position control mode
  else if(control_mode == rt_state_def.POSITION_SERVO)
  {
    // use a zero velocity goal in our position controller to have the robot 
    // gently decelerate when the position error gets small (could be current vel. instead)
    myGripperController->positionServo(servo_position,0.0);
    // keep moving our posiiton marker to avoid deformation_limit errors
    myGripperController->positionMarker = myGripperController->positionCurrent;
  }

  // force control mode
  else if(control_mode == rt_state_def.FORCE_SERVO)
  {
    //if we have reached some epsilon of our target force 
    // and the gripper is still then set consider the force contact stable
    if( ( (myGripperController->forceServo2(servo_force)) &&
            fabs(myGripperController->lpVelocity) < force_servo_velocity_tolerance))

    {
      // require the same force for 250ms
      if(findContact_delay > 250)
        stable_force = true;
      findContact_delay++;
    }    
  }

  // find contact mode
  else if(control_mode == rt_state_def.FIND_CONTACT)
  { 
    // if we have not yet met the contact requirements
    if(!myGripperController->grabObject(servo_velocity, contacts_desired))
    {
      contact_success = false;
      // make sure our velocity integral term is starting at our current position in case of contact
      myGripperController->vel_integral = myGripperController->gripper_state_now_position;
      // keep moving our posiiton marker to avoid deformation_limit errors
      myGripperController->positionMarker = myGripperController->positionCurrent;
    }
    // else if we have met the contact requirements
    else   
    {
	
      // for 50 ms after contact
      if(contactCounter < 50)
      {
        servo_position = myGripperController->positionContact;

        // keep track of the highest contact force we've applied
        servo_force = myGripperController->forceContact;	  

	// hold our position
        myGripperController->positionServo(servo_position,0.0);
      } 

      else
      {
        myGripperController->positionServo(servo_position,0.0);
        stable_contact = true;
	control_mode = rt_state_def.POSITION_SERVO;
      }

      contactCounter++;
      contact_success = true;
    }
  }

  // if in slip servo control mode
  else if(control_mode == rt_state_def.SLIP_SERVO)
      myGripperController->slipServo2();


  // else somehow we ended up with some other value for mode (should not be possible)
  else
  {
    ROS_ERROR("PR2_GRIPPER_SENSOR_CONTROLLER ENTERED A BAD STATE MACHINE STATE");
    myGripperController->jointState->commanded_effort_ = 0;
    // try to recover by setting the control mode to "disabled"
    control_mode = rt_state_def.DISABLED;
  }    

  // End State Machine
  //----------------------------------------

  //----------------------------------------
  // Non-state machine controller updating

  // update our placement variables
  if(myGripperController->place(placeConditions,acc_trigger,slip_trigger))
    placedState = true;
  
  // update zeros mode
  if(update_zeros == true)
    myPressureObserver->updateZeros2();

  //----------------------------------------
  //  Begin Feedback publication

  // publish information every nth cycle
  if(loop_count_ % 1 == 0)
  {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = servo_position;
        controller_state_publisher_->msg_.process_value = myGripperController->gripper_state_now_position;
        controller_state_publisher_->msg_.process_value_dot = myGripperController->gripper_state_now_velocity;
        controller_state_publisher_->msg_.error = myGripperController->gripper_state_now_position - servo_position;
        controller_state_publisher_->msg_.time_step = dt.toSec();
        controller_state_publisher_->msg_.command = myGripperController->gripper_state_now_measured_effort;

        controller_state_publisher_->unlockAndPublish();
      }

      if(contact_state_publisher_ && contact_state_publisher_->trylock())
      {
        contact_state_publisher_->msg_.stamp = time;
        contact_state_publisher_->msg_.left_fingertip_pad_contact = myPressureObserver->left_contact;
        contact_state_publisher_->msg_.right_fingertip_pad_contact = myPressureObserver->right_contact;
        contact_state_publisher_->msg_.left_fingertip_pad_force = myPressureObserver->padForce_left_cur_nonbiased;
        contact_state_publisher_->msg_.right_fingertip_pad_force = myPressureObserver->padForce_right_cur_nonbiased;
        contact_state_publisher_->msg_.joint_position = myGripperController->positionContact;
        contact_state_publisher_->msg_.joint_effort = myGripperController->gripper_state_now_measured_effort;
        contact_state_publisher_->msg_.rtstate.realtime_controller_state = control_mode;
        contact_state_publisher_->msg_.contact_conditions_met = stable_contact;


        contact_state_publisher_->unlockAndPublish();
      }

      if(slip_state_publisher_ && slip_state_publisher_->trylock())
      {
        slip_state_publisher_->msg_.stamp = time;
        slip_state_publisher_->msg_.joint_effort = myGripperController->gripper_state_now_measured_effort;
        slip_state_publisher_->msg_.left_fingertip_pad_force = myPressureObserver->padForce_left_cur_nonbiased;
        slip_state_publisher_->msg_.right_fingertip_pad_force = myPressureObserver->padForce_right_cur_nonbiased;
        slip_state_publisher_->msg_.deformation = myGripperController->positionCurrent - myGripperController->positionMarker;
        slip_state_publisher_->msg_.slip_detected = myGripperController->slip_flag;
        slip_state_publisher_->msg_.fingertip_force_limit_reached = myGripperController->force_limit_flag;
        slip_state_publisher_->msg_.deformation_limit_reached = myGripperController->deformation_limit_flag;
        slip_state_publisher_->msg_.gripper_empty = myGripperController->dropped_flag;
        slip_state_publisher_->msg_.rtstate.realtime_controller_state = control_mode;
        
        slip_state_publisher_->unlockAndPublish();
      }

      if(force_state_publisher_ && force_state_publisher_->trylock())
      {
        force_state_publisher_->msg_.stamp = time;
        force_state_publisher_->msg_.joint_effort = myGripperController->gripper_state_now_measured_effort;
        force_state_publisher_->msg_.left_fingertip_pad_force = myPressureObserver->padForce_left_cur_nonbiased;
        force_state_publisher_->msg_.right_fingertip_pad_force = myPressureObserver->padForce_right_cur_nonbiased;
        force_state_publisher_->msg_.force_achieved = stable_force;
        force_state_publisher_->msg_.rtstate.realtime_controller_state = control_mode;
        
        force_state_publisher_->unlockAndPublish();
      }

      if(event_detector_state_publisher_ && event_detector_state_publisher_->trylock())
      {
        event_detector_state_publisher_->msg_.stamp = time;
        event_detector_state_publisher_->msg_.trigger_conditions_met = placedState;
        event_detector_state_publisher_->msg_.slip_event = myPressureObserver->placeContact;
        event_detector_state_publisher_->msg_.acceleration_event = myAccelerationObserver->placeContact;
        event_detector_state_publisher_->msg_.acceleration_vector[0] = myAccelerationObserver->aX_bp;
        event_detector_state_publisher_->msg_.acceleration_vector[1] = myAccelerationObserver->aY_bp;
        event_detector_state_publisher_->msg_.acceleration_vector[2] = myAccelerationObserver->aZ_bp;
        
        event_detector_state_publisher_->unlockAndPublish();
      }
  }

  // publish information every nth cycle
  if(loop_count_ % 1 == 0)
  {
    if(publish_raw_data)
    {
      raw_data_publisher_->msg_.stamp = time;
      raw_data_publisher_->msg_.left_finger_pad_force = myPressureObserver->padForce_left_cur_nonbiased;
      raw_data_publisher_->msg_.right_finger_pad_force = myPressureObserver->padForce_right_cur_nonbiased;
      raw_data_publisher_->msg_.left_finger_pad_force_filtered = myPressureObserver->padForce_left_cur;
      raw_data_publisher_->msg_.right_finger_pad_force_filtered = myPressureObserver->padForce_right_cur;
      raw_data_publisher_->msg_.left_finger_pad_forces = myPressureObserver->pressure_current_zerod.pressure_left;
      raw_data_publisher_->msg_.right_finger_pad_forces = myPressureObserver->pressure_current_zerod.pressure_right;
      raw_data_publisher_->msg_.left_finger_pad_forces_filtered = myPressureObserver->pressure_cur_bias.pressure_left;
      raw_data_publisher_->msg_.right_finger_pad_forces_filtered = myPressureObserver->pressure_cur_bias.pressure_right;
      raw_data_publisher_->msg_.acc_x_raw = myAccelerationObserver->aX_lp;
      raw_data_publisher_->msg_.acc_y_raw = myAccelerationObserver->aY_lp;
      raw_data_publisher_->msg_.acc_z_raw = myAccelerationObserver->aZ_lp;
      raw_data_publisher_->msg_.acc_x_filtered = myAccelerationObserver->aX_bp;
      raw_data_publisher_->msg_.acc_y_filtered = myAccelerationObserver->aY_bp;
      raw_data_publisher_->msg_.acc_z_filtered = myAccelerationObserver->aZ_bp;
      raw_data_publisher_->msg_.left_contact = myPressureObserver->left_contact;
      raw_data_publisher_->msg_.right_contact = myPressureObserver->right_contact;

      raw_data_publisher_->unlockAndPublish();
    }
  }

  //  End Feedback publication
  //----------------------------------------

  // update our gripper action, as well as our accelerometer and pressure states with it
  myGripperController->updateData();
  
  // increment our counter and store the time
  loop_count_++;
  last_time_ = time;
}

/**
 * real-time callback to stop controller
 *
 * Sends a zero-effort command to the motor before quitting
 *
 */
void PR2GripperSensorController::stopping()
{
    // turn the motors off before stopping
    myGripperController->jointState->commanded_effort_ = 0;     
}



/**
 * This function updates the pressure sensor zero offest value.
 * It will run for 0.25 seconds and average the sensor readings,
 * after which this average is used to offest the senor readings.
 *
 * @param &req
 *   An std_srvs::Empty::Request 
 * @param &resp
 *   An std_srvs::Empty::Response
 *
 * @return
 *   returns true when complete
 */
bool PR2GripperSensorController::updateZeros(std_srvs::Empty::Request& req,std_srvs::Empty::Response& resp)
{
  ROS_INFO("Updating zeros....");
  update_zeros = true;
  ros::Duration(0.25).sleep(); 
  update_zeros = false;
  ROS_INFO(".... zeros finished updating");
 
  return true;
}


/**
 * This function forces the real-time controller into a mode where no information
 * is sent to the motor. Useful for stopping and interrupting other control modes
 *
 * @param &req
 *   An std_srvs::Empty::Request 
 * @param &resp
 *   An std_srvs::Empty::Response
 *
 * @return
 *   returns true when complete
 */
bool PR2GripperSensorController::stopMotorOutput(std_srvs::Empty::Request& req,std_srvs::Empty::Response& resp)
{
  ROS_INFO("Stopping gripper command output.");
  control_mode = rt_state_def.DISABLED;
 
  return true;
}


/**
 * Helper function called during many action commands to reset key stat
 * variables that are used to determine what is happening inside the controller
 */
void PR2GripperSensorController::reinitializeValues()
{
  findContact_delay = 0;
  stable_force = false;

  // reinitialize force ramp state
  myGripperController->firstRamp = true;

  myGripperController->vel_integral_vcontrol = myGripperController->positionCurrent;

  // reinialize the grasp contact states
  stable_contact = false;

  // reinitialize the contact forces
  myGripperController->forceContact = 0;
  myGripperController->forceContact_l = 0;
  myGripperController->forceContact_r = 0;
  
  // reset several important variables
  myGripperController->positionMarker = myGripperController->positionCurrent;
  myGripperController->slip_flag = false;
  myGripperController->force_limit_flag = false;
  myGripperController->deformation_limit_flag = false;

  myGripperController->dropped_flag = false;
  
}


/**
 * Function to take in a position request goal request and place the
 * controller into position control mode.
 *
 * @param &msg
 *   A pr2_controllers_msgs::Pr2GripperCommandConstPtr that indicates
 *   what the position request goal is
 */
void PR2GripperSensorController::positionCB(const pr2_controllers_msgs::Pr2GripperCommandConstPtr& msg)
{
  reinitializeValues();
  
  // store the message data
  servo_position = msg->position;
  max_effort = msg->max_effort;
  myGripperController->max_effort = max_effort;
  

  // put the max effort value commanded by the user back on the param server
  nodeHandle.setParam("max_joint_effort", max_effort);

  
  // change the controller state
  control_mode = rt_state_def.POSITION_SERVO;
  
  ROS_INFO("Gripper Position Servo to: %f", servo_position); 
}


/**
 * Function to take in a find_contact request goal request and place the
 * controller into contact finding control mode.
 *
 * @param &msg
 *   A pr2_gripper_sensor_msgs::PR2GripperFindContactCommandConstPtr that indicates
 *   what the find_contact goal is
 */
void PR2GripperSensorController::findContactCB(const pr2_gripper_sensor_msgs::PR2GripperFindContactCommandConstPtr& msg)
{ 
  if(myPressureObserver->broken_sensor)
  {  
    ROS_ERROR("REFUSING TO FIND CONTACT - PRESSURE SENSOR HAS ZERO READING AND MAY BE MALFUNCTIONING!");
    return;
  }

  reinitializeValues();
  contactCounter = 0;
  
  // copy over the data from our message
  contacts_desired = msg->contact_conditions;

  // reinialize the grasp contact states
  myPressureObserver->left_contact = false;
  myPressureObserver->right_contact = false;

  // change the controller state
  control_mode = rt_state_def.FIND_CONTACT;

}


/**
 * Function to take in a event_detector request goal request and 
 * set the event_detector parameters to trigger on this goal
 *
 * @param &msg
 *   A pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommandConstPtr that indicates
 *   the conditions of the event trigger
 */
void PR2GripperSensorController::eventDetectorCB(const pr2_gripper_sensor_msgs::PR2GripperEventDetectorCommandConstPtr& msg)
{ 
  placeConditions = msg->trigger_conditions;
  acc_trigger = msg->acceleration_trigger_magnitude;
  slip_trigger = msg->slip_trigger_magnitude;

  placedState = false;
  myPressureObserver->placeContact = false;
  myAccelerationObserver->placeContact = false;
}

/**
 * Function to take in a slip_servo request goal request and 
 * set the controller into slip servo mode.
 *
 * @param &msg
 *   A  pr2_gripper_sensor_msgs::PR2GripperSlipServoCommandConstPtr 
 */
void PR2GripperSensorController::slipServoCB(const pr2_gripper_sensor_msgs::PR2GripperSlipServoCommandConstPtr& msg)
{
  if(myPressureObserver->broken_sensor)
  {  
    ROS_ERROR("REFUSING TO SLIP SERVO - PRESSURE SENSOR HAS ZERO READING AND MAY BE MALFUNCTIONING!");
    return;
  }

  reinitializeValues();

  // check if the user wanted to use the default force or input a force level
  if(fingertip_start_force <= 0.0)
    myGripperController->servoForce = fingertip_start_force;
  else
    myGripperController->servoForce = servo_force;
  
  // change the controller state
  control_mode = rt_state_def.SLIP_SERVO;

  ROS_INFO("Starting Slip Servo with: %f N", myGripperController->servoForce);
}


/**
 * This function forces the real-time controller into force servo mode,
 * sending motor control commands to the gripper to attempt to hold a specified
 * force as seen by the finger pressure sensors. 
 *
 * @param &msg
 *   A  pr2_gripper_sensor_msgs::PR2GripperForceServoCommandConstPtr that defines
 *   the force servo target force
 */
void PR2GripperSensorController::forceServoCB(const pr2_gripper_sensor_msgs::PR2GripperForceServoCommandConstPtr& msg)
{
  if(myPressureObserver->broken_sensor)
  {  
    ROS_ERROR("REFUSING TO FORCE SERVO - PRESSURE SENSOR HAS ZERO READING AND MAY BE MALFUNCTIONING!");
    return;
  }

  reinitializeValues();

  if(msg->fingertip_force >= 0)
    servo_force = -(msg->fingertip_force);
  else
    servo_force = 0.0;    
  
  // change the controller state
  control_mode = rt_state_def.FORCE_SERVO;

  ROS_INFO("Starting Force Servo with: %f N", servo_force);
}


// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(pr2_gripper_sensor_controller,PR2GripperSensorController, 
                         pr2_gripper_sensor_controller::PR2GripperSensorController, 
                         pr2_controller_interface::Controller)
