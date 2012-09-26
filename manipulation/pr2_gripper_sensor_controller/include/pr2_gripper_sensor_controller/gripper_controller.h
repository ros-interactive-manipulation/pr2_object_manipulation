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
//@brief   gripperController.h - class to read data from gripper sensor
//         data helper classes and make the gripper take action on it
//         through a variety of different control modes.

#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_gripper_sensor_controller/pressure_observer.h>
#include <pr2_gripper_sensor_controller/acceleration_observer.h>
#include <pr2_gripper_sensor_controller/digitalFilter.h>

#ifndef _GRIPPER_CONTROLLER
#define _GRIPPER_CONTROLLER


class gripperController{
  
  public:
    // functions
  gripperController(pr2_mechanism_model::JointState* joint_state_, pressureObserver* pressure_observer_, accelerationObserver* acceleration_observer_);    // ctor
    ~gripperController();                                             // default dtor
    bool velocityServo(double desiredVel);
    bool positionServo(double desiredPos, double desiredVel);     // servo to a desired position
    void updateData();                                            // update gripper state info (should run at 1khz)
    bool initializeGripper();                                     // 
    bool forceServo2(double desired_Force);    // servo gripper on pressure sensor force
    bool grabObject(double close_speed, int contactsDesired);      // look for object contacts on each finger
    bool slipServo2();                                            // servo gripper on slip information from psensors
    bool forceRampTo(double force, double duration);              // ramp the current force to a specific value over time
    bool place(int placeConditions,double acc_trigger, double slip_trigger);                                                 // method to look for contact and return true when detected


    // externally modified variables
    double max_effort;
    double fingertip_force_limit;
    double positionMarker_limit;
    double positionMarker;
    
    // externally accessed variables
    bool slip_flag;
    bool deformation_limit_flag;
    bool force_limit_flag;
    bool dropped_flag;
    
    //variables
    //PerformanceCounter counter;
    double vel_integral;                                          // velocity integral for force control
    double vel_integral_vcontrol;
    double lpVelocity;                                            // a low-passsed version of our velocity
    double accThresh;
    digitalFilter *velocityLPFilt;
    double positionOpen;                                          // the 'open' position of gripper
    double positionClosed;                                        // the 'closed' position of gripper
    double positionCurrent;                                       // current gripper position
    double positionDropped;                                       // the position below which we consider things dropped
    double servoForce;                                            // the force we are trying to servo to
    double objectForce;                                           // the force found to be best to hold the current object                                             
    double positionContact;                                       // the last position we saw contact at
    double forceContact;                                          // the force seen at contact (average)
    double forceContact_l, forceContact_r;                        // the force seen on l/r fingers at contact
    double force_servo_force_tolerance;

    // storage of our gripper state information
    double gripper_state_now_measured_effort, gripper_state_now_position, gripper_state_now_velocity;
    double gripper_state_prev_measured_effort, gripper_state_prev_position, gripper_state_prev_velocity;

    double kP;                                                    // proportional gain of position controller
    double kD;                                                    // derivative gain of position controller
    double dt;                                                    // time expired this current timestep
    double coulomb;                                               // coulomb term for position controller
    
    pr2_mechanism_model::JointState* jointState;                  // a copy of our joint handle
    pressureObserver* myPressureObserver;                         // a copy of our pres. obsrv. handle
    accelerationObserver* myAccelerationObserver;                         // a copy of our acc. obsrv. handle

    bool firstRamp;                                               // flag to indicate the first iteration of forceRampTo
    double ramp_start_time;                                       // time that forceRampTo started
    double ramp_start_force;                                      // force that forceRampTo started on

  private:
    double initialPosition;                                       // the starting position when this class was called
    double time_prev, time_cur;                                   // the prev/current time of our control loop
};



void gripperController::updateData()
{
  // update our timing information
  time_prev = time_cur;
  //counter.StopCounter();
  time_cur = ros::Time::now().toSec();//counter.GetElapsedTime();


  // move old variables to gripperState_prev
  gripper_state_prev_measured_effort = gripper_state_now_measured_effort;
  gripper_state_prev_position = gripper_state_now_position;
  gripper_state_prev_velocity = gripper_state_now_velocity;

  // get new variables and store them in _now
  gripper_state_now_measured_effort = jointState->measured_effort_;                         
  gripper_state_now_position = jointState->position_; 
  gripper_state_now_velocity = jointState->velocity_;   
  lpVelocity = velocityLPFilt->getNextFilteredValue(gripper_state_now_velocity);

  // store a convenient double for our current position
  positionCurrent = gripper_state_now_position;

  // call our pressure sensor and accelerometer update algorithm
  myPressureObserver->spin();
  myAccelerationObserver->spin();
}


// method to close the hand and stop when dual contacts are encountered
bool gripperController::grabObject(double close_speed, int contactsDesired)
{
  // if the pressure sensors have not yet detected dual impacts
  if( !(myPressureObserver->graspContact(contactsDesired)) )
  {
    velocityServo(close_speed);  // keep closing
    return false;
  }
  // if we have detected dual impacts
  else
  {
    // store the contact position and force
    positionContact = positionCurrent;
    if(-myPressureObserver->padForce_left_cur_nonbiased < forceContact_l)
      forceContact_l = -myPressureObserver->padForce_left_cur_nonbiased;
    if(-myPressureObserver->padForce_right_cur_nonbiased < forceContact_r)
      forceContact_r = -myPressureObserver->padForce_right_cur_nonbiased;
    if(-myPressureObserver->padForce_cur_nonbiased < forceContact)
      forceContact = -myPressureObserver->padForce_cur_nonbiased;
    return true;
  }
}


// method to open the hand when gripper transience is encountered
bool gripperController::place(int placeConditions, double acc_trigger, double slip_trigger)
{
  bool accContact = myAccelerationObserver->checkPlaceContact(acc_trigger);
  bool slipContact = myPressureObserver->checkPlaceContact(slip_trigger);  
  int  sideImpact = myPressureObserver->checkSideImpact(0.2);
  bool returnValue = false;
  
  if(placeConditions == 0 )
    returnValue = accContact || (sideImpact != -1);
  else if(placeConditions == 1 )
    returnValue = accContact && slipContact;
  else if(placeConditions == 2 )
    returnValue = accContact || slipContact || (sideImpact != -1);
  else if(placeConditions == 3)
    returnValue = slipContact;
  else if(placeConditions == 4)
    returnValue = accContact;


  return returnValue;
}


// servo slip
bool gripperController::slipServo2()
{
  // note: with higher forceservo gains .0005 seems good.
  double slipIncrementPercent = 0.002; // .001 worked nicely. .005 seemed a little agressive   // actually percent/100. Since we are "oversampling" the real percentage we are increasing is this *41        

  if( myPressureObserver->checkSlip())
  {
    servoForce = servoForce + servoForce*slipIncrementPercent;
    slip_flag = true;
  }
  else
    slip_flag = false;

  // check for our force limit
  if(( servoForce <= fingertip_force_limit ) && (fingertip_force_limit <= 0.0))
  {
    force_limit_flag = true;
    servoForce = fingertip_force_limit;
  }
  else
    force_limit_flag = false;

  forceServo2(servoForce);
 
  // if the pressure ever drops to less than 0.1N less than our lightest applicable force
  // or the gripper ends up closed, consider the object f
  if( (-myPressureObserver->padForce_cur_nonbiased > -((myPressureObserver->forceLightest)-0.25)) || positionCurrent <= positionDropped)
  {  
    dropped_flag = true;
    positionMarker = positionCurrent;    // reset our position marker since the object was dropped
    jointState->commanded_effort_ = -100.0;   // close as hard as possible
  }
  else
    dropped_flag = false;

 

  return true;
}




// method to servo the robot to a desired finger force value
bool gripperController::forceServo2(double desired_Force)
{
  // do not allow the force controller to apply anything weaker than forceLightest
  if(desired_Force > -(myPressureObserver->forceLightest))
    desired_Force = -(myPressureObserver->forceLightest);

  // store the force we are currently trying to servo to
  servoForce = desired_Force;

  // figure out which finger is seeing less force, and use that value
  double minFingerForce = -std::min(myPressureObserver->padForce_left_cur_nonbiased,myPressureObserver->padForce_right_cur_nonbiased);

  // our gain for the force controller
  double kP_f =  0.0008;//was 0.0000003; // was .0006
  // if we are trying to close more use a strong gain
  if( desired_Force-minFingerForce < 0)
    kP_f = 0.0016; // .008 breaks eggs (spikes too hard). .003 seems good

  // calculate our force components
  double v_force = -kP_f * (minFingerForce - desired_Force);

  // limit our velocity values to limit extremely high velocities on large force descrepancies
  double v_bound = 0.50;  // set a limit on the speed (m/s?)  .005 worked  .015 worked  .035 worked  .075 looked good
  if(v_force < -v_bound)
    v_force = -v_bound;
  else if(v_force > v_bound)
    v_force = v_bound;


  // limit our integral (position) term
  double p_bound = 0.03;
  if( (((vel_integral-positionCurrent) > p_bound) && v_force*dt > 0) ||
      (((vel_integral-positionCurrent) < -p_bound) && v_force*dt < 0) )
    {}
  else
  {
    // integrate the velocity over time
    vel_integral += v_force * dt;
  }

  // assign our position to be the integrated velocity value
  double p_force = vel_integral;

  // zero closes gripper    
  positionServo(p_force,v_force);

  // return true if force achieved within some tolerance
  if(fabs(minFingerForce-desired_Force) < force_servo_force_tolerance)
    return true;
  else
    return false;
}


// method to ramp the current force to a specific value over a defined timeperiod
bool gripperController::forceRampTo(double force, double duration)
{
  // if it is the first time entering this forceRampTo function
  if(firstRamp)				       
  {
    // store the time and force we started at
    ramp_start_time = time_cur;      
    ramp_start_force = -std::max(myPressureObserver->padForce_left_cur_nonbiased,myPressureObserver->padForce_right_cur_nonbiased); 
    firstRamp = false;
    return false;
  }

  // if the time of the ramp has not expired ramp down linearly
  else if(time_cur-ramp_start_time < duration)
  {
    forceServo2( ((force-ramp_start_force)*((time_cur - ramp_start_time)/duration)) + ramp_start_force);
    return false;
  }

  return true;

}


bool gripperController::velocityServo(double desiredVel)
{ 
  // integrate the velocity over time
  vel_integral_vcontrol += desiredVel * dt;
  return positionServo(vel_integral_vcontrol, desiredVel);
}


// method to position servo the robot to a desired position. non-blocking. requires continuous polling to accurately update torque information
// @input desiredPos - the desired position to move to. 
// @input desireVel - the desiredVelocity to try to move at
// @output - true if the desired position has been achieved (within some threshold), false if it has not yet been achieved
bool gripperController::positionServo(double desiredPos, double desiredVel)
{

  // NOTE: This is coded so that it only stops the robot from closing too much, not opening
  // check if we are using a deformation limit
  // and if we are violating that limit, and trying to violate it more
  if( (positionMarker_limit >= 0.0) && (positionCurrent < positionMarker-positionMarker_limit) && (desiredPos < positionCurrent) )
    desiredPos = positionMarker-positionMarker_limit;
  if( (positionMarker_limit >= 0.0) && (positionCurrent < positionMarker-positionMarker_limit) )
    deformation_limit_flag = true;
  else
    deformation_limit_flag = false;
    
  
  // calculate our force components
  double pos_force = -kP * (gripper_state_now_position - desiredPos);
  double vel_force = -kD * (gripper_state_now_velocity - desiredVel);

  // deal with coulomb friction in the gripper
  if      (desiredVel > 0.0)  vel_force += coulomb;
  else if (desiredVel < 0.0)  vel_force -= coulomb;

  double fbk_force = pos_force + vel_force;

  // ensure we do not exceed the max_effort value
  if (max_effort >= 0.0)
    fbk_force = std::max(-max_effort, std::min(fbk_force, max_effort)); 
  
  // ensure that we do not exceed our deformation limit
  jointState->commanded_effort_ =  fbk_force;

  return false;
}


// method returns true when initialization complete
bool gripperController::initializeGripper()
{
    return true;
}

// ctor
gripperController::gripperController(pr2_mechanism_model::JointState* joint_state_, pressureObserver* pressure_observer_, accelerationObserver* acceleration_observer_)
{
  // store our gripper joint state handle
  jointState = joint_state_;

  // store our pressure observer
  myPressureObserver = pressure_observer_;

  // store our acceleration observer
  myAccelerationObserver = acceleration_observer_;

  // setup our position controller gain values
  kP = 20000; // 20000
  kD = 5000;  // 5000
  coulomb = 7;

  // some default's for our variables
  positionOpen = 0.1;         // param server stuff?
  positionClosed = 0.0;       // param server stuff?
  vel_integral = 0.0;
  positionDropped = 0.003;    // 3mm
  servoForce = 0.0;
  accThresh = 4.0;

  // instatiate our digital filter
  // 1st order butterworth, 5 hz lp
  float b_vfilt[] = { 0.0155, 0.0155};
  float a_vfilt[] = {1.0, -0.9691};
  velocityLPFilt = new digitalFilter(1, true,b_vfilt,a_vfilt);
  
  // our timestep time we expect
  dt = 0.001;  // 1 ms
  
  // store the initial position                                                                                                                                   
  initialPosition = jointState->position_;

  // set the firstInitialize state to true
  firstRamp = true;

}


// destructor
gripperController::~gripperController()
{}


#endif // _GRIPPER_CONTROLLER
