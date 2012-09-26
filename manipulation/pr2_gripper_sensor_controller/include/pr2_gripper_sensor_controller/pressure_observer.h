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
//@brief   pressure_observer.h

#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_gripper_sensor_msgs/PR2GripperPressureData.h>
#include <pr2_gripper_sensor_controller/digitalFilter.h>

#ifndef _PRESSURE_OBSERVER
#define _PRESSURE_OBSERVER

#define UNITS_PER_N 1600

class pressureObserver{
  
  public:
    // functions
    pressureObserver(pr2_hardware_interface::PressureSensor* left,pr2_hardware_interface::PressureSensor* right);    // constructor
    ~pressureObserver();                                // default destructor
    void spin();                                        // method to update sensor data. should run at 1khz
    bool graspContact(int contactsDesired);             // detects finger impacts
    void updateZeros2();                                // method to update our zero value offsets (zero_offset)
    bool checkPlaceContact(double dF);                  // method to look for impact slip and return true
    bool checkSlip(double slip_motion_limit = 0.007, double slip_force_limit = 0.18);
    int  checkSideImpact(double dF);                    // method to check for side impacts on finger sensors
    void updateBrokenSensorStatus();


    double padForce_cur;                                // fingerpad force average between both fingers
    double padForce_cur_nonbiased;                      // force average from the non-biased pressures
    double padForce_left_cur, padForce_left_prev;       // current cumulative force on our left/rigth fingerpads
    double padForce_left_cur_nonbiased;
    double padForce_right_cur, padForce_right_prev;     // previous cumulative force on our left/right fingerpads
    double padForce_right_cur_nonbiased;
    double totalLoading_left, totalLoading_right;       // total dynamic laoding on our left/right pads
    double totalLoading,totalLoading_prev;              // left and right loading average
    double cellLoading_left, cellLoading_right;         // cellular loading on the left/right fingers
    double forceLightest;
    double hpForceTrigger;
    double lfForce_left, lfForce_right;
    bool broken_sensor;                                 // flag to indicate we may have a broken sensor
    bool left_contact, right_contact;                   // flags to indicate whether a left or right contact happened
    bool placeContact;
    
    // storage of our pressure sensor state information. _now is the most recent raw data. _prev is the last subsampled value
    // _current is the current subsampled value
    pr2_gripper_sensor_msgs::PR2GripperPressureData  pressure_now, pressure_prev, pressure_current;
    pr2_gripper_sensor_msgs::PR2GripperPressureData  pressure_thresh_now, pressure_thresh_prev, pressure_thresh_current;
    pr2_gripper_sensor_msgs::PR2GripperPressureData  pressure_prev_zerod, pressure_current_zerod;
    pr2_gripper_sensor_msgs::PR2GripperPressureData  pressure_prev_bias, pressure_cur_bias;
    pr2_gripper_sensor_msgs::PR2GripperPressureData  pressure_bp;

    digitalFilter *pressureLFFilt_left[22];
    digitalFilter *pressureLFFilt_right[22];


  private:
    void getPadCentroid(boost::array<double,22> pressureArray, double *x, double *y);
    void updateXYCentroids();
    bool updatePressureState();                         // method to acquire new data from both fingers and apply appropriate subsampling 
    void updateTotalLoading();                          // method to update the toal loading of each finger totalLoad_left _right
    void zero();                                        // method to zero out data
    void bias_estimator();                              // remove mean bias for near zero data
    double getPadForce(boost::array<double,22> pressureArray);  // translate fingertip pressure vector to cumultive sum on the fingerpad (just the pad)
    void updateContactState();                          // update the state of the robots contact sensors


    pr2_hardware_interface::PressureSensor* left_finger;// a copy of our pressure sensor handle
    pr2_hardware_interface::PressureSensor* right_finger;// a copy of our pressure sensor handle

    int subSample_cnt;                                  // a counter to keep track of how many samples we ignored during subsampling
    int subSample_steps;                                // 1+ the number of values to skip during subsampling
    int zero_samples;                                   // the number of samples to use when trying to evaluate our zero offset
    int zero_cnt;                                       // counter to keep track of how many samples we've stored for zero estimation
    pr2_gripper_sensor_msgs::PR2GripperPressureData zero_offset;// a storage container for our zero offset values

    double time_prev, time_cur;                         // stroage containers for the previous and current pressure update times
    double dt;                                          // the time expired between pressure update iterations

};



#endif // _PRESSURE_OBSERVER
