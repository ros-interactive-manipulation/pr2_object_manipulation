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
//@brief   pressure_observer.cpp

#include <pr2_gripper_sensor_controller/pressure_observer.h>

// method to be called each 1ms timestep in realtime to ensure data collection and manipulation happens properly
void pressureObserver::spin()
{
  // update the sensor readings and perform actions if new data was aquired.
  // this inner loop shuld run (become true) at the update rate of the pressure sensors (24.9 Hz)
  if(updatePressureState())
  {
    // update our time values
    time_prev = time_cur;
    time_cur = ros::Time::now().toSec();

    zero();     		   // zero out our data
    bias_estimator();              // calculate filtered (bp,hp) force values
    updateTotalLoading();          // update our total loading estimates
    updateXYCentroids();
    updateBrokenSensorStatus();    // update our flag to indicate a broken sensor

    updateContactState();
  }
    
}


void pressureObserver::updateBrokenSensorStatus()
{
  if(getPadForce(pressure_now.pressure_left) == 0 || getPadForce(pressure_now.pressure_left) == 0)
    broken_sensor = true;
  else
    broken_sensor = false;

}


// NOTE: remove bias and do for all? remove completely? try remove completely first.
// remove the mean for near-zero data at low frequencies
void pressureObserver::bias_estimator()
{
  // position high-pass filter cutoff frequency                               
  double poshp_fc = 5.0;   // Hz                                               
  double poshp_lambda = 2*3.14159265*poshp_fc;
  double dt = .041;
 
  // for all cells (only if we are not currently zeroing things out)
  for( int i = 0; (i < 22); i ++) 
  {
    pressure_prev_bias.pressure_left[i] = pressure_cur_bias.pressure_left[i];
    pressure_prev_bias.pressure_right[i] = pressure_cur_bias.pressure_right[i];
                
    pressure_cur_bias.pressure_left[i] = ((1/(1+dt*poshp_lambda))*pressure_prev_bias.pressure_left[i]) + ((1/(1+dt*poshp_lambda))*(pressure_current_zerod.pressure_left[i]-pressure_prev_zerod.pressure_left[i]));
    pressure_cur_bias.pressure_right[i] = ((1/(1+dt*poshp_lambda))*pressure_prev_bias.pressure_right[i]) + ((1/(1+dt*poshp_lambda))*(pressure_current_zerod.pressure_right[i]-pressure_prev_zerod.pressure_right[i]));

    // calculate our band-passed data
    pressure_bp.pressure_left[i] = pressureLFFilt_left[i]->getNextFilteredValue(pressure_current_zerod.pressure_left[i]);
    pressure_bp.pressure_right[i] = pressureLFFilt_right[i]->getNextFilteredValue(pressure_current_zerod.pressure_right[i]);

    pressure_cur_bias.rostime = ros::Time::now().toSec();
  }  
}


bool pressureObserver::checkPlaceContact(double dF)
{
  if( checkSlip(dF,0.2)  || placeContact )
    placeContact = true;
  else
    placeContact = false;
  return placeContact;
}


int pressureObserver::checkSideImpact(double dF)
{
  for(int i = 0; i < 7; i++)
  {
    // should really be 600 and 400 depending on sensor. heh.
    if( (pressure_cur_bias.pressure_left[i]/500.0) > dF )
      return i+1;
    if( (pressure_cur_bias.pressure_right[i]/500.0) > dF )
      return i+10;
  }
  return -1;
}


// method to watch pressure sensor transience during grasp and return true
// when both fingers saw a significant force transience
bool pressureObserver::graspContact(int contactsDesired)
{

  // if we are looking for both fingers to contact 
  if( contactsDesired == 0   && left_contact && right_contact)
     return true;
  // if looking for a single finger contact
  else if( (contactsDesired == 1 && left_contact) || (contactsDesired == 2 && right_contact))
     return true;
  // if we are looking for any contact
  else if( (contactsDesired ==3) && (left_contact || right_contact) )
    return true;
 
  return false;
}                                              


// method to update contact state of fingerpads
void pressureObserver::updateContactState()
{
  //float totalLoading_contactThresh = 3000.0/UNITS_PER_N;  // was 3000 before unit change to N. Probably should be on param server
 
  if(fabs(padForce_left_cur) > hpForceTrigger || padForce_left_cur_nonbiased > forceLightest-.25 )
    left_contact = true;
  else
    left_contact = false;

  if(fabs(padForce_right_cur) > hpForceTrigger || padForce_right_cur_nonbiased > forceLightest-.25 )
    right_contact = true;
  else
    right_contact = false;
}


// method to update the toal loading of each finger PAD totalLoad_left _right
void pressureObserver::updateTotalLoading()                                    
{
  // store our previous forces on the fingerpads
  padForce_left_prev = padForce_left_cur;
  padForce_right_prev = padForce_right_cur;

  // get our new fingerpad forces
  padForce_left_cur = getPadForce(pressure_cur_bias.pressure_left);
  padForce_right_cur = getPadForce(pressure_cur_bias.pressure_right);
  padForce_left_cur_nonbiased = getPadForce(pressure_current_zerod.pressure_left);
  padForce_right_cur_nonbiased = getPadForce(pressure_current_zerod.pressure_right);
  padForce_cur = (padForce_left_cur + padForce_right_cur)/2.0;
  padForce_cur_nonbiased = (padForce_left_cur_nonbiased + padForce_right_cur_nonbiased)/2.0;

  // take the discrete derivative and absolute value to get the new total loading
  totalLoading_left = fabs((padForce_left_cur-padForce_left_prev)/(dt));
  totalLoading_right = fabs((padForce_right_cur-padForce_right_prev)/(dt));
  totalLoading_prev = totalLoading;
  totalLoading = (totalLoading_left+totalLoading_right)/2.0;

  lfForce_left = getPadForce(pressure_bp.pressure_left);
  lfForce_right = getPadForce(pressure_bp.pressure_right);
}


// method to take in a fingertip pressure vector and return the cumultive sum on the fingerpad (just the pad)
double pressureObserver::getPadForce(boost::array<double,22> pressureArray)
{
  double tempLoading = 0.0;

  // sum the fingerpads to get the new total loading
  for( int i = 7; i < 22; i ++)
  {
    tempLoading += pressureArray[i];
  }

  return tempLoading/UNITS_PER_N;

}


// method to update our zero value for each cell based on a low-pass filtered version of the no-load pressure signal
void pressureObserver::updateZeros2()
{
  double 
  zero_weight = 0.6;
  for( int i = 0; i < 22; i ++) 
  {
    zero_offset.pressure_left[i] = (  ((1.0-zero_weight)*zero_offset.pressure_left[i]) + (zero_weight*pressure_current.pressure_left[i]) ); 
    zero_offset.pressure_right[i] = ( ((1.0-zero_weight)*zero_offset.pressure_right[i]) + (zero_weight*pressure_current.pressure_right[i]) ); 
  }
}


// method to zero out our data by modifying pressureState_current
void pressureObserver::zero()
{
    // update our zero vector
    for( int i = 0; i < 22; i ++)
    {
      pressure_prev_zerod.pressure_left[i] = pressure_current_zerod.pressure_left[i];
      pressure_prev_zerod.pressure_right[i] = pressure_current_zerod.pressure_right[i];

      pressure_current_zerod.pressure_left[i] = pressure_current.pressure_left[i] - zero_offset.pressure_left[i];
      pressure_current_zerod.pressure_right[i] = pressure_current.pressure_right[i] - zero_offset.pressure_right[i];
    }  
}


// method to acquire new data from both fingers and apply appropriate subsampling
// @return true if the data was updated (we took a subsample), false if no data update occured (repetitive data)
bool pressureObserver::updatePressureState()
{

  bool newData = false;
  bool updated = false;

   // grab new data from the left and right sensors
  std::vector< uint16_t > pressureVector_left = left_finger->state_.data_;                                                                                            
  std::vector< uint16_t > pressureVector_right = right_finger->state_.data_;                                                                                          
  for( int i = 0; i < 22; i ++)                                        
  {                                                                                                                                                                                     
      pressure_now.pressure_left[i] = (double)pressureVector_left[i]; 
      pressure_now.pressure_right[i] = (double)pressureVector_right[i]; 

      // check if any values have changed, and if so set a flag
      if(pressure_now.pressure_left[i]  != pressure_current.pressure_left[i] || pressure_now.pressure_right[i] != pressure_current.pressure_right[i])
      {
	newData = true;
      }
  }

  // if we went over our 41st sample (the rate at which samples are updated) then update our stored values
  // subSample_cnt >= 41 if too many samples happened
  if( newData || subSample_cnt >= subSample_steps)
  {

     for( int i = 0; i < 22; i ++)                
     {
        // store the current value as previous
        pressure_prev.pressure_left[i] = pressure_current.pressure_left[i];
        pressure_prev.pressure_right[i] = pressure_current.pressure_right[i];

	// store the new value as current
	pressure_current.pressure_left[i] = pressure_now.pressure_left[i];
	pressure_current.pressure_right[i] = pressure_now.pressure_right[i];
     }
     // reset our sample counter to zero
     subSample_cnt = 0;

     updated = true;
  }
   
  // increment the subsample counter
  subSample_cnt++;

  return updated;
}


void pressureObserver::getPadCentroid(boost::array<double,22> pressureArray, double *x, double *y)
{
  // weights to multiply our cells on the fingerpad by to determine the x-y centroid
  double y_weights[15] = {-1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1, -1, 0, 1}; 
  double x_weights[15] = {-1, -1, -1, -0.5, -0.5, -0.5, 0, 0, 0, 0.5, 0.5, 0.5, 1, 1, 1};
  
  *y = 0;
  *x = 0;

  for(int i = 7; i < 22; i++)
  {
    *y += pressureArray[i]*y_weights[i-7];
    *x += pressureArray[i]*x_weights[i-7];
  }

  // divide the x and y by the sum of the pad
  double pressureSum = getPadForce(pressureArray);
  *y = *y/pressureSum;
  *x = *x/pressureSum;
}

void pressureObserver::updateXYCentroids()
{
  double x_centroid_left, y_centroid_left, x_centroid_right, y_centroid_right;
  getPadCentroid(pressure_current_zerod.pressure_left,&x_centroid_left,&y_centroid_left);
  getPadCentroid(pressure_current_zerod.pressure_right,&x_centroid_right,&y_centroid_right);

}


bool pressureObserver::checkSlip(double slip_motion_limit, double slip_force_limit)
{
  double disturbance_left =  std::min(std::max(slip_motion_limit*fabs(padForce_left_cur_nonbiased), .048),0.2);
  double disturbance_right = std::min(std::max(slip_motion_limit*fabs(padForce_right_cur_nonbiased), .048),0.2);

  if( ( fabs(padForce_left_cur) > disturbance_left) && (fabs(lfForce_left) < slip_force_limit))
    return true;
  else if( ( fabs(padForce_right_cur) > disturbance_right) && (fabs(lfForce_right) < slip_force_limit))
    return true;
  else
    return false;
}


// constructor
pressureObserver::pressureObserver(pr2_hardware_interface::PressureSensor* left,pr2_hardware_interface::PressureSensor* right)
{
  // store our pressure sensor state handles
  left_finger = left;
  right_finger = right;

  // initialize our subsample counter to 1
  subSample_cnt = 1;

  // set our subsampling up to pick every 41st sample (since we sample at 1ms but the pressure array updates every 41ms)
  subSample_steps = 41;

  // initialize our loading values to zero
  padForce_left_cur = 0.0;
  padForce_right_cur = 0.0;
  padForce_left_prev = 0.0;
  padForce_right_prev = 0.0;

  // initialize our time values
  time_prev = ros::Time::now().toSec();
  time_cur = ros::Time::now().toSec();
  dt = 0.041;

  // intialize the right/left contact states
  left_contact = false;
  right_contact = false;

  // initialize our placement contact state
  placeContact = false;

  // get some fresh data from the sensors
  updatePressureState();

  // set all our zero_offset pressures to the first seen value
  for( int i = 0; i < 22; i ++) 
  {
     zero_offset.pressure_left[i] = pressure_now.pressure_left[i]; 
     zero_offset.pressure_right[i] = pressure_now.pressure_right[i]; 
  }

  // create our filter for low frequency pressure changes
  // instatiate our digital filter
  // 1st order chebychev. 0.8 to 5 hz bandpass. 0.5 r. (3 terms for chebychev?)
  float b_vfilt[] = { 0.6323, 0, -0.6323};
  float a_vfilt[] = {1.0, -0.6294, -0.2647};
  for(int i=0; i < 22; i++)
  {
    pressureLFFilt_left[i] = new digitalFilter(1+1, true,b_vfilt,a_vfilt);
    pressureLFFilt_right[i] = new digitalFilter(1+1, true,b_vfilt,a_vfilt);
  }
}


// destructor
pressureObserver::~pressureObserver()
{
  delete[] pressureLFFilt_left;
  delete[] pressureLFFilt_right;
}
