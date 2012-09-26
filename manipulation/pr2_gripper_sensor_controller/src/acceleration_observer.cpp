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
//@brief   acceleration_observer.cpp - file to read the hand mounted 
//         accelerometer in the pr2 and do some simple processing to 
//         make the data more accessible to higher-level code

#include <pr2_gripper_sensor_controller/acceleration_observer.h>

// method to be called to check if we saw  a contact acceleration
bool accelerationObserver::checkPlaceContact(double dAcc)
{
  //if(( fabs(aX) > dAcc) || (fabs(aY) > dAcc)  || (fabs(aZ) > dAcc)|| placeContact )
  if( sqrt(aX_bp*aX_bp + aY_bp*aY_bp + aZ_bp*aZ_bp) > dAcc || placeContact)
    placeContact = true;
  else
    placeContact = false;
  return placeContact;
}

// method to be called each 1ms timestep in realtime to ensure data collection and manipulation happens properly
void accelerationObserver::spin()
{
  // retrieve and filter our acceleration data
  std::vector<geometry_msgs::Vector3> threeAccs = accHandle->state_.samples_;
  for( uint  i = 0; i < threeAccs.size(); i++ )  
  {
    aX_bp = accBPFilt[0]->getNextFilteredValue(threeAccs[i].x);
    aY_bp = accBPFilt[1]->getNextFilteredValue(threeAccs[i].y);
    aZ_bp = accBPFilt[2]->getNextFilteredValue(threeAccs[i].z);

    aX_lp = accLPFilt[0]->getNextFilteredValue(threeAccs[i].x);
    aY_lp = accLPFilt[1]->getNextFilteredValue(threeAccs[i].y);
    aZ_lp = accLPFilt[2]->getNextFilteredValue(threeAccs[i].z);
    readingTime = ros::Time::now().toSec();
  }    

}

// constructor
accelerationObserver::accelerationObserver(pr2_hardware_interface::Accelerometer* accelerometerHandle)
{
  accHandle = accelerometerHandle;

  // place contact flag
  placeContact = false;

  // instatiate our acceleration values
  aX_lp = 0;
  aY_lp = 0;
  aZ_lp = 0;
  aX_bp = 0;
  aY_bp = 0;
  aZ_bp = 0;

  // make sure the accelerometer has a 1.5 khz bandwidth          
  accHandle->command_.bandwidth_ = 6;

  // make sure the accelerometer has a +/- 8g range (0 = 2g, 1 =  4g)         
  accHandle->command_.range_ = 2;

  
  // create our filter for band-passed accelerometer data
  // 1st order chebychev. band-pass 5-1000 hz
  float b_bpfilt[] = {0.8305, 0, -0.8305};
  float a_bpfilt[] = {1.0,-0.3329,-0.6610};
  for(int i=0; i < 3; i++)
    accBPFilt[i] = new digitalFilter(2, true,b_bpfilt,a_bpfilt);

  // create our filter for low-passed accelerometer data
  // 1st order butterworth. low-pass 1000 hz
  float b_lpfilt[] = {0.634, 0.634};
  float a_lpfilt[] = {1.0, 0.2679};
  for(int i=0; i < 3; i++)
    accLPFilt[i] = new digitalFilter(1, true,b_lpfilt,a_lpfilt);
  
}


// destructor
accelerationObserver::~accelerationObserver()
{
  delete[] accBPFilt;
  delete[] accLPFilt;
}

