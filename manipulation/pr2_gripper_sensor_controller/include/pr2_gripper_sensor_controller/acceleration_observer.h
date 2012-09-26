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

#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_gripper_sensor_controller/digitalFilter.h>

#ifndef _ACCELERATION_OBSERVER
#define _ACCELERATION_OBSERVER


class accelerationObserver{
  
  public:
    // functions
    accelerationObserver(pr2_hardware_interface::Accelerometer* accelerometerHandle);  // ctor
    ~accelerationObserver();                    // default destructor
    void spin();                                // method to update sensor data. should run at 1khz
    bool checkPlaceContact(double dAcc);
    double aX_bp, aY_bp, aZ_bp;                 // bandpassed acceleration data                    
    double aX_lp, aY_lp, aZ_lp;                 // lowpassed acceleration data
    double readingTime;

    digitalFilter *accBPFilt[3];
    digitalFilter *accLPFilt[3];
    bool placeContact;

  private:
    pr2_hardware_interface::Accelerometer* accHandle;

};

#endif // _ACCELERATION_OBSERVER
