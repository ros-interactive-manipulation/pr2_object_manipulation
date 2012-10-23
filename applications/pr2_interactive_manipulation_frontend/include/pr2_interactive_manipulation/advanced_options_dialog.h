/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ADVANCED_OPTIONS_DIALOG
#define ADVANCED_OPTIONS_DIALOG

#include <QDialog>

#include <ros/ros.h>

namespace Ui
{
  class AdvancedOptionsDialog;
}

#include "pr2_object_manipulation_msgs/IMGUIAdvancedOptions.h"

namespace pr2_interactive_manipulation {

class InteractiveManipulationFrontend;

struct AdvancedGraspOptions
{
  bool reactive_grasping_;
  bool reactive_force_;
  bool reactive_place_;
  int lift_steps_;
  int retreat_steps_;
  int lift_direction_choice_;
  int desired_approach_;
  int min_approach_;
  float max_contact_force_;
  bool find_alternatives_;
  bool always_plan_grasps_;
  bool cycle_gripper_opening_;
};
  
class AdvancedOptionsDialog : public QDialog
{
Q_OBJECT
 private:
  InteractiveManipulationFrontend *frontend_;

  Ui::AdvancedOptionsDialog* ui_;
                                
  ros::NodeHandle root_nh_;

 private Q_SLOTS:
  virtual void setDefaultsClicked();
  virtual void acceptClicked();

  virtual void reactiveGraspingClicked();
  virtual void reactiveForceClicked();

  virtual void findAlternativesClicked();
  virtual void alwaysPlanGraspsClicked();
  virtual void cycleGripperOpeningClicked();

 public:
  AdvancedOptionsDialog(InteractiveManipulationFrontend *frontend);
  ~AdvancedOptionsDialog();

  void setOptions(AdvancedGraspOptions go);
  static AdvancedGraspOptions getDefaults(int interface_number = 0, int task_number = 0);
  AdvancedGraspOptions getOptions();

  void setOptionsMsg(pr2_object_manipulation_msgs::IMGUIAdvancedOptions go);
  static pr2_object_manipulation_msgs::IMGUIAdvancedOptions getDefaultsMsg(int interface_number = 0, int task_number = 0);
  pr2_object_manipulation_msgs::IMGUIAdvancedOptions getOptionsMsg();
};

}

#endif
