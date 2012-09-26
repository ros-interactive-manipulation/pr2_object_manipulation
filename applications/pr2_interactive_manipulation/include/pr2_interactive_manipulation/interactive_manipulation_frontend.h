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

#ifndef INTERACTIVE_MANIPULATION_FRONTEND
#define INTERACTIVE_MANIPULATION_FRONTEND

#include <QWidget>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <std_msgs/String.h>

#include "pr2_interactive_manipulation/advanced_options_dialog.h"
#include "pr2_object_manipulation_msgs/IMGUIOptions.h"
#include "pr2_object_manipulation_msgs/IMGUIAdvancedOptions.h"
#include "pr2_object_manipulation_msgs/IMGUIAction.h"
#include "pr2_object_manipulation_msgs/ActionInfo.h"

namespace Ui
{
class InteractiveManipulation;
}

namespace rviz {
class DisplayContext;
class FrameManager;
}

namespace pr2_interactive_manipulation {

class InteractiveManipulationFrontend : public QWidget
{
Q_OBJECT
public:
  InteractiveManipulationFrontend(rviz::DisplayContext* context, QWidget* parent = 0);

  ~InteractiveManipulationFrontend();

  void update();

  void setAdvancedOptions(pr2_object_manipulation_msgs::IMGUIAdvancedOptions ago){adv_options_ = ago;}

  int interface_number_;

  int task_number_;

protected Q_SLOTS:
  void graspButtonClicked();

  void placeButtonClicked();

  void plannedMoveButtonClicked();

  void cancelButtonClicked();

  void stopNavButtonClicked();

  void centerHeadButtonClicked();

  void drawReachableZonesButtonClicked();

  void resetButtonClicked();

  void armGoButtonClicked();

  void cameraLeftButtonClicked();

  void cameraRightButtonClicked();

  void cameraFrontButtonClicked();

  void cameraTopButtonClicked();

  void advancedOptionsClicked();

  void modelObjectClicked();

  //void openDrawerButtonClicked();

  void rcommandRunButtonClicked();

  void rcommandRefreshButtonClicked();

  void gripperSliderScrollChanged();
 
protected:

  void feedbackCallback(const pr2_object_manipulation_msgs::IMGUIFeedbackConstPtr &feedback);

  void statusCallback( const std_msgs::StringConstPtr &status);

  pr2_object_manipulation_msgs::IMGUIOptions getDialogOptions();

  void setCamera(std::vector<double> params);

  rviz::DisplayContext* context_;

  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;

  ros::ServiceClient rcommander_action_info_client_;
  actionlib::SimpleActionClient<pr2_object_manipulation_msgs::IMGUIAction> *action_client_;
  //actionlib::SimpleActionClient<RunScriptAction> *scripted_action_client_;
  //std::string scripted_action_name_;
  ros::Subscriber status_sub_;
  ros::Publisher draw_reachable_zones_pub_;

  std::string rcommander_action_info_name_;
  std::string rcommander_group_name_;
  std::string action_name_;
  std::string status_name_;

  std::string status_label_text_;

  boost::mutex status_label_mutex_;

  pr2_object_manipulation_msgs::IMGUIAdvancedOptions adv_options_;

private:
  Ui::InteractiveManipulation* ui_;
};

}

#endif
