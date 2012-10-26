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

#ifndef INTERACTIVE_MANIPULATION_FRAME
#define INTERACTIVE_MANIPULATION_FRAME

#include <QWidget>

#include <ros/ros.h>

#include "pr2_interactive_object_detection/UserCommandAction.h"

#include <actionlib/client/simple_action_client.h>

namespace Ui
{
class MainFrame;
}

namespace pr2_interactive_object_detection_frontend {

class InteractiveObjectDetectionFrame : public QWidget
{
Q_OBJECT
public:
  InteractiveObjectDetectionFrame( QWidget* parent = 0 );
  ~InteractiveObjectDetectionFrame();

  void update();

protected Q_SLOTS:
  void segButtonClicked();
  void recButtonClicked();
  void detButtonClicked();
  void cancelButtonClicked();
  void resetButtonClicked();

protected:
  // callbacks for user_cmd_action_client_
  void userCmdDone(const actionlib::SimpleClientGoalState& state,
      const pr2_interactive_object_detection::UserCommandResultConstPtr& result);
  void userCmdActive();
  void userCmdFeedback(const pr2_interactive_object_detection::UserCommandFeedbackConstPtr& feedback);

  // execute a user command in a new thread
  void requestUserCommand( int8_t request, bool interactive );

  // execute a user command (send request to action server). May only be called from spawnUserCommand
  void executeRequest( int8_t request, bool interactive );

  // connection to the back end node which does the actual work
  actionlib::SimpleActionClient<pr2_interactive_object_detection::UserCommandAction> user_cmd_action_client_;

  bool action_requested_;

  //basic system stuff
  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;

  std::string status_;

  std::string seg_status_;
  std::string rec_status_;
  std::string det_status_;

  pr2_interactive_object_detection::UserCommandGoal user_command_goal_;

  boost::mutex mutex_;

  boost::thread *executing_thread_;

private:
  Ui::MainFrame* ui_; // UI object created by uic from main_frame.ui
};


}

#endif
