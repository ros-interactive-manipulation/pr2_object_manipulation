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

#ifndef CAMERA_FOCUS_FRONTEND
#define CAMERA_FOCUS_FRONTEND

#include <rviz_view_controllers/CameraPlacement.h>
#include <rviz/display.h>
#include <ros/ros.h>

#include <QWidget>
#include <QString>

class QToolBar;
class QActionGroup;

namespace rviz {
class RosTopicProperty;
class Display;
}

namespace pr2_interactive_manipulation
{

class CameraFocusFrontend : public rviz::Display
{
Q_OBJECT
public:
  CameraFocusFrontend();
  ~CameraFocusFrontend();

  virtual void onInitialize();

  void addFocusButton( QString id );

protected Q_SLOTS:

  void updateTopic();

  void changeView( QAction* action );

protected:

  virtual void onEnable();
  virtual void onDisable();

  void processButtonClick(const std::string &name);

  ros::NodeHandle root_nh_;
  ros::Publisher pub_;
  rviz::RosTopicProperty* topic_prop_;
  std::string topic_;

private:
  QToolBar* toolbar_;
  QActionGroup* toolbar_actions_;
};

} //namespace

#endif
