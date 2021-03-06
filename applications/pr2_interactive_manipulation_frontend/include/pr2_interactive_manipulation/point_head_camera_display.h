/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef POINT_HEAD_CAMERA_DISPLAY_H_
#define POINT_HEAD_CAMERA_DISPLAY_H_

#include <QObject>

#include "pr2_interactive_manipulation/awesome_camera_display.h"

#include "pr2_interactive_manipulation/point_head_view_controller.h"

#include <rviz/properties/ros_topic_property.h>

namespace pr2_interactive_manipulation
{

// extends the default camera display of rviz so that
// the user can control the robot's head by clicking with the mouse
class PointHeadCameraDisplay: public rviz::AwesomeCameraDisplay
{
Q_OBJECT
public:
  PointHeadCameraDisplay();
  virtual ~PointHeadCameraDisplay();
  virtual void onInitialize();

  virtual void createProperties();

  const std::string& getPointHeadTopic();
  void setPointHeadTopic(const std::string& topic);

private Q_SLOTS:
  void updatePointHeadTopic();

private:

  rviz::RosTopicProperty* point_head_topic_property_;

  PointHeadViewController *point_head_view_controller_;
};

}

#endif /* POINT_HEAD_CAMERA_DISPLAY_H_ */
