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

#ifndef PUBLISH_CLICK_CAMERA_DISPLAY_H_
#define PUBLISH_CLICK_CAMERA_DISPLAY_H_

#include <set>

#include <rviz/properties/string_property.h>
#include <rviz/selection/selection_handler.h>
#include <pr2_interactive_manipulation/awesome_camera_display.h>

#include <object_manipulator/tools/configuration_loader.h>

#include "pr2_interactive_manipulation/publish_click_view_controller.h"

//#include "display_wrapper.h"

namespace pr2_interactive_manipulation
{

//! Extends the default camera display of rviz so that mouse clicks are broadcasted on a given topic
//! Also can hide selected displays so they don't show up / are not selectable.
class PublishClickCameraDisplay: public rviz::AwesomeCameraDisplay
{
Q_OBJECT
public:
  PublishClickCameraDisplay();
  virtual ~PublishClickCameraDisplay();
  virtual void onInitialize();

  const std::string& getPublishClickTopic();

  //! Overrides update so it can request a render at each pass
  virtual void update(float wall_dt, float ros_dt);

public Q_SLOTS:

  void publishTopicChanged();

private:

  void hideBlacklistDisplays(std::string blacklist_name);
  void restoreBlacklistDisplays();

  std::set<rviz::Display*> hidden_displays_;

  rviz::StringProperty* publish_click_topic_property_;

  PublishClickViewController *publish_click_view_controller_;
};

}

#endif 
