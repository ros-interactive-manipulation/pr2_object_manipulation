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

#include "pr2_interactive_manipulation/publish_click_camera_display.h"

#include <boost/make_shared.hpp>

#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>

namespace pr2_interactive_manipulation
{

PublishClickCameraDisplay::PublishClickCameraDisplay() :
  rviz::AwesomeCameraDisplay(),
  publish_click_view_controller_( 0 )
{
}

void PublishClickCameraDisplay::onInitialize()
{
  AwesomeCameraDisplay::onInitialize();

  //assign a PublishClickViewController to our render panel,
  //so it receives the mouse events while the "Move Camera" tool is active
  publish_click_view_controller_ = new PublishClickViewController( texture_, context_ );
  render_panel_->setViewController( publish_click_view_controller_ );

  publish_click_topic_property_ = new rviz::StringProperty("Publish Click Topic", "/interactive_manipulation_image_click",
      "Topic where click information is published.", 0, SLOT( publishTopicChanged() ) );
  addChild( publish_click_topic_property_, 1 );
  publishTopicChanged();
}

PublishClickCameraDisplay::~PublishClickCameraDisplay()
{
}

void PublishClickCameraDisplay::publishTopicChanged()
{
  std::string topic = publish_click_topic_property_->getStdString();
  publish_click_view_controller_->setTopic( topic );
}

const std::string& PublishClickCameraDisplay::getPublishClickTopic()
{
  return publish_click_view_controller_->getTopic();
}

void PublishClickCameraDisplay::update(float wall_dt, float ros_dt)
{
  AwesomeCameraDisplay::update(wall_dt, ros_dt);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pr2_interactive_manipulation::PublishClickCameraDisplay,rviz::Display )
