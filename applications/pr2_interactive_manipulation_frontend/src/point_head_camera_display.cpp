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

#include "pr2_interactive_manipulation/point_head_camera_display.h"

#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/properties/property.h>

#include <pr2_controllers_msgs/PointHeadAction.h>


namespace pr2_interactive_manipulation
{

PointHeadCameraDisplay::PointHeadCameraDisplay():
  rviz::AwesomeCameraDisplay(),
  point_head_view_controller_( 0 )
{
}

void PointHeadCameraDisplay::onInitialize()
{
  AwesomeCameraDisplay::onInitialize();

  //assign a PointHeadViewController to our render panel,
  //so it receives the mouse events while the "Move Camera" tool is active
  point_head_view_controller_ = new PointHeadViewController( texture_, context_ );
  render_panel_->setViewController( point_head_view_controller_ );

  setPointHeadTopic( "/head_traj_controller/point_head_action/goal" );

  createProperties();
}

PointHeadCameraDisplay::~PointHeadCameraDisplay()
{
  //point_head_view_controller_ will be deleted by render_panel_
}

void PointHeadCameraDisplay::createProperties()
{
  point_head_topic_property_ = new rviz::RosTopicProperty("Point Head Action Topic",
                                                        QString::fromStdString(getPointHeadTopic()),
                                                        QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
                                                        QString("Action topic where to send the point head commands to."),
                                                        this,
                                                        SLOT( updatePointHeadTopic() ) );
}

void PointHeadCameraDisplay::setPointHeadTopic(const std::string& topic)
{
  point_head_view_controller_->setTopic( topic );
}

const std::string& PointHeadCameraDisplay::getPointHeadTopic()
{
  return point_head_view_controller_->getTopic();
}

void PointHeadCameraDisplay::updatePointHeadTopic()
{
  point_head_view_controller_->setTopic( point_head_topic_property_->getTopicStd() );
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pr2_interactive_manipulation::PointHeadCameraDisplay,rviz::Display )

