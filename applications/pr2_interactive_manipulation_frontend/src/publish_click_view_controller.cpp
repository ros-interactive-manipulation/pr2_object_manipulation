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

#include "pr2_interactive_manipulation/publish_click_view_controller.h"

#include <rviz/display_context.h>
#include <OGRE/OgreSceneManager.h>

#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreVector3.h>

#include <pr2_object_manipulation_msgs/ImageClick.h>

namespace pr2_interactive_manipulation
{

PublishClickViewController::PublishClickViewController( rviz::ROSImageTexture &texture, 
                                                        rviz::DisplayContext* context) :
    rviz::ViewController( ),
    root_nh_(""),
    texture_(texture)
{
  ros::NodeHandle n;
  initialize( context );
}

PublishClickViewController::~PublishClickViewController()
{
}

void PublishClickViewController::handleMouseEvent( rviz::ViewportMouseEvent& event)
{
  setStatus( "" );
  setCursor( Crosshair );


  if ( event.leftDown() )
  {
    if ( !texture_.getImage().get() )
    {
      ROS_ERROR( "No image received. Cannot compute look-at point." );
      return;
    }
    int width = event.viewport->getActualWidth();
    int height = event.viewport->getActualHeight();
    Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
        (float)event.x / (float)width, (float)event.y / (float)height );
    Ogre::Vector3 origin = mouse_ray.getOrigin();
    Ogre::Vector3 direction = mouse_ray.getDirection();
    pr2_object_manipulation_msgs::Ray ray;
    ray.header.stamp = ros::Time::now();
    ray.header.frame_id = context_->getFixedFrame().toStdString();
    ray.origin.x = origin.x;
    ray.origin.y = origin.y;
    ray.origin.z = origin.z;
    ray.direction.x = direction.x;
    ray.direction.y = direction.y;
    ray.direction.z = direction.z;
    pr2_object_manipulation_msgs::ImageClick click;
    click.ray = ray;
    click.camera_frame_id = texture_.getImage()->header.frame_id;
    click_pub_.publish(click);
  }
}

void PublishClickViewController::setTopic(const std::string& topic)
{
  topic_ = topic;
  click_pub_ = root_nh_.advertise<pr2_object_manipulation_msgs::ImageClick>(topic, 1);
}

}
