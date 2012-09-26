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

#ifndef PUBLISH_CLICK_VIEW_CONTROLLER_H_
#define PUBLISH_CLICK_VIEW_CONTROLLER_H_

#include <ros/ros.h>

#include <rviz/view_controller.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/viewport_mouse_event.h>

#include <OGRE/OgreCamera.h>

namespace pr2_interactive_manipulation
{

// View controller that, when the image is clicked, published info about the click on a given topic
class PublishClickViewController : public rviz::ViewController
{
public:
  PublishClickViewController( rviz::ROSImageTexture &texture, 
      rviz::DisplayContext* context);

  virtual ~PublishClickViewController();

  virtual void handleMouseEvent(rviz::ViewportMouseEvent& evt);

  //no need to serialize this class, as it stores no data
  virtual void fromString(const std::string& str) {};
  virtual std::string toString() { return ""; }

  virtual std::string getClassName() { return "pr2_interactive_manipulation::PublishClickViewController"; }

  const std::string& getTopic() { return topic_; }
  void setTopic(const std::string& topic);

  // Since this view controller does not actually control the view,
  // these functions which change the pose or direction of the view
  // are no-ops: lookAt() and reset().
  virtual void lookAt( const Ogre::Vector3& point ) {}
  virtual void reset() {}

protected:
  virtual void onActivate() {};
  virtual void onDeactivate() {};
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, 
                                    const Ogre::Quaternion& old_reference_orientation) {};

  ros::NodeHandle root_nh_;
  rviz::ROSImageTexture &texture_;
  std::string topic_;
  ros::Publisher click_pub_;
};

}

#endif 
