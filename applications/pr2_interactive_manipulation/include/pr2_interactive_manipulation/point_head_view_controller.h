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

#ifndef POINT_HEAD_VIEW_CONTROLLER_H_
#define POINT_HEAD_VIEW_CONTROLLER_H_

#include <ros/ros.h>

#include <pr2_controllers_msgs/PointHeadAction.h>

#include <actionlib/client/simple_action_client.h>

#include <rviz/view_controller.h>
#include <rviz/image/ros_image_texture.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>

#include <OGRE/OgreCamera.h>

#define DEBUG_DISPLAY

namespace pr2_interactive_manipulation
{

// View controller that, instead of controlling the camera,
// makes the robot point it's head (approximately) in the direction of a mouse click
// by computing a look-at point 1m in front of the camera.
// note: this will only work in a camera display of one of the head cameras
class PointHeadViewController : public rviz::ViewController
{
public:
  PointHeadViewController( rviz::ROSImageTexture &texture, rviz::DisplayContext* context);
  virtual ~PointHeadViewController();

  virtual void handleMouseEvent(rviz::ViewportMouseEvent& evt);

  //no need to serialize this class, as it stores no data
  virtual void fromString(const std::string& str) {};
  virtual std::string toString() { return ""; }

  virtual void lookAt( const Ogre::Vector3& point );
  virtual std::string getClassName() { return "pr2_interactive_manipulation::PointHeadViewController"; }

  const std::string& getTopic() { return topic_; }
  void setTopic(const std::string& topic);

  /** Reset the view controller to some sane initial state, like
   * looking at 0,0,0 of the target frame. */
  virtual void reset();

protected:
  virtual void onActivate() {};
  virtual void onDeactivate() {};
  virtual void onTargetFrameChanged(const Ogre::Vector3& old_reference_position, 
                                    const Ogre::Quaternion& old_reference_orientation) {};

#ifdef DEBUG_DISPLAY
  ros::Publisher marker_pub_;
#endif

  rviz::ROSImageTexture &texture_;

  typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadActionClient;
  PointHeadActionClient *point_head_action_client_;

  std::string topic_;
};

}

#endif /* POINT_HEAD_VIEW_CONTROLLER_H_ */
