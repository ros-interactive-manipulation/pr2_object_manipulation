/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include <rviz/default_plugin/tools/point_tool.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/viewport_mouse_event.h>

namespace pr2_interactive_manipulation
{

class LookAtTool: public rviz::PointTool
{
public:
  LookAtTool();
};




class SetGripperTool: public rviz::PointTool
{
Q_OBJECT

public:
  SetGripperTool();
  virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

public Q_SLOTS:
  virtual void updatePoseTopic();

protected:
  void createProperties();

  //! number of pixels around point to sample for depth image
  rviz::IntProperty *normal_box_padding_;

  //! Maximum far distance for depth image
  rviz::FloatProperty *far_clip_distance_;

  //! Name of topic on which to publish the full pose.
  rviz::StringProperty * topic_property_pose_;

  //! Publisher for pose.
  ros::Publisher pub_pose_;

};

class NavigateToTool: public rviz::PointTool
{
public:
  NavigateToTool();
};

}
