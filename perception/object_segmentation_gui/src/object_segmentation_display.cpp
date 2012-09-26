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

#include "object_segmentation_gui/object_segmentation_display.h"
#include "object_segmentation_gui/object_segmentation_rviz_ui.h"

#include <rviz/window_manager_interface.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>

#include <OGRE/OgreRenderWindow.h>

namespace object_segmentation_gui {

ObjectSegmentationDisplay::ObjectSegmentationDisplay()
  : Display()
  , object_segmentation_ui_(0)
{
}

ObjectSegmentationDisplay::~ObjectSegmentationDisplay()
{
  delete object_segmentation_ui_;
}

void ObjectSegmentationDisplay::onEnable()
{
  if ( !object_segmentation_ui_ )
  {
    ROS_INFO( "Creating UI" );
    object_segmentation_ui_ = new ObjectSegmentationRvizUI( context_ );
  }
  object_segmentation_ui_->startActionServer( update_nh_ );
}

void ObjectSegmentationDisplay::onDisable()
{
  object_segmentation_ui_->stopActionServer();
}

void ObjectSegmentationDisplay::update(float wall_dt, float ros_dt)
{
  object_segmentation_ui_->update(wall_dt, ros_dt);
}

void ObjectSegmentationDisplay::createProperties()
{
}
  
}
