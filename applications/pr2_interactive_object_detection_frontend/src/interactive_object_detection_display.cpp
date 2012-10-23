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

#include "pr2_interactive_object_detection_frontend/interactive_object_detection_display.h"

#include <rviz/window_manager_interface.h>
#include <rviz/visualization_manager.h>
#include <rviz/panel_dock_widget.h>

#include "pr2_interactive_object_detection_frontend/interactive_object_detection_frame.h"

#include <QDockWidget>

namespace pr2_interactive_object_detection_frontend {

InteractiveObjectDetectionDisplay::InteractiveObjectDetectionDisplay()
  : Display()
  , frame_dock_( 0 )
{
}

void InteractiveObjectDetectionDisplay::onInitialize()
{
  window_manager_ = context_->getWindowManager();
  ROS_ASSERT(window_manager_);
  QWidget* parent = window_manager_->getParentWindow();
  ROS_ASSERT(parent);

  frame_ = new InteractiveObjectDetectionFrame( parent );
  frame_dock_ = window_manager_->addPane( "Object Detection", frame_ );
}
  
InteractiveObjectDetectionDisplay::~InteractiveObjectDetectionDisplay()
{
  delete frame_dock_;
}

void InteractiveObjectDetectionDisplay::onEnable()
{
  frame_dock_->show();
}

void InteractiveObjectDetectionDisplay::onDisable()
{
  frame_dock_->hide();
}

void InteractiveObjectDetectionDisplay::update(float, float)
{
  frame_->update();
}

}
