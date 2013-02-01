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

#include "pr2_interactive_manipulation/interactive_manipulation_frontend_display.h"

#include <rviz/display_context.h>
#include <rviz/panel_dock_widget.h>
#include <rviz/window_manager_interface.h>

#include "pr2_interactive_manipulation/interactive_manipulation_frontend.h"

#include <QDockWidget>

namespace pr2_interactive_manipulation {

InteractiveManipulationFrontendDisplay::InteractiveManipulationFrontendDisplay() : 
  Display(),
  frame_(0),
  frame_dock_(0)
{
}
  
InteractiveManipulationFrontendDisplay::~InteractiveManipulationFrontendDisplay()
{
  delete frame_dock_;
}

void InteractiveManipulationFrontendDisplay::onInitialize()
{
  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  ROS_ASSERT(window_context);
  frame_ = new InteractiveManipulationFrontend(context_, window_context->getParentWindow());
  frame_dock_ = window_context->addPane( "Manipulation", frame_ );
}

void InteractiveManipulationFrontendDisplay::onEnable()
{
  frame_dock_->show();
}

void InteractiveManipulationFrontendDisplay::onDisable()
{
  if ( frame_dock_ ) frame_dock_->hide();
}

void InteractiveManipulationFrontendDisplay::update(float, float)
{
  if ( frame_ ) frame_->update();
}

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pr2_interactive_manipulation::InteractiveManipulationFrontendDisplay,rviz::Display )
