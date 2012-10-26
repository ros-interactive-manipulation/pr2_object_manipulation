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

#ifndef INTERACTIVE_MANIPULATION_DISPLAY_H
#define INTERACTIVE_MANIPULATION_DISPLAY_H

#include "rviz/display.h"

namespace rviz {
class VisualizationManager;
class WindowManagerInterface;
class PanelDockWidget;
}

namespace pr2_interactive_object_detection_frontend
{

class InteractiveObjectDetectionFrame;

class InteractiveObjectDetectionDisplay : public rviz::Display
{
public:
  InteractiveObjectDetectionDisplay();
  
  virtual ~InteractiveObjectDetectionDisplay();

  virtual void onInitialize();

  virtual void update(float wall_dt, float ros_dt);

protected:

  virtual void onEnable();

  virtual void onDisable();

  std::string name_;
  
  rviz::WindowManagerInterface* window_manager_;

  InteractiveObjectDetectionFrame *frame_;

  rviz::PanelDockWidget* frame_dock_; // The dock widget containing the frame.
};

}

#endif
