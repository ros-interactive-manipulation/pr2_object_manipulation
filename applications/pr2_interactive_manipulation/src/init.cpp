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

#include <pluginlib/class_list_macros.h>

// aleeper: Why does this file exist? Why isn't each plugin declared in its own file?


#include "pr2_interactive_manipulation/interactive_manipulation_frontend_display.h"
#include "pr2_interactive_manipulation/publish_click_camera_display.h"
#include "pr2_interactive_manipulation/point_head_camera_display.h"
#include "pr2_interactive_manipulation/camera_focus_frontend.h"


PLUGINLIB_DECLARE_CLASS( pr2_interactive_manipulation, InteractiveManipulationFrontend, pr2_interactive_manipulation::InteractiveManipulationFrontendDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( pr2_interactive_manipulation, PointHeadCamera, pr2_interactive_manipulation::PointHeadCameraDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( pr2_interactive_manipulation, PublishClickCamera, pr2_interactive_manipulation::PublishClickCameraDisplay, rviz::Display )
PLUGINLIB_DECLARE_CLASS( pr2_interactive_manipulation, CameraFocusFrontend, pr2_interactive_manipulation::CameraFocusFrontend, rviz::Display )
