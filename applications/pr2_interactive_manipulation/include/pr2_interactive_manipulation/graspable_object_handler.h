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

#ifndef INTERACTIVE_MANIPULATION_GRASPABLE_OBJECT_HANDLER
#define INTERACTIVE_MANIPULATION_GRASPABLE_OBJECT_HANDLER

#include "pr2_object_manipulation_msgs/IMGUIAction.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <pr2_interactive_object_detection/GraspableObjectList.h>

#include <tabletop_collision_map_processing/collision_map_interface.h>

namespace pr2_interactive_manipulation
{

class InteractiveMarkerNode;

class GraspableObjectHandler
{
public:

  GraspableObjectHandler( std::string name,
      InteractiveMarkerNode *node,
      const object_manipulation_msgs::GraspableObject &object,
      const arm_navigation_msgs::Shape &mesh,
      interactive_markers::InteractiveMarkerServer &marker_server,
      pr2_object_manipulation_msgs::IMGUIOptions &options,
      tabletop_collision_map_processing::CollisionMapInterface *col);

  ~GraspableObjectHandler();

  object_manipulation_msgs::GraspableObject getGraspableObject() {return object_;}

  std::string getCollisionName() {return collision_object_name_;}

private:

  typedef interactive_markers::MenuHandler MenuHandler;

  void makeMenu();
  void updateMenu();

  void pickup( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void addCollisionObject( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void makeMarker();

  void switchFlag( bool *value, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  interactive_markers::InteractiveMarkerServer &marker_server_;
  actionlib::SimpleActionClient<pr2_object_manipulation_msgs::IMGUIAction> im_gui_action_client_;

  pr2_object_manipulation_msgs::IMGUIOptions &options_;

  object_manipulation_msgs::GraspableObject object_;
  const arm_navigation_msgs::Shape mesh_;

  bool use_rec_result_;

  bool picking_;

  tabletop_collision_map_processing::CollisionMapInterface *collision_map_interface_;

  interactive_markers::MenuHandler menu_handler_;

  MenuHandler::EntryHandle left_pickup_h_;
  MenuHandler::EntryHandle right_pickup_h_;
  MenuHandler::EntryHandle add_collision_object_h_;
  MenuHandler::EntryHandle use_rec_result_h_;

  std::string name_;

  std::string collision_object_name_;

  InteractiveMarkerNode* interactive_marker_node_;
};

}

#endif
