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

#ifndef INTERACTIVE_MANIPULATION_MARKER_NODE
#define INTERACTIVE_MANIPULATION_MARKER_NODE

#include "pr2_object_manipulation_msgs/IMGUIAction.h"
#include "pr2_object_manipulation_msgs/PickupIMObjectAction.h"

#include "graspable_object_handler.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <manipulation_msgs/GraspableObject.h>

#include <manipulation_msgs/GraspableObjectList.h>

#include <pr2_interactive_manipulation/PickupConfig.h>

#include <tabletop_collision_map_processing/collision_map_interface.h>

namespace pr2_interactive_manipulation
{

class InteractiveMarkerNode
{

public:
  InteractiveMarkerNode();
  ~InteractiveMarkerNode();

  std::vector<manipulation_msgs::GraspableObject> getMovableObstacles();

private:

  void processGraspableObjects(const manipulation_msgs::GraspableObjectListConstPtr &objects);

  void processConfig(PickupConfig &config, uint32_t level);

  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber graspable_objects_sub_;

  actionlib::SimpleActionClient<pr2_object_manipulation_msgs::IMGUIAction> im_gui_action_client_;

  interactive_markers::InteractiveMarkerServer marker_server_;

  std::vector< boost::shared_ptr<GraspableObjectHandler> > object_handlers_;

  pr2_object_manipulation_msgs::IMGUIOptions options_;

  dynamic_reconfigure::Server<PickupConfig> dyn_conf_srv_;

  tabletop_collision_map_processing::CollisionMapInterface collision_map_interface_;

  actionlib::SimpleActionServer<pr2_object_manipulation_msgs::PickupIMObjectAction> pickup_as_;

  void pickupIMObject(const pr2_object_manipulation_msgs::PickupIMObjectGoalConstPtr &goal);
};

}

#endif
