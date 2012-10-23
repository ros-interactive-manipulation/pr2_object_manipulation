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

#include "pr2_interactive_manipulation/interactive_marker_node.h"

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

using namespace visualization_msgs;
using namespace interactive_markers;

namespace pr2_interactive_manipulation
{

typedef actionlib::SimpleActionClient<pr2_object_manipulation_msgs::IMGUIAction> IMGUIActionClient;

InteractiveMarkerNode::InteractiveMarkerNode() :
  root_nh_(""),
  priv_nh_("~"),
  im_gui_action_client_("imgui_action", true),
  marker_server_("interactive_manipulation")
{
  graspable_objects_sub_ = root_nh_.subscribe("interactive_object_recognition_result", 10,
    &InteractiveMarkerNode::processGraspableObjects, this);

  options_.collision_checked = true;
  options_.grasp_selection = 1;

  options_.adv_options.lift_steps = 10;
  options_.adv_options.retreat_steps = 10;
  options_.adv_options.reactive_force = false;
  options_.adv_options.reactive_grasping = false;
  options_.adv_options.reactive_place = false;
  options_.adv_options.lift_direction_choice = 0;

  dyn_conf_srv_.setCallback( boost::bind(&InteractiveMarkerNode::processConfig, this, _1, _2) );
  collision_map_interface_.resetCollisionId(1000);

}

InteractiveMarkerNode::~InteractiveMarkerNode()
{
}

void InteractiveMarkerNode::processConfig(PickupConfig &config, uint32_t level)
{
  options_.adv_options.reactive_grasping = config.reactive_grasping;
  options_.adv_options.lift_steps = config.lift_steps;
  if (config.lift_vertically) options_.adv_options.lift_direction_choice = 0;
  else options_.adv_options.lift_direction_choice = 1;
}

void InteractiveMarkerNode::processGraspableObjects(
                                          const object_manipulation_msgs::GraspableObjectListConstPtr &objects)
{
  object_handlers_.clear();

  if ( objects->meshes.size() != objects->graspable_objects.size() )
  {
    ROS_ERROR( "Number of meshes an graspable objects is not identical!" );
    return;
  }

  for ( unsigned o=0; o<objects->meshes.size() && o<objects->graspable_objects.size(); o++ )
  {
    char name[255];
    sprintf( name, "object_%d", o );

    object_handlers_.push_back( boost::shared_ptr<GraspableObjectHandler>( new GraspableObjectHandler( name, this, objects->graspable_objects[o], objects->meshes[o], marker_server_, options_, &collision_map_interface_ ) ) );
  }
  marker_server_.applyChanges();
}

std::vector<object_manipulation_msgs::GraspableObject> InteractiveMarkerNode::getMovableObstacles()
{
  std::vector<object_manipulation_msgs::GraspableObject> movable_obstacles;
  for (size_t i=0; i<object_handlers_.size(); i++)
  {
    if (!object_handlers_[i]->getCollisionName().empty())
    {
      movable_obstacles.push_back(object_handlers_[i]->getGraspableObject());
    }    
  }
  return movable_obstacles;
}

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "interactive_manipulation_marker_node");
  pr2_interactive_manipulation::InteractiveMarkerNode node;
  ros::spin();
  return 0;
}

