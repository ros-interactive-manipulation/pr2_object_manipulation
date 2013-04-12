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

#include "pr2_interactive_manipulation/graspable_object_handler.h"

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "pr2_interactive_manipulation/interactive_marker_node.h"

using namespace visualization_msgs;
using namespace interactive_markers;

namespace pr2_interactive_manipulation
{

  GraspableObjectHandler::GraspableObjectHandler( std::string name,
      InteractiveMarkerNode *node,
      const manipulation_msgs::GraspableObject &object,
      const shape_msgs::Mesh &mesh,
      interactive_markers::InteractiveMarkerServer &marker_server,
      pr2_object_manipulation_msgs::IMGUIOptions &options,
      tabletop_collision_map_processing::CollisionMapInterface *col ) :
    marker_server_(marker_server),
    im_gui_action_client_("imgui_action", false),
    options_(options),
    object_(object),
    mesh_(mesh),
    use_rec_result_(true),
    collision_map_interface_(col),
    use_rec_result_h_(0),
    name_(name),
    interactive_marker_node_(node)
  {
    makeMenu();
    makeMarker();
  }

  GraspableObjectHandler::~GraspableObjectHandler()
  {
    marker_server_.erase(name_);
  }

  void GraspableObjectHandler::makeMenu()
  {
    // create menu
    MenuHandler::EntryHandle pickup_h = menu_handler_.insert( "Pick Up" );
    left_pickup_h_ = menu_handler_.insert( pickup_h, "Using Left Arm", 
        boost::bind( &GraspableObjectHandler::pickup, this, _1) );
    right_pickup_h_ = menu_handler_.insert( pickup_h, "Using Right Arm", 
        boost::bind( &GraspableObjectHandler::pickup, this, _1) );

    add_collision_object_h_ = menu_handler_.insert("Add Collision Object", 
        boost::bind( &GraspableObjectHandler::addCollisionObject, this, _1));

    // only add choice if there is actually one
    if ( mesh_.triangles.size() > 0 && object_.cluster.points.size() > 0 )
    {
      use_rec_result_h_ = menu_handler_.insert( "Use Recognition Result",
          boost::bind( &GraspableObjectHandler::switchFlag, this, &use_rec_result_, _1) );
    }

    menu_handler_.insert( "Advanced options..",
        visualization_msgs::MenuEntry::ROSRUN, 
        "dynamic_reconfigure reconfigure_gui " + ros::this_node::getName() );

    updateMenu();
  }

  void GraspableObjectHandler::updateMenu()
  {
    if ( use_rec_result_h_ )
    {
      menu_handler_.setCheckState( use_rec_result_h_, use_rec_result_ ? MenuHandler::CHECKED : MenuHandler::UNCHECKED );
    }

    menu_handler_.reApply( marker_server_ );
    marker_server_.applyChanges();
  }

  void GraspableObjectHandler::addCollisionObject( const InteractiveMarkerFeedbackConstPtr &feedback )
  {
    ros::Time start_time = ros::Time::now();
    while (!collision_map_interface_->connectionsEstablished(ros::Duration(1.0)) )
    {
      ROS_INFO("Waiting for collision map services");
      if (ros::Time::now() - start_time >= ros::Duration(5.0))
      {
        ROS_ERROR("collision map services not found");
        return;
      }
    }
    try
    {
      if (!object_.potential_models.empty())
      {
        collision_map_interface_->processCollisionGeometryForObject(object_.potential_models[0], collision_object_name_);
      }
      else
      {
        object_manipulation_msgs::ClusterBoundingBox bbox;
        collision_map_interface_->getClusterBoundingBox(object_.cluster, bbox.pose_stamped, bbox.dimensions);
        collision_map_interface_->processCollisionGeometryForBoundingBox(bbox, collision_object_name_);
      }
    }
    catch (tabletop_collision_map_processing::CollisionMapException &ex)
    {
      ROS_ERROR_STREAM("Exception adding object to collision map: " << ex.what());
      return;
    }
    ROS_INFO_STREAM("Object added to collision map as " << collision_object_name_);
    object_.collision_name = collision_object_name_;
  }

  void GraspableObjectHandler::pickup( const InteractiveMarkerFeedbackConstPtr &feedback )
  {
    // pick up
    ROS_INFO_STREAM( "Picking up object # " << feedback->marker_name );

    // determine arm selection
    MenuHandler::EntryHandle arm_h = feedback->menu_entry_id;
    if ( arm_h == left_pickup_h_ ) callPickup(1);
    else if ( arm_h == right_pickup_h_ ) callPickup(0);
  }

  //arm_selection: 0=right, 1=left
  void GraspableObjectHandler::callPickup(int arm_selection)
  {
    options_.arm_selection = arm_selection;
    manipulation_msgs::GraspableObject object = object_;

    if ( !use_rec_result_ )
    {
      object.potential_models.clear();
    }

    pr2_object_manipulation_msgs::IMGUIGoal goal;
    goal.options = options_;
    goal.options.grasp_selection = 1;
    goal.options.selected_object = object;
    goal.options.movable_obstacles = interactive_marker_node_->getMovableObstacles();
    goal.command.command = goal.command.PICKUP;

    im_gui_action_client_.sendGoal(goal);

    marker_server_.clear();
    marker_server_.applyChanges();
  }


  void GraspableObjectHandler::makeMarker()
  {
    InteractiveMarker int_marker;
    int_marker.name = name_;

    Marker marker;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    marker.frame_locked = false;

    if ( use_rec_result_ && mesh_.vertices.size() > 0 )
    {
      int_marker.pose = object_.potential_models[0].pose.pose;
      int_marker.header = object_.potential_models[0].pose.header;

      marker.type = Marker::TRIANGLE_LIST;

      ROS_INFO_STREAM( "Adding mesh. #triangles=" << mesh_.triangles.size() );

      marker.points.reserve( mesh_.triangles.size()*3 );

      for ( size_t t=0; t<mesh_.triangles.size(); t++ )
      {
        if ( (size_t)mesh_.triangles[t].vertex_indices[0] < mesh_.vertices.size() &&
             (size_t)mesh_.triangles[t].vertex_indices[1] < mesh_.vertices.size() &&
             (size_t)mesh_.triangles[t].vertex_indices[2] < mesh_.vertices.size() )
        {
          marker.points.push_back( mesh_.vertices[ mesh_.triangles[t].vertex_indices[0] ] );
          marker.points.push_back( mesh_.vertices[ mesh_.triangles[t].vertex_indices[1] ] );
          marker.points.push_back( mesh_.vertices[ mesh_.triangles[t].vertex_indices[2] ] );
        }
        else
        {
          ROS_ERROR("Mesh contains invalid triangle!");
          break;
        }
      }
    }
    else
    {
      int_marker.header = object_.cluster.header;
      marker.scale.x = 0.01;
      marker.scale.y = 0.01;
      marker.scale.z = 0.01;
      marker.type = Marker::CUBE_LIST;

      marker.points.resize( object_.cluster.points.size() );
      marker.colors.resize( object_.cluster.points.size() );

      ROS_INFO_STREAM( "Adding point cluster. #points=" << object_.cluster.points.size() );
      bool use_color = false;
      if(object_.cluster.channels.size() != 0 && object_.cluster.channels[0].name == "rgb") use_color = true;
      for ( size_t i=0; i<object_.cluster.points.size(); i++)
      {
        marker.points[i].x = object_.cluster.points[i].x;
        marker.points[i].y = object_.cluster.points[i].y;
        marker.points[i].z = object_.cluster.points[i].z;
        if(use_color)
        {
          uint32_t rgb = *reinterpret_cast<int*>(&object_.cluster.channels[0].values[i]);
          marker.colors[i].r = ((rgb >> 16) & 0x0000ff)/255. * 0.5;
          marker.colors[i].g = ((rgb >> 8) & 0x0000ff)/255. * 1.5;
          marker.colors[i].b = (rgb & 0x0000ff)/255. * 0.5;
          marker.colors[i].a = 1.0;
          if(marker.colors[i].g > 1.0) marker.colors[i].g = 1.0;
          else if(marker.colors[i].g < 0.5) marker.colors[i].g = 0.5;
        }
      }
    }

    InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = InteractiveMarkerControl::MENU;
    control.orientation_mode = InteractiveMarkerControl::INHERIT;

    control.markers.push_back( marker );

    int_marker.controls.push_back( control );

    marker_server_.insert( int_marker );
    menu_handler_.apply( marker_server_, int_marker.name );
    marker_server_.applyChanges();
  }


  void GraspableObjectHandler::switchFlag( bool *value, 
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    *value = !(bool)(*value);
    makeMarker();
    updateMenu();
  }


}
