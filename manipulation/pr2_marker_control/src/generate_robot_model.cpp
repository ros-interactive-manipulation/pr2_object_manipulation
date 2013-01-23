/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "pr2_marker_control/generate_robot_model.h"

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>

using namespace visualization_msgs;

urdf::Model getRobotURDFModel()
{
  std::string urdf_content;
  urdf::Model model;

  std::string robot_parameter = "robot_description";

  ros::NodeHandle nh;

  // obtain robot description from parameter server
  std::string loc;
  if (nh.searchParam(robot_parameter, loc))
  {
    nh.getParam(loc, urdf_content);
  }
  else
  {
    ROS_ERROR( "Parameter [ %s ] does not exist, and was not found by searchParam()", robot_parameter.c_str());
  }

  if (urdf_content.empty())
  {
    ROS_ERROR( "URDF is empty");
  }

  // parse xml-based robot description
  TiXmlDocument doc;
  doc.Parse(urdf_content.c_str());
  if (!doc.RootElement())
  {
    ROS_ERROR( "URDF failed XML parse");
  }

  // create robot model
  if (!model.initXml(doc.RootElement()))
  {
    ROS_ERROR( "URDF failed Model parse");
  }

  ROS_DEBUG( "URDF parsed OK");

  return model;
}

void addMeshMarkersFromRobotModel(boost::shared_ptr<const urdf::Link> link, std::vector<InteractiveMarker>& int_markers)
{
  // All interactive markers will be set to time(0), so they update automatically in rviz.
  const std::string& link_name = link->name;

  geometry_msgs::PoseStamped ps;
  ps.header.stamp = ros::Time(0);
  ps.header.frame_id = "/" + link_name;

  if (link && link->visual && link->visual->geometry)
  {
    const urdf::Geometry &geom = *link->visual->geometry;

    switch (geom.type)
    {
 /*     case urdf::Geometry::SPHERE:
      {
        const urdf::Sphere& sphere = static_cast<const urdf::Sphere&>(geom);

        break;
      }
      case urdf::Geometry::BOX:
      {
        const urdf::Box& box = static_cast<const urdf::Box&>(geom);

        break;
      }
      case urdf::Geometry::CYLINDER:
      {
        const urdf::Cylinder& cylinder = static_cast<const urdf::Cylinder&>(geom);

        break;
      }*/
      case urdf::Geometry::MESH:
      {
        const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geom);

        if (mesh.filename.empty())
          return;

        const std::string& mesh_filename = mesh.filename;

        std_msgs::ColorRGBA color;

        color.r = 1.0; //link->visual->material->color.r;
        color.g = 1.0; //link->visual->material->color.g;
        color.b = 1.0; //link->visual->material->color.b;
        color.a = 1; //link->visual->material->color.a;

        ROS_DEBUG("Creating interactive marker for link %s (%s)", link_name.c_str(), mesh_filename.c_str());

        // create mesh marker
        visualization_msgs::Marker mesh_marker;

        mesh_marker.pose.position.x = link->visual->origin.position.x;
        mesh_marker.pose.position.y = link->visual->origin.position.y;
        mesh_marker.pose.position.z = link->visual->origin.position.z;

        mesh_marker.pose.orientation.x = link->visual->origin.rotation.x;
        mesh_marker.pose.orientation.y = link->visual->origin.rotation.y;
        mesh_marker.pose.orientation.z = link->visual->origin.rotation.z;
        mesh_marker.pose.orientation.w = link->visual->origin.rotation.w;

        mesh_marker.color = color;

        mesh_marker.mesh_resource = mesh_filename;
        mesh_marker.mesh_use_embedded_materials = true;
        mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        mesh_marker.scale.x = mesh.scale.x;
        mesh_marker.scale.y = mesh.scale.y;
        mesh_marker.scale.z = mesh.scale.z;

        // interactive marker control
        visualization_msgs::InteractiveMarkerControl control;
        control.markers.push_back(mesh_marker);
        control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
        control.always_visible = true;

        // create interactive marker
        InteractiveMarker int_marker;

        int_marker.header = ps.header;
        int_marker.pose.orientation.w = 1;
        int_marker.name = link_name;
        int_marker.description = link_name;
        int_marker.scale = 0.3;

        int_marker.controls.push_back(control);

        int_markers.push_back(int_marker);
        break;
      }
      default:
        ROS_WARN("Unsupported geometry type for element: %d", geom.type);
        break;
    }
  }
  else
  {
    ROS_DEBUG("Robot model: link: %s has a null child!", link->name.c_str());
  }

  // recursively traverse the urdf tree
  for (std::vector<boost::shared_ptr<urdf::Link> >::const_iterator child = link->child_links.begin();
      child != link->child_links.end(); child++)
    addMeshMarkersFromRobotModel(*child,int_markers);

}

void addMeshMarkersFromRobotModel(std::vector<InteractiveMarker>& int_markers)
{
  urdf::Model model = getRobotURDFModel();
  addMeshMarkersFromRobotModel(model.getRoot(),int_markers);
}

