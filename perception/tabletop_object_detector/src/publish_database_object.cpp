
/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
  
#include <string>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include <household_objects_database_msgs/GetModelMesh.h>

visualization_msgs::Marker getFitMarker(const shape_msgs::Mesh &mesh)
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.type = visualization_msgs::Marker::POINTS;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.scale.x = 0.003;
  marker.scale.y = 0.003;
  marker.points.insert(marker.points.begin(), mesh.vertices.begin(), mesh.vertices.end());
  //add a line starting from origin along the x direction of the object
  for (int i=0; i<40; i++)
  {
    geometry_msgs::Point p;
    p.x = 0.0005 * i;
    p.y = p.z = 0.0;
    marker.points.push_back(p);
  }
  return marker;
}

int main(int argc, char **argv)
{
  if (argc < 2) 
  {
    ROS_ERROR("Usage: publish_database_object model_id");
    exit(0);
  }

  //create handlers and initialize ros
  ros::Publisher marker_pub_;
  ros::ServiceClient get_model_mesh_srv_;
  ros::init(argc, argv, "publish_database_mesh");
  ros::NodeHandle nh_("");

  //advertise markers
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("database_object"), 10);
  //wait a little but to make sure rviz subscribes
  ros::Duration(1.0).sleep();

  //subscribe to service that gets a model mesh form the database
  std::string get_model_mesh_srv_name = "/objects_database_node/get_model_mesh";
  while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) && nh_.ok() ) 
  {
    ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
  }
  if (!nh_.ok()) exit(0);
  get_model_mesh_srv_ = nh_.serviceClient<household_objects_database_msgs::GetModelMesh>
    (get_model_mesh_srv_name, true);

  int model_id = atoi(argv[1]);
  ROS_INFO("Publishing mesh for model %d", model_id);
  
  //get the mesh
  household_objects_database_msgs::GetModelMesh get_mesh;
  get_mesh.request.model_id = model_id;
  if ( !get_model_mesh_srv_.call(get_mesh) || 
       get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
  {
    ROS_ERROR("Failed to call database get mesh service for marker display");
    exit(0);
  }

  visualization_msgs::Marker fitMarker =  getFitMarker(get_mesh.response.mesh);
  //change here with the desired frame
  fitMarker.header.frame_id = "foo_frame";
  fitMarker.header.stamp = ros::Time::now();
  //publish at origin
  fitMarker.pose.orientation.w = 1;
  fitMarker.ns = "publish_database_object";
  fitMarker.id = 0;
  marker_pub_.publish(fitMarker);

  ROS_INFO("Marker published");
  //wait a little but to make sure the marker gets to rviz
  ros::Duration(1.0).sleep();
}
