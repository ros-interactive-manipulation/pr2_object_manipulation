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

// Author(s): Gil Jones and Matei Ciocarlie

#include "tabletop_collision_map_processing/collision_map_interface.h"

#include <tf/transform_datatypes.h>

#include <std_srvs/Empty.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <arm_navigation_msgs/AttachedCollisionObject.h>

#include <object_manipulation_msgs/FindClusterBoundingBox.h>
#include <object_manipulation_msgs/FindClusterBoundingBox2.h>

#include <household_objects_database_msgs/DatabaseModelPose.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include <object_manipulator/tools/msg_helpers.h>

namespace tabletop_collision_map_processing {

static const std::string CLUSTER_BOUNDING_BOX_NAME = "find_cluster_bounding_box";
static const std::string CLUSTER_BOUNDING_BOX2_NAME = "find_cluster_bounding_box2";
static const std::string CLUSTER_BOUNDING_BOX_3D_NAME = "find_cluster_bounding_box_3d";
static const std::string CLUSTER_BOUNDING_BOX2_3D_NAME = "find_cluster_bounding_box2_3d";

CollisionMapInterface::CollisionMapInterface() : 
  root_nh_(""),
  priv_nh_("~"),
  connections_established_(false),
  collision_object_current_id_(0)
{
  collision_object_pub_ = root_nh_.advertise<arm_navigation_msgs::CollisionObject>("collision_object", 10);
  attached_object_pub_ = root_nh_.advertise<arm_navigation_msgs::AttachedCollisionObject>("attached_collision_object", 10); 
}
 
void CollisionMapInterface::connectServices()
{
  cluster_bounding_box_client_ = root_nh_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox>
    (CLUSTER_BOUNDING_BOX_NAME, true);
  cluster_bounding_box2_client_ = root_nh_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>
    (CLUSTER_BOUNDING_BOX2_NAME, true);
  cluster_bounding_box_3d_client_ = root_nh_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox>
    (CLUSTER_BOUNDING_BOX_3D_NAME, true);
  cluster_bounding_box2_3d_client_ = root_nh_.serviceClient<object_manipulation_msgs::FindClusterBoundingBox2>
    (CLUSTER_BOUNDING_BOX2_3D_NAME, true);
  priv_nh_.param<int>("point_skip_num", point_skip_num_, 1);
  priv_nh_.param<int>("collision_box_size", collision_box_size_, 0.003);
  priv_nh_.param<double>("table_thickness", table_thickness_, 0.);
}

bool CollisionMapInterface::connectionsPresent()
{
  if ( !ros::service::exists(CLUSTER_BOUNDING_BOX_NAME, true) ) return false;
  if ( !ros::service::exists(CLUSTER_BOUNDING_BOX2_NAME, true) ) return false;
  return true;
}

bool CollisionMapInterface::connectionsEstablished(ros::Duration timeout)
{
  if (connections_established_) return true;
  ros::Time start_time = ros::Time::now();
  ros::Duration ping_time = ros::Duration(1.0);
  if (timeout >= ros::Duration(0) && ping_time > timeout) ping_time = timeout;
  while (!connectionsPresent())
  {
    ping_time.sleep();
    if (!root_nh_.ok()) return false;
    if (timeout >= ros::Duration(0) &&
	ros::Time::now() - start_time >= timeout) return false;
  }
  connectServices();
  connections_established_ = true;
  return true;
}

void CollisionMapInterface::removeCollisionModel(std::string collision_name)
{
  arm_navigation_msgs::CollisionObject reset_object;
  reset_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  reset_object.header.frame_id = "base_link";
  reset_object.header.stamp = ros::Time::now();
  reset_object.id = collision_name;
  collision_object_pub_.publish(reset_object);
}

void CollisionMapInterface::resetCollisionModels()
{
  arm_navigation_msgs::CollisionObject reset_object;
  reset_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  reset_object.header.frame_id = "base_link";
  reset_object.header.stamp = ros::Time::now();
  reset_object.id = "all";
  collision_object_pub_.publish(reset_object);
  //collision_object_current_id_ = 0;
}

void CollisionMapInterface::resetAttachedModels()
{
  arm_navigation_msgs::CollisionObject reset_object;
  reset_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::REMOVE;
  reset_object.header.frame_id = "base_link";
  reset_object.header.stamp = ros::Time::now();
  reset_object.id = "all";
  arm_navigation_msgs::AttachedCollisionObject reset_attached_objects;
  reset_attached_objects.object.header.frame_id = "base_link";
  reset_attached_objects.object.header.stamp = ros::Time::now();
  reset_attached_objects.link_name = "all";
  reset_attached_objects.object = reset_object;
  attached_object_pub_.publish(reset_attached_objects);
}

std::string CollisionMapInterface::getNextObjectName()
{
  std::ostringstream iss;
  iss << collision_object_current_id_++;
  if (collision_object_current_id_ > 10000) collision_object_current_id_ = 0;
  return "graspable_object_" + iss.str();
}

void CollisionMapInterface::processCollisionGeometryForTable(const tabletop_object_detector::Table &table,
                                                   std::string table_collision_name) 
{
  arm_navigation_msgs::CollisionObject table_object;
  table_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  table_object.header.frame_id = table.pose.header.frame_id;
  table_object.header.stamp = ros::Time::now();
  table_object.shapes.resize(1);
  table_object.poses.resize(1);

  //if the convex hull mesh is available, use that
  if(table.convex_hull.triangles.size() != 0)
  {
    ROS_INFO("using convex hull mesh for table collision geometry");
    arm_navigation_msgs::Shape table_shape;
    table_shape.type = table_shape.MESH;
    table_shape.vertices = table.convex_hull.vertices;
    for (size_t i=0; i<table.convex_hull.triangles.size(); i++)
    {
      table_shape.triangles.push_back( table.convex_hull.triangles.at(i).vertex_indices.at(0) );
      table_shape.triangles.push_back( table.convex_hull.triangles.at(i).vertex_indices.at(1) );
      table_shape.triangles.push_back( table.convex_hull.triangles.at(i).vertex_indices.at(2) );
    }
    table_object.shapes[0] = table_shape;
    table_object.poses[0] = table.pose.pose;
    if(table_thickness_)
    {
      ROS_INFO("using table_thickness of %.3f", table_thickness_);
      tf::Transform T, P;
      tf::poseMsgToTF(table.pose.pose, P);
      geometry_msgs::Pose shifted_table_pose;
      geometry_msgs::Pose shifted_table_offset = object_manipulator::msg::createPoseMsg(tf::Pose(tf::Quaternion::getIdentity(), tf::Vector3(0,0,-table_thickness_)));
      tf::poseMsgToTF(shifted_table_offset, T);
      tf::poseTFToMsg( P*T, shifted_table_pose);
      table_object.shapes.push_back(table_shape);
      table_object.poses.push_back(shifted_table_pose);
    }
  }

  //otherwise, create a box using the old table specification (min/max values)
  else
  {
    ROS_INFO("using box table collision geometry");
    table_object.shapes[0].type = arm_navigation_msgs::Shape::BOX;
    table_object.shapes[0].dimensions.resize(3);
    table_object.shapes[0].dimensions[0] = fabs(table.x_max-table.x_min);
    table_object.shapes[0].dimensions[1] = fabs(table.y_max-table.y_min);
    if(table_thickness_) table_object.shapes[0].dimensions[2] = table_thickness_;
    else table_object.shapes[0].dimensions[2] = 0.01;

    //set the origin of the table object in the middle of the table
    tf::Transform table_trans;
    tf::poseMsgToTF(table.pose.pose, table_trans);
    tf::Transform table_translation;
    table_translation.setIdentity();
    table_translation.setOrigin( tf::Vector3( (table.x_min + table.x_max)/2.0, (table.y_min + table.y_max)/2.0, 0.0) );
    table_trans = table_trans * table_translation;
    tf::poseTFToMsg(table_trans, table_object.poses[0]);
  }

  table_object.id = table_collision_name;
  collision_object_pub_.publish(table_object);
}

void CollisionMapInterface::processCollisionGeometryForObject
  (const household_objects_database_msgs::DatabaseModelPose &model_pose,
   std::string &collision_name)
{
  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.shapes.resize(1);

  if (!getMeshFromDatabasePose(model_pose, collision_object.shapes[0]))
  {
    throw CollisionMapException("Loading mesh for database object failed");
  }
  collision_object.header.frame_id = model_pose.pose.header.frame_id;
  collision_object.header.stamp = ros::Time::now();
  collision_object.poses.push_back(model_pose.pose.pose);
  
  collision_object.shapes[0].type = arm_navigation_msgs::Shape::MESH;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  collision_name = getNextObjectName();
  collision_object.id = collision_name;
  collision_object_pub_.publish(collision_object);
}


void CollisionMapInterface::processCollisionGeometryForObjectAsBoundingBox
  (const household_objects_database_msgs::DatabaseModelPose &model_pose,
   std::string &collision_name)
{
  arm_navigation_msgs::Shape model_shape;
  if (!getMeshFromDatabasePose(model_pose, model_shape))
  {
    throw CollisionMapException("Loading mesh for database object failed");
  }
  sensor_msgs::PointCloud cluster;
  cluster.header = model_pose.pose.header;
  for (size_t i=0; i<model_shape.vertices.size(); i++)
  {
    geometry_msgs::Point32 point;
    point.x = model_shape.vertices[i].x;
    point.y = model_shape.vertices[i].y;
    point.z = model_shape.vertices[i].z;
    cluster.points.push_back(point);
  }
  object_manipulation_msgs::ClusterBoundingBox box;
  getClusterBoundingBox(cluster, box.pose_stamped, box.dimensions);

  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

  collision_name = getNextObjectName();
  collision_object.id = collision_name;

  collision_object.header.frame_id = model_pose.pose.header.frame_id;
  collision_object.header.stamp = ros::Time::now();
  
  arm_navigation_msgs::Shape shape;
  shape.type = arm_navigation_msgs::Shape::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = box.dimensions.x;
  shape.dimensions[1] = box.dimensions.y;
  shape.dimensions[2] = box.dimensions.z;
  collision_object.shapes.push_back(shape);

  //multiply bbox trans by object trans
  tf::Transform model_trans;
  tf::poseMsgToTF(model_pose.pose.pose, model_trans);
  tf::Transform box_trans;
  tf::poseMsgToTF(box.pose_stamped.pose, box_trans);
  tf::Transform total_trans = model_trans * box_trans;

  geometry_msgs::Pose total_pose;
  tf::poseTFToMsg(total_trans, total_pose);
  collision_object.poses.push_back(total_pose);

  collision_object_pub_.publish(collision_object);
}

void 
CollisionMapInterface::processCollisionGeometryForBoundingBox(const object_manipulation_msgs::ClusterBoundingBox &box, 
							      std::string &collision_name)
{
  ROS_INFO("Adding bounding box with dimensions %f %f %f to collision map", 
	   box.dimensions.x, box.dimensions.y, box.dimensions.z);

  arm_navigation_msgs::CollisionObject collision_object;
  collision_object.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;

  collision_name = getNextObjectName();
  collision_object.id = collision_name;

  collision_object.header.frame_id = box.pose_stamped.header.frame_id;
  collision_object.header.stamp = ros::Time::now();
  
  arm_navigation_msgs::Shape shape;
  shape.type = arm_navigation_msgs::Shape::BOX;
  shape.dimensions.resize(3);
  shape.dimensions[0] = box.dimensions.x;
  shape.dimensions[1] = box.dimensions.y;
  shape.dimensions[2] = box.dimensions.z;
  collision_object.shapes.push_back(shape);
  collision_object.poses.push_back(box.pose_stamped.pose);

  collision_object_pub_.publish(collision_object);
}

void CollisionMapInterface::processCollisionGeometryForCluster(const sensor_msgs::PointCloud &cluster,
							       std::string &collision_name, float collision_size)
{
  ROS_INFO("Adding cluster with %u points to collision map", (unsigned int)cluster.points.size()); 

  arm_navigation_msgs::CollisionObject many_boxes;
  many_boxes.operation.operation = arm_navigation_msgs::CollisionObjectOperation::ADD;
  many_boxes.header = cluster.header;
  many_boxes.header.stamp = ros::Time::now();
  unsigned int num_to_use = (unsigned int)(cluster.points.size()/point_skip_num_);
  many_boxes.shapes.resize(num_to_use);
  many_boxes.poses.resize(num_to_use);

  if(collision_size < 0)
    collision_size = collision_box_size_;

  for(unsigned int i = 0; i < num_to_use; i++) {
    arm_navigation_msgs::Shape shape;
    shape.type = arm_navigation_msgs::Shape::BOX;
    shape.dimensions.resize(3);
    shape.dimensions[0] = collision_size;
    shape.dimensions[1] = collision_size;
    shape.dimensions[2] = collision_size;
    many_boxes.shapes[i]=shape;
    geometry_msgs::Pose pose;
    pose.position.x = cluster.points[i*point_skip_num_].x;
    pose.position.y = cluster.points[i*point_skip_num_].y;
    pose.position.z = cluster.points[i*point_skip_num_].z;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    many_boxes.poses[i] = pose;
  }

  collision_name = getNextObjectName();
  //use this name for the collision map
  many_boxes.id = collision_name;
  collision_object_pub_.publish(many_boxes);
}

bool CollisionMapInterface::getMeshFromDatabasePose(const household_objects_database_msgs::DatabaseModelPose &model_pose,
						    arm_navigation_msgs::Shape& mesh) 
{
  static bool service_initialized = false;
  if (!service_initialized)
  {    
    std::string get_model_mesh_srv_name;
    priv_nh_.param<std::string>("get_model_mesh_srv", get_model_mesh_srv_name, "get_model_mesh_srv");
    ros::Time start_time = ros::Time::now();
    while ( !ros::service::waitForService(get_model_mesh_srv_name, ros::Duration(2.0)) ) 
    {
      ROS_INFO("Waiting for %s service to come up", get_model_mesh_srv_name.c_str());
      if (!root_nh_.ok() || ros::Time::now() - start_time >= ros::Duration(5.0))
      {
        return false;
      }
    }
    get_model_mesh_srv_ = root_nh_.serviceClient<household_objects_database_msgs::GetModelMesh>
      (get_model_mesh_srv_name, false);
    service_initialized = true;
  }
  household_objects_database_msgs::GetModelMesh get_mesh;
  get_mesh.request.model_id = model_pose.model_id;
  if ( !get_model_mesh_srv_.call(get_mesh) )
  {
    ROS_ERROR("Get model mesh service service call failed altogether");
    return false;
  }
  else if (get_mesh.response.return_code.code != get_mesh.response.return_code.SUCCESS )
  {
    ROS_ERROR("Get model mesh service returned an error");
    return false;
  }
  //translate to arm_navigation_msgs::Shape 
  mesh.vertices = get_mesh.response.mesh.vertices;
  mesh.triangles.clear();
  for (size_t i=0; i<get_mesh.response.mesh.triangles.size(); i++)
  {
    mesh.triangles.push_back( get_mesh.response.mesh.triangles.at(i).vertex_indices.at(0) );
    mesh.triangles.push_back( get_mesh.response.mesh.triangles.at(i).vertex_indices.at(1) );
    mesh.triangles.push_back( get_mesh.response.mesh.triangles.at(i).vertex_indices.at(2) );
  }
  return true;
}

void CollisionMapInterface::getClusterBoundingBox(const sensor_msgs::PointCloud &cluster,
						  geometry_msgs::PoseStamped &pose_stamped, 
						  geometry_msgs::Vector3 &dimensions)
{
  object_manipulation_msgs::FindClusterBoundingBox srv;
  srv.request.cluster = cluster;
  if (!cluster_bounding_box_client_.call(srv.request, srv.response))
  {
    ROS_ERROR("Failed to call cluster bounding box client");
    throw CollisionMapException("Failed to call cluster bounding box client");
  }
  pose_stamped = srv.response.pose;
  dimensions = srv.response.box_dims;
  if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
  {
    ROS_ERROR("Cluster bounding box client returned an error (0.0 bounding box)");
    throw CollisionMapException("Bounding box computation failed");  
  }
}

void CollisionMapInterface::getClusterBoundingBox(const sensor_msgs::PointCloud2 &cluster,
						  geometry_msgs::PoseStamped &pose_stamped, 
						  geometry_msgs::Vector3 &dimensions)
{
  object_manipulation_msgs::FindClusterBoundingBox2 srv;
  srv.request.cluster = cluster;
  if (!cluster_bounding_box2_client_.call(srv.request, srv.response))
  {
    ROS_ERROR("Failed to call cluster bounding box client");
    throw CollisionMapException("Failed to call cluster bounding box client");
  }
  pose_stamped = srv.response.pose;
  dimensions = srv.response.box_dims;
  if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
  {
    ROS_ERROR("Cluster bounding box 2 client returned an error (0.0 bounding box)");
    throw CollisionMapException("Bounding box computation failed");  
  }
}

void CollisionMapInterface::getClusterBoundingBox3D(const sensor_msgs::PointCloud &cluster,
						  geometry_msgs::PoseStamped &pose_stamped, 
						  geometry_msgs::Vector3 &dimensions)
{
  object_manipulation_msgs::FindClusterBoundingBox srv;
  srv.request.cluster = cluster;
  if (!cluster_bounding_box_3d_client_.call(srv.request, srv.response))
  {
    ROS_ERROR("Failed to call cluster bounding box client");
    throw CollisionMapException("Failed to call cluster bounding box client");
  }
  pose_stamped = srv.response.pose;
  dimensions = srv.response.box_dims;
  if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
  {
    ROS_ERROR("Cluster bounding box 3d client returned an error (0.0 bounding box)");
    throw CollisionMapException("Bounding box computation failed");  
  }
}

void CollisionMapInterface::getClusterBoundingBox3D(const sensor_msgs::PointCloud2 &cluster,
						  geometry_msgs::PoseStamped &pose_stamped, 
						  geometry_msgs::Vector3 &dimensions)
{
  object_manipulation_msgs::FindClusterBoundingBox2 srv;
  srv.request.cluster = cluster;
  if (!cluster_bounding_box2_3d_client_.call(srv.request, srv.response))
  {
    ROS_ERROR("Failed to call cluster bounding box client");
    throw CollisionMapException("Failed to call cluster bounding box client");
  }
  pose_stamped = srv.response.pose;
  dimensions = srv.response.box_dims;
  if (dimensions.x == 0.0 && dimensions.y == 0.0 && dimensions.z == 0.0)
  {
    ROS_ERROR("Cluster bounding box 2 3d client returned an error (0.0 bounding box)");
    throw CollisionMapException("Bounding box computation failed");  
  }
}


/*! Assumes the bounding box is in the same frame as the table */
bool CollisionMapInterface::extendBoundingBoxZToTable(const tabletop_object_detector::Table &table,
                                                      geometry_msgs::PoseStamped &pose_stamped, 
                                                      geometry_msgs::Vector3 &dimensions)
{
  if (table.pose.header.frame_id != pose_stamped.header.frame_id)
  {
    ROS_INFO("cannot extend bbox to table, they are not in the same frame");
    return false;
  }

  //get the distance from the box to the table
  tf::Vector3 table_location(table.pose.pose.position.x,
                           table.pose.pose.position.y,
                           table.pose.pose.position.z);
  tf::Vector3 bbox_location(pose_stamped.pose.position.x, 
                          pose_stamped.pose.position.y, 
                          pose_stamped.pose.position.z);
  tf::Vector3 table_to_bbox = bbox_location - table_location;
  tf::Transform table_trans;
  tf::poseMsgToTF(table.pose.pose, table_trans);
  tf::Vector3 table_z = table_trans.getBasis().getColumn(2);
  ROS_INFO("Table z: %f %f %f in frame %s", 
           table_z.getX(), table_z.getY(), table_z.getZ(), table.pose.header.frame_id.c_str());
  double box_to_table_distance = table_to_bbox.dot(table_z);
  ROS_INFO("Table distance: %f", box_to_table_distance);

  //now we actually make the assumptions that the z axes are close to each other
  //as doing an actual computation was just too hard
  tf::Transform bbox_trans;
  tf::poseMsgToTF(pose_stamped.pose, bbox_trans);
  tf::Vector3 bbox_z = bbox_trans.getBasis().getColumn(2);
  if (fabs(bbox_z.dot(table_z)) < 0.9848) //10 degrees
  { 
    ROS_INFO("cannot extend bbox to table; z axes do not coincide");
    return false;
  }
  
  double z_to_table = box_to_table_distance;
  ROS_INFO("z_to_table: %f", z_to_table);
  if ( z_to_table < 0.005 || z_to_table > 0.3)
  {
    ROS_INFO("cannot extend bbox to table; getting z equal to %f", z_to_table);
    return false;
  }
  ROS_INFO("Old z: %f", dimensions.z);
  double new_z = dimensions.z/2.0 + z_to_table;
  ROS_INFO("New z: %f", new_z);
  pose_stamped.pose.position.z = pose_stamped.pose.position.z + (dimensions.z/2.0) - (new_z/2.0);
  dimensions.z = new_z;
  return true;
}


} //namespace tabletop_collision_map_processing
