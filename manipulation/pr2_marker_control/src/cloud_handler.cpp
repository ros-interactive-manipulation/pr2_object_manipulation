/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

// author: Adam Leeper

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <point_cloud_server/StoreCloudAction.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>

#include <pr2_object_manipulation_msgs/CameraFocus.h>

#include <pr2_marker_control/cloud_handler.h>

using namespace visualization_msgs;
using namespace interactive_markers;


// **** 10 ***** 20 ****** 30 ****** 40 ****** 50 ****** 60 ****** 70 ****** 80 ****** 90 ***** 100 ***** 110 ***** 120

CloudHandler::CloudHandler( ros::NodeHandle *nh, tf::TransformListener *tfl, std::string marker_name, 
                            std::string topic_name, std::string server_name,
                            object_manipulator::MechanismInterface &mechanism,
                            std::string cloud_frame):
  marker_name_(marker_name)
  , nh_(nh)
  , tfl_(tfl)
  , marker_server_(topic_name, server_name, false)
  , cloud_pts_   ( new pcl::PointCloud  <PointT> ()       )
  , cloud_normals_  ( new pcl::PointCloud  <pcl::Normal> ()  )
  , tree_(new pcl::search::KdTree<PointT>())
  , mechanism_(mechanism)
  , cloud_server_client_("point_cloud_server_action", true)
  , cloud_frame_(cloud_frame)
{
  ros::NodeHandle pnh("~");
  pnh.param<double>("voxel_size", voxel_size_, 0.002);
  pnh.param<std::string>("head_pointing_frame", head_pointing_frame_, "/default_head_pointing_frame");
  
  pub_right_click_ = nh_->advertise<geometry_msgs::PoseStamped>("right_click_point", 1);
  pub_left_click_ = nh_->advertise<geometry_msgs::PoseStamped>("left_click_point", 1);
  pub_refresh_flag_ = nh_->advertise<std_msgs::String>("refresh_flag", 1);
  pub_focus_ = nh_->advertise<pr2_object_manipulation_msgs::CameraFocus>("camera_focus", 1);
  
  // This must come last!
  makeMenu();
}

CloudHandler::~CloudHandler()
{
  marker_server_.erase(marker_name_);
}

void CloudHandler::makeMenu()
{
  // create menu
  menu_handler_.insert( "Broadcast click position",  boost::bind( &CloudHandler::menuPoint, this, _1) );
  menu_handler_.insert( "Focus camera here",  boost::bind( &CloudHandler::menuFocus, this, _1) );
  menu_handler_.insert( "Refresh",  boost::bind( &CloudHandler::refresh, this ) );
  menu_handler_.insert( "Clear",    boost::bind( &CloudHandler::clear, this) );
}

void CloudHandler::leftClickPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(!feedback->mouse_point_valid)
  {
    ROS_WARN("Clicked point had an invalid position. Not looking there!");
    return;
  }

  ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                         << feedback->header.frame_id << " at point\n" << feedback->mouse_point );
  
  geometry_msgs::PointStamped click_point;
  click_point.point = feedback->mouse_point;
  click_point.header = feedback->header;
  click_point.header.stamp = ros::Time(0);
  
  try
  {
    !mechanism_.pointHeadAction(click_point, head_pointing_frame_, false); 
  }
  catch (object_manipulator::ServiceNotFoundException &ex)
  {
    ROS_WARN("Can't point head, a needed service or action server was not found.");
  }
}

void CloudHandler::menuFocus( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(!feedback->mouse_point_valid) return;

  ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                         << feedback->header.frame_id << " at point\n" << feedback->mouse_point );
  
  geometry_msgs::PointStamped click_point;
  click_point.point = feedback->mouse_point;
  click_point.header = feedback->header;
  click_point.header.stamp = ros::Time(0);
  try{
    tfl_->transformPoint(cloud_frame_, click_point, click_point );
  }
  catch(...)
  {
    ROS_ERROR("TF had a problem transforming between [%s] and [%s].",
              cloud_frame_.c_str(), click_point.header.frame_id.c_str());
    return;
  }
  ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                         << click_point.header.frame_id << " at point\n" << click_point.point );
  
  pr2_object_manipulation_msgs::CameraFocus msg;
  msg.focal_point = click_point;
  pub_focus_.publish(msg);
}

void CloudHandler::menuPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  if(feedback->mouse_point_valid)
  {
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                           << feedback->header.frame_id << " at point\n" << feedback->mouse_point );

    geometry_msgs::PointStamped click_point;
    click_point.point = feedback->mouse_point;
    click_point.header = feedback->header;
    click_point.header.stamp = ros::Time(0);
    geometry_msgs::PointStamped head_point;
    head_point.header.frame_id = "/head_tilt_link";
    try{
      tfl_->transformPoint(cloud_frame_, click_point, click_point );
    }
    catch(...)
    {
      ROS_ERROR("TF had a problem transforming between [%s] and [%s].",
                cloud_frame_.c_str(), click_point.header.frame_id.c_str());
      return;
    }
    try{
      tfl_->transformPoint(cloud_frame_, head_point, head_point);
    }
    catch(...)
    {
      ROS_ERROR("TF had a problem transforming between [%s] and [%s].",
                cloud_frame_.c_str(), head_point.header.frame_id.c_str());
      return;
    }
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Button click in frame "
                           << click_point.header.frame_id << " at point\n" << click_point.point );
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Head point in frame "
                           << head_point.header.frame_id << " at point\n" << head_point.point );

    PointT position; //(point.point.x, point.point.y, point.point.z);
    position.x = click_point.point.x;
    position.y = click_point.point.y;
    position.z = click_point.point.z;
    std::vector< int >   	k_indices(1, 0);
    std::vector< float >  k_sqr_distances(1, 0);
    int N = tree_->nearestKSearch ( position, 1, k_indices, k_sqr_distances);
    if(!N)
    {
      ROS_ERROR("Found no point near clicked point... Serious error... Not broadcasting.");
      return;
    }
    int index = k_indices[0];

    geometry_msgs::PoseStamped ps;
    
    //PointT pt = cloud_pts_->points[index];
    //pcl::Normal norm = cloud_normals_->points[index];

    tf::Vector3 normal = tf::Vector3(cloud_normals_->points[index].normal_x,
                                     cloud_normals_->points[index].normal_y,
                                     cloud_normals_->points[index].normal_z);

    tf::Vector3 point = tf::Vector3(cloud_pts_->points[index].x,
                                    cloud_pts_->points[index].y,
                                    cloud_pts_->points[index].z);

    tf::Vector3 head = tf::Vector3(head_point.point.x, head_point.point.y, head_point.point.z);
    tf::Vector3 point_to_head = (head - point).normalized();
    // We actually want the normal that points away from the head
    if( point_to_head.dot(normal) < 0) normal *= -1;

    ps.pose.position.x = point.x();
    ps.pose.position.y = point.y();
    ps.pose.position.z = point.z();
    ROS_DEBUG_STREAM_NAMED("cloud_handler", "Nearest point at:\n" << ps.pose.position );

    tf::Vector3 Z = normal.normalized();
    tf::Vector3 Y = normal.cross(tf::Vector3(0,0,1)).normalized();
    tf::Vector3 X = Y.cross(normal).normalized();
    tf::Matrix3x3 mat(X.x(), Y.x(), Z.x(),
                    X.y(), Y.y(), Z.y(),
                    X.z(), Y.z(), Z.z());
    tf::Quaternion q;
    mat.getRotation(q);
    tf::quaternionTFToMsg(q, ps.pose.orientation);
    ps.header = click_point.header;
    pub_right_click_.publish(ps);
  }
  else
  {
    ROS_WARN("Clicked point had an invalid position. Not broadcasting.");
  }
}

void CloudHandler::saveCloudAndNormals()
{

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(msg_cloud_, *cloud_pts_);
  ROS_DEBUG_NAMED("cloud_handler", "Input cloud in frame [%s] has %d points.", cloud_pts_->header.frame_id.c_str(), (int)cloud_pts_->points.size());

  //std::vector<int> dummy_variable;
  //pcl::removeNaNFromPointCloud(*cloud_pts_, *cloud_pts_, dummy_variable);
  //ROS_DEBUG_NAMED("cloud_handler", "After removing NaNs cloud has %d points.", (int)cloud_pts_->points.size());
  //cloud_pts_->header = msg_cloud_.header;


  // Create a KD-Tree
  ROS_DEBUG_NAMED("cloud_handler", "Building kdtree...");
  ros::WallTime begin = ros::WallTime::now();
  tree_.reset(new pcl::search::KdTree<PointT> ());
  tree_->setInputCloud (cloud_pts_);
  //tree->addPointsFromInputCloud();
  ROS_DEBUG_NAMED("cloud_handler", "Initializing kdtree took %.3lf ms", (ros::WallTime::now() - begin).toSec()*1000.0);


  cloud_normals_.reset(new pcl::PointCloud<pcl::Normal> ());

  begin = ros::WallTime::now();
  ROS_DEBUG_NAMED("cloud_handler", "Estimating normals...");
  pcl::NormalEstimationOMP<PointT, pcl::Normal> mls;

  // Set parameters
  mls.setInputCloud (cloud_pts_);
  mls.setSearchMethod (tree_);
  mls.setKSearch(20);
  // Reconstruct
  //ROS_DEBUG_NAMED("haptics", "computing...");
  mls.compute(*cloud_normals_);
  //ROS_DEBUG_NAMED("haptics", "...done!");
  ROS_DEBUG_NAMED("cloud_handler", "OMP Normal estimation took %.3lf ms", (ros::WallTime::now() - begin).toSec()*1000.0);

  //  Need to remove NANs
//    pcl::PassThrough<pcl::PointXYZRGB > sor;
//    sor.setInputCloud (cloud);
//    sor.setFilterLimits(-100, 100);
//    sor.setFilterFieldName("z");
//    sor.filter (*cloud);
//    ROS_DEBUG_NAMED("cloud_handler", "After passthrough cloud has %d points.", (int)cloud->points.size());



}

void CloudHandler::get( sensor_msgs::PointCloud2 &cloud)
{
  cloud = msg_cloud_;
}

sensor_msgs::PointCloud2 CloudHandler::get()
{
  return msg_cloud_;
}

void CloudHandler::updateCloud(sensor_msgs::PointCloud2 cloud, std::string name)
{
    ROS_DEBUG_NAMED("cloud_handler", "Downsampling cloud...");
    ros::WallTime begin = ros::WallTime::now();
    // Create the filtering object
    sensor_msgs::PointCloud2::Ptr ptr (new sensor_msgs::PointCloud2 (cloud));

    //sensor_msgs::PointCloud2 temp =  cloud_server_client_.getResult()->cloud;
    //sensor_msgs::PointCloud2::Ptr ptr(&temp);
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
    sor.setInputCloud (ptr);
    sor.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
    sor.filter (msg_cloud_);
    ROS_DEBUG_NAMED("cloud_handler", "Downsampling took %.3lf ms", (ros::WallTime::now() - begin).toSec()*1000.0);
    //msg_cloud_ = cloud_server_client_.getResult()->cloud;
    ptr.reset();
    saveCloudAndNormals();
    makeMarker(voxel_size_ * 1.8);

    std_msgs::String flag;
    flag.data = name;
    pub_refresh_flag_.publish(flag);
}

void CloudHandler::refresh()
{
  ROS_DEBUG_NAMED("cloud_handler", "Processing menu-callback refresh, topic [%s]", topic_.c_str());
  refresh(topic_);
}

void CloudHandler::refresh(const std::string &topic)
{
  topic_ = topic;

  ROS_DEBUG_NAMED("cloud_handler", "Sending request for cloud on topic [%s]", topic_.c_str());
  point_cloud_server::StoreCloudGoal cloud_goal;
  cloud_goal.action = cloud_goal.GET;
  cloud_goal.topic = topic_;
  cloud_goal.storage_frame_id =   cloud_frame_;
  cloud_goal.result_frame_id =    cloud_frame_;
  cloud_goal.name = "interactive_manipulation_snapshot";
  cloud_server_client_.sendGoalAndWait(cloud_goal, ros::Duration(15.0), ros::Duration(5.0));

  if(cloud_server_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_DEBUG_NAMED("cloud_handler", "Got response from server!");
    updateCloud(cloud_server_client_.getResult()->cloud, cloud_goal.name);
  }
  else
  {
    ROS_DEBUG_NAMED("cloud_handler", "Server did not succeed, status %s", cloud_server_client_.getState().toString().c_str());
  }
}

void CloudHandler::clear()
{
  marker_server_.erase(marker_name_);
  marker_server_.applyChanges();
}


void CloudHandler::makeMarker(float size)
{
  InteractiveMarker int_marker;
  int_marker.name = marker_name_;

  Marker marker;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.frame_locked = false;

  if(cloud_pts_->points.size())
  {
    int_marker.header = msg_cloud_.header;
    int_marker.header.stamp = ros::Time(0);
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;

    int num_points = cloud_pts_->points.size();
    marker.points.resize( num_points );
    marker.colors.resize( num_points );

    //ROS_INFO_STREAM( "Adding point cluster. #points=" << num_points << " frame=" << msg_cloud_.header.frame_id);

    //pcl::PointCloud<PointT>::Ptr &cloud = cloud_pts_;
    for ( int i=0; i<num_points; i++)
    {
      marker.points[i].x = cloud_pts_->points[i].x;
      marker.points[i].y = cloud_pts_->points[i].y;
      marker.points[i].z = cloud_pts_->points[i].z;
      marker.colors[i].r = cloud_pts_->points[i].r/255.;
      marker.colors[i].g = cloud_pts_->points[i].g/255.;
      marker.colors[i].b = cloud_pts_->points[i].b/255.;
      marker.colors[i].a = 1.0;
    }
  }

  InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;

  control.markers.push_back( marker );

  int_marker.controls.push_back( control );

  marker_server_.insert( int_marker, boost::bind( &CloudHandler::leftClickPoint, this, _1 ),
                         visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK);
  menu_handler_.apply( marker_server_, marker_name_ );
  marker_server_.applyChanges();
  ROS_INFO("made interactive marker for cloud");
}


//}
