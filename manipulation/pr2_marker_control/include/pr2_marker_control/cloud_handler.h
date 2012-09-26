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

#ifndef PR2_MARKER_CONTROL_CLOUD_HANDLER
#define PR2_MARKER_CONTROL_CLOUD_HANDLER

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <sensor_msgs/PointCloud2.h>

#include <actionlib/client/simple_action_client.h>
#include <point_cloud_server/StoreCloudAction.h>
#include <pcl/search/kdtree.h>
#include <tf/transform_listener.h>
#include <object_manipulator/tools/mechanism_interface.h>

typedef pcl::PointXYZRGB PointT;


class CloudHandler
{
public:

  CloudHandler( ros::NodeHandle *nh, tf::TransformListener *tfl, 
                std::string marker_name,
                std::string topic_name, std::string server_name,
                object_manipulator::MechanismInterface &mechanism,
                std::string cloud_frame);

  ~CloudHandler();

  //! Clear the cloud stored in this object
  void clear();

  //! Refresh the cloud stored in this object
  void refresh();
  void refresh(const std::string &topic);
  void updateCloud(sensor_msgs::PointCloud2 cloud, std::string name);

  //! Get the cloud stored in this object
  sensor_msgs::PointCloud2 get();
  void get(sensor_msgs::PointCloud2 &cloud);

private:

  typedef interactive_markers::MenuHandler MenuHandler;

  void makeMenu();

  void pickup( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  void makeMarker(float size);

  void saveCloudAndNormals();

  void menuPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void menuFocus( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
  void leftClickPoint( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

  std::string marker_name_, topic_;
  ros::NodeHandle *nh_;
  ros::Publisher pub_left_click_, pub_right_click_, pub_refresh_flag_, pub_focus_;
  tf::TransformListener *tfl_;

  interactive_markers::InteractiveMarkerServer marker_server_;
  interactive_markers::MenuHandler menu_handler_;

  sensor_msgs::PointCloud2 msg_cloud_;
  pcl::PointCloud<PointT>::Ptr cloud_pts_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  pcl::search::KdTree<PointT>::Ptr tree_;
  double voxel_size_;
  bool double_menu_;
  
  object_manipulator::MechanismInterface &mechanism_;
  actionlib::SimpleActionClient<point_cloud_server::StoreCloudAction> cloud_server_client_;
  std::string head_pointing_frame_;
  std::string cloud_frame_;
};

//}

#endif
