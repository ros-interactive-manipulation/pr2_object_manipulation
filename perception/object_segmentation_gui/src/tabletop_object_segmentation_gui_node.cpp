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

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

#include <visualization_msgs/Marker.h>

#include <tabletop_object_detector/TabletopSegmentation.h>

#include <tf/transform_listener.h>

#include "interactive_perception_msgs/ObjectSegmentationGuiAction.h"


class TabletopObjectSegmenterGui
{
private:
  //! The node handle
  ros::NodeHandle root_nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;

  //! Publisher for markers
  ros::Publisher marker_pub_;
  
  ros::ServiceServer os_srv_;

  tf::TransformListener listener_;

  actionlib::SimpleActionClient<interactive_perception_msgs::ObjectSegmentationGuiAction> *os_gui_action_client_;

  //! Used to remember the number of markers we publish so we can delete them later
  int num_markers_published_;
  //! The current marker being published
  int current_marker_id_;

  
  //------------------ Callbacks -------------------

  //! Callback for service calls
  bool segmServiceCallback(tabletop_object_detector::TabletopSegmentation::Request &request,
			   tabletop_object_detector::TabletopSegmentation::Response &response);

  //------------------ Individual processing steps -------
  bool assembleSensorData(interactive_perception_msgs::ObjectSegmentationGuiGoal &goal,
			  ros::Duration time_out);

  //! Converts raw table detection results into a Table message type
  /*
    template <class PointCloudType>
    Table getTable(roslib::Header cloud_header, const tf::Transform &table_plane_trans,
    const PointCloudType &table_points);
  */
  
  //! Publishes rviz markers for the given tabletop clusters
  /*
    template <class PointCloudType>
    void publishClusterMarkers(const std::vector<PointCloudType> &clusters, roslib::Header cloud_header);
  */

  //------------------- Complete processing -----

  //! labels point cloud with the given labels
  /*
    void labelCloud(const sensor_msgs::PointCloud2 &cloud, const sensor_msgs::Image &labels,
    tabletop_object_detector::TabletopSegmentation::Response &response);
  */
  
  //! Clears old published markers and remembers the current number of published markers
  void clearOldMarkers(std::string frame_id);


public:

  TabletopObjectSegmenterGui() :
    root_nh_(""), priv_nh_("~")
  {
    os_srv_ = root_nh_.advertiseService("/interactive_tabletop_segmentation",
                                        &TabletopObjectSegmenterGui::segmServiceCallback, this);
    
    os_gui_action_client_ = 0;
    
    ROS_INFO("Tabletop Object Segmenter Gui node started");
  }

  ~TabletopObjectSegmenterGui()
  {
    delete os_gui_action_client_;
  }

};

bool TabletopObjectSegmenterGui::segmServiceCallback(tabletop_object_detector::TabletopSegmentation::Request &request,
						     tabletop_object_detector::TabletopSegmentation::Response &response)
{
  // reinitializing clients on each call to get around potential actionlib bug
  while(! os_gui_action_client_->waitForServer(ros::Duration(2.0)) && priv_nh_.ok())
    {
      ROS_INFO("Waiting for action client on topic %s", "segmentation_popup");
    }

  if ( os_gui_action_client_ ) delete os_gui_action_client_;
  os_gui_action_client_ = new actionlib::SimpleActionClient<interactive_perception_msgs::ObjectSegmentationGuiAction>
    ("segmentation_popup", true);
  
  interactive_perception_msgs::ObjectSegmentationGuiGoal segm_goal;
  if (!assembleSensorData(segm_goal, ros::Duration(15.0))) return false;
  ROS_INFO("Assembled Sensor Data");
  os_gui_action_client_->sendGoal(segm_goal);
  ROS_INFO("Send Data as goal");
  while (!os_gui_action_client_->waitForResult(ros::Duration(0.5)) && priv_nh_.ok())
    {
      ROS_INFO("Waiting for result from action client on topic %s", "segmentation_popup");
    }
  if (!priv_nh_.ok()) 
    {
      os_gui_action_client_->cancelGoal();
      return false;
    }
  
  if (os_gui_action_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("The segmentation gui action has not succeeded;");
      response.result = response.OTHER_ERROR;
    }
  else
    {    
      ROS_INFO("Segmentation Gui grasp has succeeded;");
      /*
      sensor_msgs::Image labeledSegm = os_gui_action_client_->getResult()->labeledSegments;
      
      labelCloud(segm_goal.sensor_data.point_cloud, labeledSegm, response);
      clearOldMarkers(segm_goal.sensor_data.point_cloud.header.frame_id);
      */
    }
  return true;
}

bool TabletopObjectSegmenterGui::assembleSensorData(interactive_perception_msgs::ObjectSegmentationGuiGoal &goal,
						    ros::Duration time_out) 
{
  sensor_msgs::Image::ConstPtr recent_image;
  stereo_msgs::DisparityImage::ConstPtr recent_disparity_image;
  sensor_msgs::CameraInfo::ConstPtr recent_camera_info;
  sensor_msgs::Image::ConstPtr recent_wide_image;
  sensor_msgs::CameraInfo::ConstPtr recent_wide_camera_info;

  ROS_INFO("Segmentation through User Interaction: waiting for messages...");
  std::string image_topic("/narrow_stereo/left/image_rect");
  std::string disparity_image_topic("/narrow_stereo_textured/disparity");
  std::string camera_info_topic("/narrow_stereo_textured/left/camera_info");
  std::string wide_camera_info_topic("/wide_stereo/left/camera_info");
  std::string wide_image_topic("/wide_stereo/left/image_rect_color");

  ros::Time start_time = ros::Time::now();
  while ((!recent_image || !recent_disparity_image || !recent_camera_info 
	  || !recent_wide_image || !recent_wide_camera_info) && priv_nh_.ok())
    {
      if (!recent_image)
	{
	  ROS_INFO_STREAM("  Waiting for message on topic " << image_topic);
	    recent_image = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, root_nh_, ros::Duration(0.5));
	}
      if (!recent_disparity_image)
	{
	  ROS_INFO_STREAM("  Waiting for message on topic " << disparity_image_topic);
	  recent_disparity_image = ros::topic::waitForMessage<stereo_msgs::DisparityImage>
	    (disparity_image_topic, root_nh_, ros::Duration(0.5));
	}
      if (!recent_camera_info)
	{
	  ROS_INFO_STREAM("  Waiting for message on topic " << camera_info_topic);
	  recent_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
	    (camera_info_topic, root_nh_, ros::Duration(0.5));
	}
      if (!recent_wide_camera_info)
	{
	  ROS_INFO_STREAM("  Waiting for message on topic " << wide_camera_info_topic);
	  recent_wide_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
	    (wide_camera_info_topic, root_nh_, ros::Duration(0.5));
	}
      if (!recent_wide_image)
	{
	  ROS_INFO_STREAM("  Waiting for message on topic " << wide_image_topic);
	  recent_wide_image = ros::topic::waitForMessage<sensor_msgs::Image>(wide_image_topic, root_nh_, 
									     ros::Duration(0.5));
	}
      
      ros::Time current_time = ros::Time::now();
      if (time_out >= ros::Duration(0) && current_time - start_time >= time_out)
	{
	  ROS_INFO("Timed out");
	  return false;
	}
    }
  if (!priv_nh_.ok()) return false;
  ROS_INFO("All required messages received");
  
  goal.image = *recent_image;
  goal.disparity_image = *recent_disparity_image;
  goal.camera_info = *recent_camera_info;
  goal.wide_field = *recent_wide_image;
  goal.wide_camera_info = *recent_wide_camera_info;

  return true;
}
 
/*
void TabletopObjectSegmenterGui::labelCloud(const sensor_msgs::PointCloud2 &cloud, 
					    const sensor_msgs::Image &labels,
					    tabletop_object_detector::TabletopSegmentation::Response &response)
{
  // need association between pixels and point clouds 
  // make a table message -> see tabletop_segmentation processCloud()
  // assume background label=0, table label=1
  // assamble object clusters
  
}
*/

void TabletopObjectSegmenterGui::clearOldMarkers(std::string frame_id)
{
  for (int id=current_marker_id_; id < num_markers_published_; id++)
    {
      visualization_msgs::Marker delete_marker;
      delete_marker.header.stamp = ros::Time::now();
      delete_marker.header.frame_id = frame_id;
      delete_marker.id = id;
      delete_marker.action = visualization_msgs::Marker::DELETE;
      delete_marker.ns = "tabletop_node";
      marker_pub_.publish(delete_marker);
    }
  num_markers_published_ = current_marker_id_;
  current_marker_id_ = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tabletop_segmentation_gui_node");
  TabletopObjectSegmenterGui node;
  ros::spin();
  return 0;
}
