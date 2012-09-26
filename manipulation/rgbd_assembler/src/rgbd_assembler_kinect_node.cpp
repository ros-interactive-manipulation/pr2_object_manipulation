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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>

#include "rgbd_assembler/msg_saver.h"
#include "rgbd_assembler/RgbdAssembly.h"

namespace rgbd_assembler
{

class RgdbAssemblerKinect
{

private:
  //! The node handle
  ros::NodeHandle root_nh_;
  //! Node handle in the private namespace
  ros::NodeHandle priv_nh_;

  ros::ServiceServer rgbd_srv_;

  //------------------ Callbacks -------------------

  //! Callback for service calls
  bool rgbdServiceCallback( RgbdAssembly::Request &request,
      RgbdAssembly::Response &response );

  //------------------ Individual processing steps -------
  bool assembleSensorData( ros::Duration time_out );

  // ------------------- The Data --------------

  sensor_msgs::Image image_;
  stereo_msgs::DisparityImage disparity_image_;
  sensor_msgs::CameraInfo cam_info_;
  sensor_msgs::PointCloud2 point_cloud_;

  std::string image_topic_;
  std::string disparity_image_topic_;
  std::string camera_info_topic_;
  std::string point_cloud_topic_;

  bool use_disparity_image_;

public:

  RgdbAssemblerKinect() :
    root_nh_(""), priv_nh_("~")
  {
    rgbd_srv_ = root_nh_.advertiseService(
        root_nh_.resolveName("/rgbd_assembly"),
        &RgdbAssemblerKinect::rgbdServiceCallback, this);

    priv_nh_.param<std::string>("image_topic", image_topic_, "/head_mount_kinect/depth_registered/points_image");
    priv_nh_.param<std::string>("disparity_image_topic", disparity_image_topic_, "/head_mount_kinect/depth_registered/disparity");
    priv_nh_.param<std::string>("camera_info_topic", camera_info_topic_, "/head_mount_kinect/depth_registered/camera_info");
    priv_nh_.param<std::string>("point_cloud_topic", point_cloud_topic_, "/head_mount_kinect/depth_registered/points");
    priv_nh_.param<bool>("use_disparity_image", use_disparity_image_, true);

    ROS_INFO("RGB-D Kinect Assembler node started");
  }

  ~RgdbAssemblerKinect()
  {
  }

};

bool RgdbAssemblerKinect::rgbdServiceCallback( RgbdAssembly::Request &request,
    RgbdAssembly::Response &response )
{
  if (!assembleSensorData(ros::Duration(15.0)))
    return false;
  ROS_INFO("Assembled Sensor Data");

  if (!priv_nh_.ok())
    return false;

  ROS_INFO("RGB-D Kinectassembly has succeeded;");
  response.point_cloud = point_cloud_;
  response.image = image_;
  if(use_disparity_image_) response.disparity_image = disparity_image_;
  response.camera_info = cam_info_;
  response.result = response.SUCCESS;

  return true;
}

bool RgdbAssemblerKinect::assembleSensorData( ros::Duration time_out )
{
  ROS_INFO("RGB-D Assembly: waiting for messages...");

  MsgSaver<sensor_msgs::Image> recent_image(image_topic_);
  MsgSaver<stereo_msgs::DisparityImage> recent_disparity_image(disparity_image_topic_);
  MsgSaver<sensor_msgs::CameraInfo> recent_camera_info(camera_info_topic_);
  MsgSaver<sensor_msgs::PointCloud2> recent_point_cloud(point_cloud_topic_);
  ros::Time start_time = ros::Time::now();

  while (((use_disparity_image_ && !recent_disparity_image.hasMsg()) || !recent_image.hasMsg()
      || !recent_camera_info.hasMsg() || !recent_point_cloud.hasMsg())
      && priv_nh_.ok())
  {
    ros::spinOnce();

    ros::Time current_time = ros::Time::now();
    if (time_out >= ros::Duration(0) && current_time - start_time >= time_out)
    {
      ROS_ERROR("Timed out while waiting for sensor data.");
      return false;
    }
    ros::Duration(1.0).sleep();
  }
  if (!priv_nh_.ok())
    return false;

  image_ = *recent_image.getMsg();
  if(use_disparity_image_) disparity_image_ = *recent_disparity_image.getMsg();
  cam_info_ = *recent_camera_info.getMsg();
  point_cloud_ = *recent_point_cloud.getMsg();

  image_.header = disparity_image_.header;

  if ( use_disparity_image_ && (image_.width != disparity_image_.image.width || image_.height != disparity_image_.image.height) )
  {
    ROS_ERROR( "Size mismatch between image and disparity image!");
    return false;
  }
  
  if ( image_.width != point_cloud_.width || image_.height != point_cloud_.height )
  {
    ROS_ERROR( "Size mismatch between image and point cloud!");
    return false;
  }
  
  return true;
}

} // namespace rgbd_assembler

int main( int argc, char **argv )
{
  ros::init(argc, argv, "rgbd_assembler_kinect_node");
  rgbd_assembler::RgdbAssemblerKinect node;
  ros::spin();
  return 0;
}
