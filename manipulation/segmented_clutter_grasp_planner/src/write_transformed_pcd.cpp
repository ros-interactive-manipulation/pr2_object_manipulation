/*********************************************************************
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

// Author(s): Kaijen Hsiao

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "write_transformed_pcd");
  sensor_msgs::PointCloud2 transformed_cloud;

  if(argc < 3)
  {
    ROS_ERROR("Usage: write_transformed_pcd <point_cloud_topic> <frame_id>");
    exit(-1);
  }
  tf::TransformListener tfl(ros::Duration(20.0));
  //need to let the transform listener build up some transforms
  ros::Duration(1.0).sleep();

  sensor_msgs::PointCloud2::ConstPtr cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(argv[1], ros::Duration(5.0));
  if(!cloud)
  {
    ROS_INFO("No message received on topic %s", argv[1]);
    exit(-2);
  }
  tfl.waitForTransform(argv[2], cloud->header.frame_id, cloud->header.stamp, ros::Duration(2.0));
  pcl_ros::transformPointCloud(argv[2], *cloud, transformed_cloud, tfl);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(transformed_cloud, *cloud_out);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB> ("cloud.pcd", *cloud_out, false);
  return 0;
  
}
