
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

// Author(s): Jeannette Bohg

#include <ros/ros.h>

#include <string>
#include <vector>

#include "rgbd_assembler/RgbdAssembly.h"

#include "rgbd_assembler/utils.h"

#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>

visualization_msgs::Marker getCloudMarker(const sensor_msgs::PointCloud2 &cloud)
{
  static bool first_time = true;
  if (first_time) {
    srand ( time(NULL) );
    first_time = false;
  }

  //create the marker
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = 0.002;
  marker.scale.y = 0.002;
  marker.scale.z = 1.0;

  marker.color.r = ((double)rand())/RAND_MAX;
  marker.color.g = ((double)rand())/RAND_MAX;
  marker.color.b = ((double)rand())/RAND_MAX;
  marker.color.a = 1.0;
  
  int w = cloud.width;
  int h = cloud.height;
  
  for (int i=0; i<h; i++) {
    for (int j=0; j<w; j++)  {
      int adr = i * w + j;
      float nan;
      memcpy (&nan, &cloud.data[adr * cloud.point_step + cloud.fields[0].offset], sizeof (float));
      if(!isnan(nan)){
	geometry_msgs::Point p;
	float x, y, z;
	memcpy (&x, &cloud.data[adr * cloud.point_step + cloud.fields[0].offset], sizeof (float));
	memcpy (&y, &cloud.data[adr * cloud.point_step + cloud.fields[1].offset], sizeof (float));
	memcpy (&z, &cloud.data[adr * cloud.point_step + cloud.fields[2].offset], sizeof (float));
	p.x = x;
	p.y = y;
	p.z = z;
	
	marker.points.push_back(p);

      }
    }
  }

  return marker;
}

void publishClusterMarkers(const sensor_msgs::PointCloud2 &cloud, const ros::Publisher &marker_pub)
{
  int current_cluster_marker = 1;
  visualization_msgs::Marker cloud_marker =  getCloudMarker(cloud);
  cloud_marker.header = cloud.header;
  cloud_marker.pose.orientation.w = 1;
  cloud_marker.ns = "tabletop_node";
  cloud_marker.id = current_cluster_marker;
  marker_pub.publish(cloud_marker);
}

void fillRgbImage(sensor_msgs::Image &rgb_img, 
		  const sensor_msgs::PointCloud2 &point_cloud)
{

  rgb_img.header = point_cloud.header;
  rgb_img.height = point_cloud.height;
  rgb_img.width =  point_cloud.width;
  rgb_img.encoding = enc::RGB8;
  rgb_img.is_bigendian = false;
  rgb_img.step = 3 * rgb_img.width;
  size_t size = rgb_img.step * rgb_img.height;
  rgb_img.data.resize(size);
    
  for(unsigned int x=0; x<rgb_img.width; ++x){
    for(unsigned int y=0; y<rgb_img.height; ++y){
      int i = y * rgb_img.width + x;
	
      float rgb;
      memcpy ( &rgb, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[3].offset], sizeof (float));
      float r, g, b;
      rgbd_assembler::transformRGB(rgb, r, g, b);
	  
      int wide_i = y * rgb_img.step + x*3;
      rgb_img.data[wide_i+0] = round(r*255.0f);
      rgb_img.data[wide_i+1] = round(g*255.0f);
      rgb_img.data[wide_i+2] = round(b*255.0f);

    }
  }
}

/*! Simply pings the RGB-D Assembler and stores the images*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_rgbd_assembler_node");
  ros::NodeHandle nh_;

   //! Publisher for markers
  ros::Publisher marker_pub_;

   //! Publisher for images
  image_transport::ImageTransport it_(nh_);
  
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>(nh_.resolveName("tabletop_segmentation_markers"), 10);
  image_transport::Publisher image_pub_ = it_.advertise("rgb_out", 1);
  

  std::string service_name("/rgbd_assembly");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh_.ok() ) {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh_.ok()) exit(0);

  rgbd_assembler::RgbdAssembly rgbd_assembler_srv;
  if (!ros::service::call(service_name, rgbd_assembler_srv)) {
      ROS_ERROR("Call to rgbd service failed");
      exit(0);
  }
  if (rgbd_assembler_srv.response.result != rgbd_assembler_srv.response.SUCCESS) {
    ROS_ERROR("RGB-D Assembly service returned error %d", rgbd_assembler_srv.response.result);
    exit(0);
  }
  
  publishClusterMarkers(rgbd_assembler_srv.response.point_cloud, marker_pub_);

  sensor_msgs::Image rgb_img;
  fillRgbImage(rgb_img, rgbd_assembler_srv.response.point_cloud);
  
  ros::Rate loop_rate(5);
  while (nh_.ok()) {
    image_pub_.publish(rgb_img);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return true;
}
