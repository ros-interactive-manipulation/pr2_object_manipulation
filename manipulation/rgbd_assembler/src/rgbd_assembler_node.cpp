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
#include <sensor_msgs/PointCloud.h>

#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>

#include "rviz_interaction_tools/image_tools.h"

#include "rgbd_assembler/msg_saver.h"
#include "rgbd_assembler/utils.h"
#include "rgbd_assembler/RgbdAssembly.h"

#include <limits>
#include <math.h>


namespace rgbd_assembler {

  class RgbdAssembler {
    
    typedef geometry_msgs::Point32 Point;

  private:
    //! The node handle
    ros::NodeHandle root_nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
  
    ros::ServiceServer rgbd_srv_;
  
    tf::TransformListener listener_;
  
    
    //------------------ Callbacks -------------------

    //! Callback for service calls
    bool rgbdServiceCallback(RgbdAssembly::Request &request,
			     RgbdAssembly::Response &response);
  
    //------------------ Individual processing steps -------
    bool assembleSensorData(ros::Duration time_out);
  
    //------------------- Complete processing -----
  
    // reconstruct point cloud and fill 3D, 2D and RGB channels
    bool assembleRgbd();
    
    void getPointCloudFromIm(sensor_msgs::PointCloud2 &point_cloud, 
			     const stereo_msgs::DisparityImage &disparity_image,
			     const sensor_msgs::CameraInfo &narrow_cam_info);
    
    void projectPointCloud( sensor_msgs::PointCloud2 &point_cloud,
			    const sensor_msgs::CameraInfo &cam_info );

    void getPixel( const sensor_msgs::PointCloud2 &point_cloud,
		   int i, Point &p);
  
    void extractRgbVals( sensor_msgs::PointCloud2 &point_cloud, 
			 const sensor_msgs::Image &wide_img,
			 const sensor_msgs::Image &narrow_img );
    
    // ------------------- The Data --------------
  
    sensor_msgs::Image          narrow_image_;
    sensor_msgs::Image          wide_image_;
    stereo_msgs::DisparityImage disparity_image_;
    sensor_msgs::CameraInfo     narrow_cam_info_;
    sensor_msgs::CameraInfo     wide_cam_info_;
    sensor_msgs::PointCloud2    point_cloud_;
    
  public:

    RgbdAssembler() :
      root_nh_(""), priv_nh_("~")
    {
      rgbd_srv_ = root_nh_.advertiseService(root_nh_.resolveName("/rgbd_assembly"),
					    &RgbdAssembler::rgbdServiceCallback, this);
      
      
      ROS_INFO("RGB-D Assembler node started");
    }

    ~RgbdAssembler()
    {}

  };

  bool RgbdAssembler::rgbdServiceCallback(RgbdAssembly::Request &request,
					  RgbdAssembly::Response &response)
  {
    ros::Time start_time = ros::Time::now();
    if (!assembleSensorData(ros::Duration(15.0))) return false;
    ROS_INFO_STREAM("RGBD Assembler: gathered sensor data after " << ros::Time::now() - start_time << " seconds");
  
    if (!priv_nh_.ok()) 
      return false;
  
    if (!assembleRgbd()) {
      ROS_INFO("RGB-D assembly has not succeeded;");
      response.result = response.OTHER_ERROR;
    } else {
      response.point_cloud     = point_cloud_;
      response.image           = narrow_image_;
      response.disparity_image = disparity_image_;
      response.camera_info     = narrow_cam_info_;
      response.result = response.SUCCESS;
      ROS_INFO_STREAM("RGBD Assembler: finished assembly after " << ros::Time::now() - start_time << " seconds");
    }  
    return true;
  }

  bool RgbdAssembler::assembleSensorData(ros::Duration time_out) 
  {
    ROS_INFO("RGB-D Assembly: waiting for messages...");
    
    MsgSaver< sensor_msgs::Image > 
      recent_wide_image("/wide_stereo/left/image_rect_color");
    MsgSaver< sensor_msgs::Image > 
      recent_image("/narrow_stereo/left/image_rect");
    MsgSaver< stereo_msgs::DisparityImage > 
      recent_disparity_image("/narrow_stereo_textured/disparity");
    MsgSaver< sensor_msgs::CameraInfo > 
      recent_camera_info("/narrow_stereo_textured/left/camera_info");
    MsgSaver< sensor_msgs::CameraInfo > 
      recent_wide_camera_info("/wide_stereo/left/camera_info");
  
    ros::Time start_time = ros::Time::now();

    //ros::spin() is called in the main thread, so we just have to wait..
    while ( (!recent_disparity_image.hasMsg() || 
	     !recent_image.hasMsg() || 
	     !recent_camera_info.hasMsg() || 
	     !recent_wide_camera_info.hasMsg() || 
	     !recent_wide_image.hasMsg())
	    && priv_nh_.ok()) {
      
      ros::spinOnce();
  
      ros::Time current_time = ros::Time::now();
      if (time_out >= ros::Duration(0) && current_time - start_time >= time_out) {
	ROS_ERROR("Timed out while waiting for sensor data.");
	return false;
      }
      ros::Duration(0.1).sleep();
    }
    if (!priv_nh_.ok()) return false;
  
    narrow_image_    = *recent_image.getMsg();
    wide_image_      = *recent_wide_image.getMsg();
    disparity_image_ = *recent_disparity_image.getMsg();
    narrow_cam_info_ = *recent_camera_info.getMsg();
    wide_cam_info_   = *recent_wide_camera_info.getMsg();
  
    return true;
  }

  bool RgbdAssembler::assembleRgbd() {
    
    ROS_INFO("Reconstructing point cloud");
    
    // Make non-dense point cloud from image coordinates
    getPointCloudFromIm(point_cloud_, disparity_image_, narrow_cam_info_);

    int w = point_cloud_.width;
    int h = point_cloud_.height;

    // annoying conversion between point cloud types
    sensor_msgs::PointCloud2::ConstPtr point_cloud_ptr 
      = boost::make_shared<const  sensor_msgs::PointCloud2> (point_cloud_);
    sensor_msgs::PointCloud old_cloud;  
    sensor_msgs::convertPointCloud2ToPointCloud (*point_cloud_ptr, old_cloud);

    // transform image plane to wide stereo frame
    try
      {
	ROS_DEBUG("Transforming point cloud from frame %s into frame %s", 
		  point_cloud_.header.frame_id.c_str(),
		  "wide_stereo_optical_frame");
	listener_.transformPointCloud("wide_stereo_optical_frame", old_cloud, old_cloud);    
      }
    catch (tf::TransformException ex)
      {
	ROS_ERROR("Failed to transform point cloud from frame %s into frame %s; error %s", 
		  old_cloud.header.frame_id.c_str(),
		  "wide_stereo_optical_frame",
		  ex.what());
	return false;
      }
    
    sensor_msgs::PointCloud2 converted_cloud; 
    sensor_msgs::convertPointCloudToPointCloud2 (old_cloud, point_cloud_);
    
    // setting back dimensions of point cloud to width and height of the image
    point_cloud_.height = h;
    point_cloud_.width  = w;

    // project cloud to wide field stereo and add rgb channel to cloud
    projectPointCloud(point_cloud_, wide_cam_info_);
    
    extractRgbVals(point_cloud_, wide_image_, narrow_image_);
    
    return true;
  }

  void RgbdAssembler::getPointCloudFromIm(sensor_msgs::PointCloud2 &point_cloud, 
					  const stereo_msgs::DisparityImage &disparity_image,
					  const sensor_msgs::CameraInfo &narrow_cam_info)
  {
    int w = narrow_cam_info.width;
    int h = narrow_cam_info.height;
     
    // Generate PointCloud2 data structure
    point_cloud.header       = narrow_cam_info.header;
    point_cloud.height       = h;
    point_cloud.width        = w;
    point_cloud.is_dense     = false;
    point_cloud.is_bigendian = false;
    // size of field is 3 for x, y, z and 3 for rgb and u,v
    point_cloud.fields.resize( 3 + 3);
    point_cloud.fields[0].name = "x"; point_cloud.fields[1].name = "y"; point_cloud.fields[2].name = "z";
    point_cloud.fields[3].name = "rgb"; point_cloud.fields[4].name = "u"; point_cloud.fields[5].name = "v";
    int offset = 0;
    for (size_t d = 0; d < point_cloud.fields.size (); ++d, offset += 4) {
      point_cloud.fields[d].offset = offset;
      point_cloud.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      point_cloud.fields[d].count  = 1;
    }
    
    point_cloud.point_step = offset;
    point_cloud.row_step   = point_cloud.point_step * w;
    
    point_cloud.data.resize (w * h * point_cloud.point_step);
    
    for (int i=0; i<h; i++) {
      for (int j=0; j<w; j++)  {
	int adr = i * w + j;
	float x, y, z;
	if (rviz_interaction_tools::hasDisparityValue(disparity_image, i, j)) {
	  
	  rviz_interaction_tools::getPoint( disparity_image, narrow_cam_info_, i, j, x, y, z);
	   
	  memcpy (&point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[0].offset], &x, sizeof (float));
	  memcpy (&point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[1].offset], &y, sizeof (float));
	  memcpy (&point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[2].offset], &z, sizeof (float));

	} else {

	  float nan = std::numeric_limits<float>::quiet_NaN();
	  memcpy (&point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[0].offset], &nan, sizeof (float));
	  memcpy (&point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[1].offset], &nan, sizeof (float));
	  memcpy (&point_cloud.data[adr * point_cloud.point_step + point_cloud.fields[2].offset], &nan, sizeof (float));

	}
      }
    }
  }


  void RgbdAssembler::projectPointCloud(sensor_msgs::PointCloud2 &point_cloud,
					const sensor_msgs::CameraInfo &cam_info ){
    
    float fx = cam_info.P[0*4+0];
    float fy = cam_info.P[1*4+1];
    float cx = cam_info.P[0*4+2]; 
    float cy = cam_info.P[1*4+2]; 
    float tx = cam_info.P[0*4+3]; 
    float ty = cam_info.P[1*4+3]; 
    
    int w = cam_info.width;
    int h = cam_info.height;

    for(int x=0; x<w; ++x){
      for(int y=0; y<h; ++y){
	int i = y*w+x;
	float nan;
	memcpy (&nan, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[0].offset], sizeof (float));
	if(!isnan(nan)){
	  
	  float x, y, z;

	  memcpy (&x, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[0].offset], sizeof (float));
	  memcpy (&y, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[1].offset], sizeof (float));
	  memcpy (&z, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[2].offset], sizeof (float));
	  
	  float u = (fx * x + tx)/z + cx;
	  float v = (fy * y + ty)/z + cy;
	  
	  memcpy (&point_cloud.data[i * point_cloud.point_step + point_cloud.fields[4].offset], &u, sizeof (float));
	  memcpy (&point_cloud.data[i * point_cloud.point_step + point_cloud.fields[5].offset], &v, sizeof (float));
	  
	}
      }
    }
  }

  void RgbdAssembler::getPixel( const sensor_msgs::PointCloud2 &point_cloud,
				int i, Point &p){
    float u, v;
    memcpy ( &u, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[4].offset], sizeof (float));
    memcpy ( &v, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[5].offset], sizeof (float));
    
    p.x = u;
    p.y = v;
    p.z = 1.0;
    
  }

  void RgbdAssembler::extractRgbVals(sensor_msgs::PointCloud2 &point_cloud, 
				     const sensor_msgs::Image &wide_img,
				     const sensor_msgs::Image &narrow_img)
  {
    
    ROS_INFO("Size of point cloud in extractRgbVals %d", (int)point_cloud.data.size());

    for(unsigned int x=0; x<narrow_img.width; ++x){
      for(unsigned int y=0; y<narrow_img.height; ++y){
	int i = y * narrow_img.width + x;
	
	float val  = (float)narrow_img.data.at(i)/255.0f;
	float r, g, b;
	
	float nan;
	memcpy (&nan, &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[0].offset], sizeof (float));
	if(!isnan(nan)){
	  Point p;
	  getPixel( point_cloud, i, p);
	  
	  int wide_i = (int)p.y * (int)wide_img.step + (int)p.x*3;
	  r = (float)wide_img.data.at(wide_i+2)/255.0f;
	  g = (float)wide_img.data.at(wide_i+1)/255.0f;
	  b = (float)wide_img.data.at(wide_i+0)/255.0f;
	    
	  float h, s, v;
	  RGBToHSV(r,g,b,h,s,v);
	  HSVToRGB(h,s,val, r, g, b);
	  
	} else {
	  
	  r = val;
	  g = val;
	  b = val;

	}

	float rgb = getRGB(r,g,b);
	memcpy ( &point_cloud.data[i * point_cloud.point_step + point_cloud.fields[3].offset], &rgb, sizeof (float));
      }
    }
  }
  
} // namespace rgbd_assembler

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rgbd_assembler_node");
  rgbd_assembler::RgbdAssembler node;
  ros::spin();
  return 0;
}
