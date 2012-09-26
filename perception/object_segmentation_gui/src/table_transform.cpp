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

#include "object_segmentation_gui/table_transform.h"

namespace object_segmentation_gui {
  
  TableTransform::TableTransform()
  {}

  /*
  TableTransform::TableTransform( const sensor_msgs::CameraInfo &cam_info, float baseline, float up_direction)
    :  cam_info_(cam_info)
    ,  baseline_(baseline)
    ,  up_direction_(up_direction)
  {}
  */
  
  TableTransform::~TableTransform()
  {}

  void TableTransform::setParams(const sensor_msgs::CameraInfo &cam_info, float baseline, float up_direction)
  {
    cam_info_     = cam_info;
    baseline_     = baseline;
    up_direction_ = up_direction;
  }

  
  // convert plane parameters in disparity space to 3D space
  tabletop_object_detector::Table TableTransform::get3DTable(float alpha, 
							     float beta, 
							     float gamma, 
							     sensor_msgs::PointCloud &table_points,
							     std_msgs::Header table_header){
   
    
    pcl::ModelCoefficients table_coefficients = 
      convertTo3DPlane( cam_info_, alpha, beta, gamma);
    
    ROS_INFO ("Table found with %d inliers: [%f %f %f %f].", 
	      (int)table_points.points.size (),
	      table_coefficients.values[0], table_coefficients.values[1], 
	      table_coefficients.values[2], table_coefficients.values[3]);
    
    tf::Transform table_plane_trans = getPlaneTransform (table_coefficients, up_direction_);
    if (!transformPlanePoints( table_plane_trans, table_points)) {
      ROS_WARN("Table could not be computed");
      tabletop_object_detector::Table table;
      geometry_msgs::Pose table_pose;
      table_pose.position.x=table_pose.position.y=table_pose.position.z=table_pose.orientation.x
	=table_pose.orientation.y=table_pose.orientation.z=table_pose.orientation.w=0;
      return table;
    }
    
    return getTable<sensor_msgs::PointCloud>(table_header, table_plane_trans, table_points);
  }

  
  pcl::ModelCoefficients TableTransform::convertTo3DPlane(const sensor_msgs::CameraInfo &camera_info, 
							  const float alpha, 
							  const float beta, 
							  const float gamma )
  {
    float fx = camera_info.P[0*4+0]; 
    float cx = camera_info.P[0*4+2]; 
    float cy = camera_info.P[1*4+2]; 
    
    float A = alpha/baseline_;
    float B = beta/baseline_;
    float C = gamma/(baseline_ * fx) + (A*cx + B*cy)/fx;
    
    
    // calc disparity for principal point
    float disp = alpha*cx + beta*cy + gamma;
    // convert point to 3D
    float Z = baseline_*fx/disp;
    float D = C * Z;
    ROS_DEBUG("Apart from rounding error D should be 1 and is %f", D);

    float NMod = sqrt(A*A+B*B+C*C);
    
    A/=NMod;
    B/=NMod;
    C/=NMod;
    D/=NMod;
    
    pcl::ModelCoefficients table_coefficients;
    table_coefficients.values.push_back(A);
    table_coefficients.values.push_back(B);
    table_coefficients.values.push_back(C); 
    table_coefficients.values.push_back(D);

    return table_coefficients;
  }

  /*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
  tf::Transform TableTransform::getPlaneTransform (pcl::ModelCoefficients coeffs, 
						   double up_direction)
  {
    ROS_ASSERT(coeffs.values.size() > 3);
    double a = coeffs.values[0], b = coeffs.values[1], c = coeffs.values[2], d = coeffs.values[3];
    //asume plane coefficients are normalized
    tf::Vector3 position(a*d, b*d, c*d);
    tf::Vector3 z(a, b, c);
    //make sure z points "up"
    ROS_DEBUG("z.dot: %0.3f", z.dot(tf::Vector3(0,0,1)));
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    if ( z.dot( tf::Vector3(0, 0, up_direction) ) < 0)
      {
	z = -1.0 * z;
	ROS_DEBUG("flipped z");
      }
    ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
    
    //try to align the x axis with the x axis of the original frame
    //or the y axis if z and x are too close too each other
    tf::Vector3 x(1, 0, 0);
    if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = tf::Vector3(0, 1, 0);
    tf::Vector3 y = z.cross(x).normalized();
    x = y.cross(z).normalized();
    
    tf::Matrix3x3 rotation;
    rotation[0] = x; 	// x
    rotation[1] = y; 	// y
    rotation[2] = z; 	// z
    rotation = rotation.transpose();
    tf::Quaternion orientation;
    rotation.getRotation(orientation);
    return tf::Transform(orientation, position);
  }

  bool TableTransform::transformPlanePoints ( const tf::Transform& table_plane_trans,
					      sensor_msgs::PointCloud &table_points )
  {
    // Transform the data
    tf::TransformListener listener;
    tf::StampedTransform  table_pose_frame(table_plane_trans, table_points.header.stamp, 
					   table_points.header.frame_id, "table_frame");
    listener.setTransform(table_pose_frame);
    std::string error_msg;
    
    if (!listener.canTransform("table_frame", table_points.header.frame_id, 
			       table_points.header.stamp, &error_msg))
      {
	ROS_ERROR("Cannot transform point cloud from frame %s to table frame; error %s", 
		  table_points.header.frame_id.c_str(), error_msg.c_str());
	return false;
      }
    try
      {
	listener.transformPointCloud("table_frame", table_points, table_points);
      }
    catch (tf::TransformException ex)
      {
	ROS_ERROR("Failed to transform point cloud from frame %s into table_frame; error %s", 
		  table_points.header.frame_id.c_str(), ex.what());
	return false;
      }
    table_points.header.frame_id = "table_frame";
    return true;
  }

  template <class PointCloudType>
  tabletop_object_detector::Table TableTransform::getTable(std_msgs::Header cloud_header,
							   const tf::Transform &table_plane_trans, 
							   const PointCloudType &table_points)
  {
    tabletop_object_detector::Table table;
    
    //get the extents of the table
    if (!table_points.points.empty()) 
      {
	table.x_min = table_points.points[0].x;
	table.x_max = table_points.points[0].x;
	table.y_min = table_points.points[0].y;
	table.y_max = table_points.points[0].y;
      }  
    for (size_t i=1; i<table_points.points.size(); ++i) 
      {
	if (table_points.points[i].x<table.x_min && table_points.points[i].x>-3.0) 
	  table.x_min = table_points.points[i].x;
	if (table_points.points[i].x>table.x_max && table_points.points[i].x< 3.0) 
	  table.x_max = table_points.points[i].x;
	if (table_points.points[i].y<table.y_min && table_points.points[i].y>-3.0) 
	  table.y_min = table_points.points[i].y;
	if (table_points.points[i].y>table.y_max && table_points.points[i].y< 3.0) 
	  table.y_max = table_points.points[i].y;
      }
    
    geometry_msgs::Pose table_pose;
    tf::poseTFToMsg(table_plane_trans, table_pose);

    ROS_DEBUG("Table Pose: [%f %f %f] [%f %f %f %f] ", 
	      table_pose.position.x, table_pose.position.y, table_pose.position.z,
	      table_pose.orientation.x, table_pose.orientation.y, table_pose.orientation.z, 
	      table_pose.orientation.w);

    table.pose.pose = table_pose;
    table.pose.header = cloud_header;
    
    return table;
  }


} // object_segmentation_gui


