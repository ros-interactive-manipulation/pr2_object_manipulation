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

#ifndef TABLE_TRANSFORM
#define TABLE_TRANSFORM

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>

#include "pcl/ModelCoefficients.h"
#include "tabletop_object_detector/Table.h"

#include <tf/transform_listener.h>

namespace object_segmentation_gui {
  
  class TableTransform
  {
  public:
    
    TableTransform();
    ~TableTransform();
    
    void setParams(const sensor_msgs::CameraInfo &cam_info, float baseline, float up_direction);

    tabletop_object_detector::Table get3DTable( float alpha, float beta, float gamma, 
						sensor_msgs::PointCloud &table_points,
						std_msgs::Header table_header);
    
  protected:
    
  private:
    
    pcl::ModelCoefficients convertTo3DPlane(const sensor_msgs::CameraInfo &camera_info, 
					    const float alpha, const float beta, const float gamma );
    
    tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs, double up_direction);

    bool transformPlanePoints ( const tf::Transform& table_plane_trans,
				sensor_msgs::PointCloud &table_points );
    
    template <class PointCloudType>
      tabletop_object_detector::Table getTable(std_msgs::Header cloud_header,
					       const tf::Transform &table_plane_trans, 
					       const PointCloudType &table_points);

    sensor_msgs::CameraInfo cam_info_;
   
    float baseline_;
    float up_direction_;

    //! A tf transform listener
    tf::TransformListener listener_;
  };
}

#endif // TABLE_TRANSFORM
