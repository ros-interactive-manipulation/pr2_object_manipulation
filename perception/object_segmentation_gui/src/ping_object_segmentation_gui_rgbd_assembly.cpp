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
#include <sensor_msgs/PointCloud2.h>

#include <stereo_msgs/DisparityImage.h>

#include "interactive_perception_msgs/ObjectSegmentationGuiAction.h"
#include <rgbd_assembler/RgbdAssembly.h>

template <class ServiceType>
bool connectService(  ros::NodeHandle &nh, 
		      ros::ServiceClient& service_client, 
		      std::string topic )
{
  while ( !ros::service::waitForService(topic, ros::Duration(2.0)) && nh.ok() )
    {
    ROS_INFO_STREAM("Waiting for '" << topic << "' service to come up..");
    }
  service_client = nh.serviceClient<ServiceType>(topic, true);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_object_segmentation_gui");
  ros::NodeHandle nh("~");
  std::string topic("/segmentation_popup");

  actionlib::SimpleActionClient<interactive_perception_msgs::ObjectSegmentationGuiAction> client(topic, true);
  while(!client.waitForServer(ros::Duration(2.0)) && nh.ok())
    {
      ROS_INFO("Waiting for action client on topic %s", topic.c_str());
    }
  
  ros::ServiceClient rgbd_assembler_client_;
  rgbd_assembler::RgbdAssembly rgbd_assembler_srv_;

  ROS_INFO("Connecting service..");
  
  if ( !connectService<rgbd_assembler::RgbdAssembly>( nh, 
						      rgbd_assembler_client_, 
						      "/rgbd_assembly" ) ) 
    return -1;

  while (nh.ok())
    {
      if (!rgbd_assembler_client_.call(rgbd_assembler_srv_))
      {
	ROS_ERROR("Call to rgbd assembler service failed");
	return false;
      }
    
    if (rgbd_assembler_srv_.response.result != 
	rgbd_assembler_srv_.response.SUCCESS)
      {
	ROS_ERROR( "RGB-D Assembler service returned error " );
	return false;
      }

    
    interactive_perception_msgs::ObjectSegmentationGuiGoal os_gui_goal;
    os_gui_goal.image = 
      rgbd_assembler_srv_.response.image;    
    os_gui_goal.disparity_image = 
      rgbd_assembler_srv_.response.disparity_image;
    os_gui_goal.camera_info = 
      rgbd_assembler_srv_.response.camera_info;
    os_gui_goal.point_cloud = 
      rgbd_assembler_srv_.response.point_cloud;
    
    client.sendGoal(os_gui_goal);
    while (!client.waitForResult(ros::Duration(0.5)) && nh.ok())
      {
      }
    if (!nh.ok()) 
      {
	client.cancelGoal();
	break;
      }
    
    if (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
	ROS_INFO("The object segmentation action has not succeeded;");
      }
    else
      {    
	ROS_INFO("Object Segmentation has succeeded;");
      }
    ros::Duration(3.0).sleep();
  }
}
