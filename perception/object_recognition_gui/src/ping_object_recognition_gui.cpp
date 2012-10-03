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

#include "interactive_perception_msgs/ObjectRecognitionGuiAction.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_object_recognition_gui");
  ros::NodeHandle nh("~");
  std::string topic("/object_recognition_ui");

  actionlib::SimpleActionClient<interactive_perception_msgs::ObjectRecognitionGuiAction> client(topic, true);
  while(!client.waitForServer(ros::Duration(2.0)) && nh.ok())
  {
    ROS_INFO("Waiting for action client on topic %s", topic.c_str());
  }

  while (nh.ok())
  {
    sensor_msgs::Image::ConstPtr recent_image;
    stereo_msgs::DisparityImage::ConstPtr recent_disparity_image;
    sensor_msgs::CameraInfo::ConstPtr recent_camera_info;

    ROS_INFO("Waiting for messages...");
    std::string image_topic("/narrow_stereo/left/image_rect");
    std::string disparity_image_topic("/narrow_stereo_textured/disparity");
    std::string camera_info_topic("/narrow_stereo_textured/left/camera_info");
    while ((!recent_image || !recent_disparity_image || !recent_camera_info) && nh.ok())
    {
      if (!recent_image)
      {
        ROS_INFO_STREAM("  Waiting for message on topic " << image_topic);
        recent_image = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic, nh, ros::Duration(0.5));
      }
      if (!recent_disparity_image)
      {
        ROS_INFO_STREAM("  Waiting for message on topic " << disparity_image_topic);
        recent_disparity_image = ros::topic::waitForMessage<stereo_msgs::DisparityImage>
          (disparity_image_topic, nh, ros::Duration(0.5));
      }
      if (!recent_camera_info)
      {
        ROS_INFO_STREAM("  Waiting for message on topic " << camera_info_topic);
        recent_camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>
          (camera_info_topic, nh, ros::Duration(0.5));
      }
    }
    if (!nh.ok()) exit(0);
    ROS_INFO("All required messages received; sending goal to object recognition popup action");

    interactive_perception_msgs::ObjectRecognitionGuiGoal or_gui_goal;
    or_gui_goal.image = *recent_image;
    or_gui_goal.camera_info = *recent_camera_info;

    client.sendGoal(or_gui_goal);
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
      ROS_INFO("The object recognition popup action has not succeeded;");
    }
    else
    {    
      ROS_INFO("Gripper click has succeeded;");
    }
    ros::Duration(3.0).sleep();
  }
}
