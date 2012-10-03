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

#include "pr2_interactive_manipulation/point_head_view_controller.h"

#include <rviz/display_context.h>
#include <OGRE/OgreSceneManager.h>

#include <OGRE/OgreViewport.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreVector3.h>

#include <visualization_msgs/Marker.h>

namespace pr2_interactive_manipulation
{

PointHeadViewController::PointHeadViewController( rviz::ROSImageTexture &texture, 
                                                  rviz::DisplayContext* context ) :
    rviz::ViewController( ),
    texture_(texture),
    point_head_action_client_(0)
{
  ros::NodeHandle n;
#ifdef DEBUG_DISPLAY
  marker_pub_ = n.advertise<visualization_msgs::Marker>("point_head_view_controller/pointing_direction", 1);
#endif
 // global_orientation_ = Ogre::Quaternion::IDENTITY;
  initialize( context );
}

PointHeadViewController::~PointHeadViewController()
{
  delete point_head_action_client_;
}

void PointHeadViewController::handleMouseEvent( rviz::ViewportMouseEvent& event)
{
  if ( event.leftDown() )
  {
    int width = event.viewport->getActualWidth();
    int height = event.viewport->getActualHeight();

    Ogre::Ray mouse_ray = event.viewport->getCamera()->getCameraToViewportRay(
        (float)event.x / (float)width, (float)event.y / (float)height );

    lookAt( mouse_ray.getPoint(1.0) );
  }

}

void PointHeadViewController::setTopic(const std::string& topic)
{
  delete point_head_action_client_;
  topic_ = topic;

  //remove the trailing '/goal' from the topic

  std::string action_topic = topic_;

  size_t goal_str_pos = topic_.rfind("/goal");
  if (goal_str_pos != std::string::npos)
  {
    action_topic.erase(goal_str_pos);
  }

  point_head_action_client_ = new PointHeadActionClient(action_topic, true);
}

void PointHeadViewController::reset()
{
  if ( !point_head_action_client_ )
  {
    return;
  }

  if ( !texture_.getImage().get() )
  {
    ROS_ERROR( "No image received. Cannot compute look-at point." );
    return;
  }

  point_head_action_client_->waitForServer( ros::Duration( 0.1 ) );

  if ( !point_head_action_client_->isServerConnected() )
  {
    ROS_ERROR( "Point head action client is not connected!" );
    return;
  }

  // send a point head goal which is 5 meters in front of the robot, on the ground.
  pr2_controllers_msgs::PointHeadGoal point_head_goal;

  point_head_goal.pointing_axis.x = 0;
  point_head_goal.pointing_axis.y = 0;
  point_head_goal.pointing_axis.z = 1;

  point_head_goal.pointing_frame = texture_.getImage()->header.frame_id;

  point_head_goal.target.header.frame_id = "base_footprint";
  point_head_goal.target.header.stamp = ros::Time::now();

  point_head_goal.target.point.x = 5;
  point_head_goal.target.point.y = 0;
  point_head_goal.target.point.z = 0;

  point_head_goal.max_velocity = 1.0;

  point_head_action_client_->sendGoal( point_head_goal );
}

void PointHeadViewController::lookAt( const Ogre::Vector3& look_at_point )
{
  if ( !point_head_action_client_ )
  {
    return;
  }

  if ( !texture_.getImage().get() )
  {
    ROS_ERROR( "No image received. Cannot compute look-at point." );
    return;
  }

  point_head_action_client_->waitForServer( ros::Duration( 0.1 ) );

  if ( !point_head_action_client_->isServerConnected() )
  {
    ROS_ERROR( "Point head action client is not connected!" );
    return;
  }

  Ogre::Vector3 look_at_point_robot = look_at_point;

  // send point head goal

  pr2_controllers_msgs::PointHeadGoal point_head_goal;

  point_head_goal.pointing_axis.x = 0;
  point_head_goal.pointing_axis.y = 0;
  point_head_goal.pointing_axis.z = 1;

  point_head_goal.pointing_frame = texture_.getImage()->header.frame_id;

  std::string fixed_frame = context_->getFixedFrame().toStdString();
  point_head_goal.target.header.frame_id = fixed_frame;
  point_head_goal.target.header.stamp = ros::Time::now();

  point_head_goal.target.point.x = look_at_point_robot.x;
  point_head_goal.target.point.y = look_at_point_robot.y;
  point_head_goal.target.point.z = look_at_point_robot.z;

  point_head_goal.max_velocity = 1.0;

  point_head_action_client_->sendGoal( point_head_goal );

#ifdef DEBUG_DISPLAY

  ROS_INFO( "Looking at position %f %f %f in pointing frame %s (fixed frame is %s)", point_head_goal.target.point.x,
      point_head_goal.target.point.y, point_head_goal.target.point.z,
      point_head_goal.pointing_frame.c_str(),
      point_head_goal.target.header.frame_id.c_str() );

  // publish debug marker

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = context_->getFixedFrame().toStdString();
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "PointHeadViewController";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::SPHERE;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = look_at_point_robot.x;
  marker.pose.position.y = look_at_point_robot.y;
  marker.pose.position.z = look_at_point_robot.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  marker_pub_.publish(marker);

#endif
}

}
