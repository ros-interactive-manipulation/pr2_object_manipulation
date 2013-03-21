/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include "pr2_interactive_manipulation/point_tools.h"

#include <rviz/properties/string_property.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/features/integral_image_normal.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreViewport.h>
#include <OGRE/OgreCamera.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/properties/int_property.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>





namespace pr2_interactive_manipulation
{

LookAtTool::LookAtTool()
{
  topic_property_->setStdString("/rviz/look_here");
};

SetGripperTool::SetGripperTool()
{
  createProperties();
  topic_property_->setStdString("/rviz/set_gripper_point"); 
  topic_property_pose_->setStdString("/cloud_click_point");
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_pose_->getStdString(), 1 );
};

void SetGripperTool::createProperties()
{
  topic_property_pose_ = new rviz::StringProperty("Pose Topic",
                                                  "/cloud_click_point", "Topic on which to publish poses.",
                                                  getPropertyContainer(),
                                                  SLOT( updatePoseTopic() ), this);

  // number of pixels around point to sample for depth image
  normal_box_padding_ = new rviz::IntProperty("Normal box padding",
                                              7,
                                              "Size of box around clicked point to sample for normal");
  
  // Maximum far distance for depth image 
  far_clip_distance_ = new rviz::FloatProperty("Far Clip Distance",
                                              50.0,
                                              "Maximum distance of depth selection");

  
}

void SetGripperTool::updatePoseTopic()
{
  pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_pose_->getStdString(), 1 );
}




int SetGripperTool::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  int flags = 0;  
  Ogre::Vector3 pos;

  // Test if the mouse is over anything
  bool success = context_->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, pos );
  setCursor( success ? hit_cursor_ : std_cursor_ );

  // If we are actually over a point, set the tooltip appropriately
  if ( success )
  {
    std::ostringstream s;
    s << "<b>Left-Click:</b> Select this point.";
    s.precision(3);
    s << " [" << pos.x << "," << pos.y << "," << pos.z << "]";
    setStatus( s.str().c_str() );
    
    // On click, get a patch of points in the region, and calculate the normal to the point.
    if( event.leftUp() )
    {
      tf::Vector3 normal_vec(0,1,0); // Default normal

      // Set maximum depth for projection
      float starting_max_depth = event.viewport->getCamera()->getFarClipDistance();
      event.viewport->getCamera()->setFarClipDistance(far_clip_distance_->getFloat());

      
      int bp = normal_box_padding_->getInt();
      unsigned int box_width(2*bp+1);
      
      // Set up ordered point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>(box_width, box_width));
      cloud->reserve(box_width * box_width);
      
      // Get patch of points in region around click point
      std::vector<Ogre::Vector3> box_pos;
      bool success_box = context_->getSelectionManager()->get3DPatch(event.viewport, event.x - bp, event.y - bp, box_width, box_width, true, box_pos);
      
      // Pack ogre points in to point cloud
      for (unsigned int i = 0; i < box_pos.size(); ++ i)
      {
        (*cloud)[i] = pcl::PointXYZ(box_pos[i].x, box_pos[i].y, box_pos[i].z);         
      }
        

      // If we retrieved a significant number of points, calculate the actual normal
      if(box_pos.size() > 3)
      {
       pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
       
       Ogre::Vector3 camera_pos = event.viewport->getCamera()->getDerivedPosition();

       ne.setViewPoint(camera_pos.x, camera_pos.y, camera_pos.z);
       ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
       ne.setMaxDepthChangeFactor(0.02f);
       ne.setNormalSmoothingSize(10.0f);
       ne.setRectSize(bp - 1, bp -1);
       ne.setInputCloud(cloud);

       // Get index of center point
       unsigned center_point_index = bp + ( box_width ) * bp;
       pcl::Normal normal_point;
       ne.computePointNormal(bp, bp, center_point_index, normal_point);

       // If there was a normal point, generate normal and make sure it points in the right direction.
       if (!std::isnan(normal_point.normal_x)){
         
         tf::Vector3 point_to_head(tf::Vector3(camera_pos.x - pos.x, camera_pos.y - pos.y, camera_pos.z - pos.z).normalized());
         
         normal_vec = tf::Vector3(normal_point.normal_x, normal_point.normal_y, normal_point.normal_z);           
         
         // Flip normal if necessary.
         if(point_to_head.dot(normal_vec) < 0)
           normal_vec *= -1;
       }
       else
       {
         ROS_WARN("Failed to find normal - using default %f %f %f", normal_vec.x(), normal_vec.y(), normal_vec.z());         
       } 
      }  
                   
      // Generate pose from point and normal -- Maybe do something smarter to align 
      // gripper to curvature.
      geometry_msgs::PoseStamped ps;
    
      ps.pose.position.x = pos.x;
      ps.pose.position.y = pos.y;
      ps.pose.position.z = pos.z;
      ps.header.frame_id = context_->getFixedFrame().toStdString();
      ps.header.stamp = ros::Time::now();
      
      tf::Vector3 Z = normal_vec.normalized();
      tf::Vector3 Y = normal_vec.cross(tf::Vector3(0,0,1)).normalized();
      tf::Vector3 X = Y.cross(normal_vec).normalized();
      tf::Matrix3x3 mat(X.x(), Y.x(), Z.x(),
                        X.y(), Y.y(), Z.y(),
                        X.z(), Y.z(), Z.z());
      tf::Quaternion q;
      mat.getRotation(q);
      tf::quaternionTFToMsg(q, ps.pose.orientation);
          
      pub_pose_.publish( ps );     
      
      if ( auto_deactivate_property_->getBool() )
      {
        flags |= Finished;
      }

      // Reset clipping distance to default distance.
      event.viewport->getCamera()->setFarClipDistance(starting_max_depth); 
    }
     
  }
  else
  {
    setStatus( "Move over an object to select the target point." );
  }
  

  return flags;
  
}

NavigateToTool::NavigateToTool()
{
  topic_property_->setStdString("/rviz/navigate_to");
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( pr2_interactive_manipulation::LookAtTool, rviz::Tool )
PLUGINLIB_EXPORT_CLASS( pr2_interactive_manipulation::SetGripperTool, rviz::Tool )
PLUGINLIB_EXPORT_CLASS( pr2_interactive_manipulation::NavigateToTool, rviz::Tool )
