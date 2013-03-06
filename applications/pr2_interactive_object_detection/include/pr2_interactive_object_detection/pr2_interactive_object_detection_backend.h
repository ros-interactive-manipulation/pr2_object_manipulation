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

#ifndef InteractiveObjRecBackend_H
#define InteractiveObjRecBackend_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <interactive_perception_msgs/ObjectRecognitionGuiAction.h>

#include <rgbd_assembler/RgbdAssembly.h>

#include <tabletop_object_detector/TabletopSegmentation.h>
#include <tabletop_object_detector/TabletopObjectRecognition.h>
#include <tabletop_object_detector/Table.h>

#include <tabletop_collision_map_processing/collision_map_interface.h>

#include <object_manipulator/tools/mechanism_interface.h>

#include <pr2_interactive_object_detection/UserCommandAction.h>
#include <pr2_interactive_object_detection/msg_saver.h>

#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

/// @brief Does all the work for the Interactive Object Recognition GUI
class InteractiveObjDetBackend
{
public:

  InteractiveObjDetBackend();
  ~InteractiveObjDetBackend();

private:

  /// react to a request by the user
  void processUserCommand(const pr2_interactive_object_detection::UserCommandGoal::ConstPtr &goal);

  /// assemble all the basic sensor data needed for the detection process
  bool getSensorData( ros::Duration time_out );

  /// do all steps for segmentation
  bool doSegmentation( bool interactive );

  /// call a popup window to let the user do the segmentation
  bool doInteractiveSegmentation( );

  /// call the automatic tabletop segmentation
  bool doAutoSegmentation( );

  /// do all steps for object recognition, based on segmentation
  bool doRecognition( bool interactive );

  /// call automatic recognition service
  bool doAutoRecognition( );

  /// call a popup window, letting the user decide on the recognition results
  bool doInteractiveRecognition();

  /// call the TabletopObjectDetection service
  bool doAutoDetection( );

  /// do all steps for object recognition without segmentation by calling an external service
  bool doDetection( bool interactive );

  /// publish a list of graspable objects
  bool publishResult( int &num_recognized );

  /// delete fit markers and publish an empty result (kills interactive markers)
  void resetMarkers( );

  /// model info console output
  int printObjects(const std::vector<manipulation_msgs::GraspableObject> &objects);

  /// retrieve info from model database
  bool getModelInfo(const household_objects_database_msgs::DatabaseModelPose &model_pose,
        std::string &name, std::string &all_tags);

  /// get pose of from_frame relative to to_frame
  bool getPose( geometry_msgs::Pose &pose, std::string from_frame, std::string to_frame );

  /// modify the given given pose in-place to be relative to target_frame
  bool transformPose( geometry_msgs::PoseStamped &pose, std::string target_frame );

  /// retrieve mesh from model database
  bool getModelMesh( int model_id, shape_msgs::Mesh& mesh );

  /// add the segmented table to the collision map
  bool addTableToCollisionMap(tabletop_object_detector::Table table );

  void statusFeedback( std::string status );
  void abortAction( std::string error );
  void finishAction( std::string result );
  void preemptAction( );

  /// repeatedly try to connect to the given service
  template <class ServiceType>
  bool connectService( ros::ServiceClient& service_client, std::string topic );

  void publishFitMarkers(
      const std::vector<household_objects_database_msgs::DatabaseModelPoseList> &potential_models,
      const tabletop_object_detector::Table &table );

  void clearOldMarkers(std::string frame_id);

  ros::NodeHandle root_nh_;
  ros::NodeHandle priv_nh_;

  ros::ServiceClient rgbd_assembler_client_;

  ros::ServiceClient get_model_description_client_;
  ros::ServiceClient get_model_mesh_client_;

  ros::ServiceClient auto_seg_client_;

  ros::ServiceClient auto_obj_rec_client_;

  ros::ServiceClient tabletop_detection_client_;

  //! Publisher for markers
  ros::Publisher marker_pub_;

  // requests a popup where the user refines detection results
  actionlib::SimpleActionClient<interactive_perception_msgs::ObjectRecognitionGuiAction> *obj_rec_popup_client_;

  // receives commands from the gui control interface
  actionlib::SimpleActionServer<pr2_interactive_object_detection::UserCommandAction> *user_command_server_;

  // collection of the sensor data
  sensor_msgs::Image image_;
  stereo_msgs::DisparityImage disparity_image_;
  sensor_msgs::CameraInfo  camera_info_;
  sensor_msgs::PointCloud2 point_cloud_;

  rgbd_assembler::RgbdAssembly rgbd_assembler_srv_;

  // requests & results of intermediate steps
  tabletop_object_detector::TabletopSegmentationResponse segmentation_result_;
  tabletop_object_detector::TabletopObjectRecognitionResponse recognition_result_;

  tf::TransformListener tf_listener_;

  // used for publishing the result message
  ros::Publisher result_publisher_;

  // for adding the table to the collision map
  tabletop_collision_map_processing::CollisionMapInterface collision_map_interface_;

  //! The current marker being published
  int current_marker_id_;

  std::vector< visualization_msgs::Marker > delete_markers_;

  // if we did interactive segmentation in the first step, we will treat the result differently
  bool segmentation_was_interactive_;

  // parameters
  double min_marker_quality_;

  std::string robot_reference_frame_id_;

  double table_x_;
  double table_y_;
  double table_z_;
};

#endif
