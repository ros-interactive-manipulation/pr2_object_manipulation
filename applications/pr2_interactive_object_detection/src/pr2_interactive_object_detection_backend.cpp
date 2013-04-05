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

#include "pr2_interactive_object_detection/pr2_interactive_object_detection_backend.h"

#include <manipulation_msgs/GraspableObjectList.h>

#include <household_objects_database_msgs/GetModelDescription.h>
#include <household_objects_database_msgs/GetModelMesh.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <stereo_msgs/DisparityImage.h>

#include <interactive_perception_msgs/ObjectSegmentationGuiAction.h>

#include <tabletop_object_detector/marker_generator.h>
#include <tabletop_object_detector/TabletopDetection.h>

using namespace tabletop_object_detector;
using namespace interactive_perception_msgs;
using namespace manipulation_msgs;
using namespace object_manipulation_msgs;
using namespace household_objects_database_msgs;
using namespace pr2_interactive_object_detection;
using namespace rgbd_assembler;

InteractiveObjDetBackend::InteractiveObjDetBackend() :
  root_nh_(""),
  priv_nh_("~"),
  tf_listener_( ros::Duration(60), true )
{
  obj_rec_popup_client_ = new actionlib::SimpleActionClient<ObjectRecognitionGuiAction>
    ("object_recognition_popup", true);

  user_command_server_ = new actionlib::SimpleActionServer<UserCommandAction>
    (root_nh_, "object_detection_user_command", boost::bind(&InteractiveObjDetBackend::processUserCommand, this, _1), false );
  user_command_server_->start();

  ROS_INFO("Connecting services..");

  if ( !connectService<RgbdAssembly>( rgbd_assembler_client_, "rgbd_assembly" ) ) return;

  if ( !connectService<TabletopSegmentation>( auto_seg_client_, "tabletop_segmentation" ) ) return;

  if ( !connectService<TabletopObjectRecognition>( auto_obj_rec_client_, "tabletop_object_recognition" ) ) return;

  if ( !connectService<TabletopDetection>( tabletop_detection_client_, "tabletop_detection" ) ) return;

  if ( !connectService<GetModelDescription>
       ( get_model_description_client_, "objects_database_node/get_model_description" ) ) return;

  if ( !connectService<GetModelMesh>
       ( get_model_mesh_client_, "objects_database_node/get_model_mesh" ) ) return;

  result_publisher_ = root_nh_.advertise<GraspableObjectList>("interactive_object_recognition_result", 1, true );

  marker_pub_ = priv_nh_.advertise<visualization_msgs::Marker>("object_markers", 1);

  priv_nh_.param<double>("min_marker_quality", min_marker_quality_, 0.003);
  priv_nh_.param<std::string>("robot_reference_frame_id", robot_reference_frame_id_, "base_link");
  priv_nh_.param<double>("table_x", table_x_, 1);
  priv_nh_.param<double>("table_y", table_y_, 0);
  priv_nh_.param<double>("table_z", table_z_, 0);

  ROS_INFO("Backend server for Interactive Object Detection GUI started");
}


InteractiveObjDetBackend::~InteractiveObjDetBackend()
{
  delete obj_rec_popup_client_;
  delete user_command_server_;
}


template <class ServiceType>
bool InteractiveObjDetBackend::connectService( ros::ServiceClient& service_client, std::string topic )
{
  /*
  if ( service_client.isValid() )
    return;

  while ( !ros::service::waitForService(topic, ros::Duration(2.0)) && priv_nh_.ok() )
  {
    ROS_INFO_STREAM("Waiting for '" << topic << "' service to come up..");
  }
  if ( !root_nh_.ok() ) return false;
  */

  //we only need non-persistent connections here
  service_client = root_nh_.serviceClient<ServiceType>(topic, false);
  return true;
}


void InteractiveObjDetBackend::processUserCommand(const UserCommandGoal::ConstPtr &goal)
{
  switch (goal->request)
  {
    case UserCommandGoal::SEGMENT:
      doSegmentation( goal->interactive );
      break;
    case UserCommandGoal::RECOGNIZE:
      doRecognition( goal->interactive );
      break;
    case UserCommandGoal::DETECT:
      doDetection( goal->interactive );
      break;
    case UserCommandGoal::RESET:
      resetMarkers();
      break;
    default:
      ROS_ERROR("Unknown user command requested.");
      break;
  }

  if (user_command_server_->isPreemptRequested())
  {
    preemptAction();
  }
}


void InteractiveObjDetBackend::statusFeedback( std::string msg )
{
  UserCommandFeedback feedback;
  feedback.status = msg;
  ROS_INFO_STREAM( feedback.status );
  user_command_server_->publishFeedback( feedback );
}

void InteractiveObjDetBackend::abortAction( std::string msg )
{
  ROS_ERROR_STREAM( "Action aborted: " << msg );
  UserCommandResult result;
  user_command_server_->setAborted( result, msg );
}

void InteractiveObjDetBackend::finishAction( std::string msg )
{
  if ( user_command_server_->isPreemptRequested() )
  {
    ROS_WARN( "Preempt requested - action cannot be finished" );
    return;
  }
  ROS_INFO_STREAM( "Action finished: " << msg );
  UserCommandResult result;
  user_command_server_->setSucceeded( result, msg );
}

void InteractiveObjDetBackend::preemptAction( )
{
  ROS_WARN_STREAM( "Action canceled." );
  UserCommandResult result;
  user_command_server_->setPreempted( result, "Action canceled." );
}

bool InteractiveObjDetBackend::getSensorData( ros::Duration time_out )
{
  ros::Time start_time = ros::Time::now();
  if (!rgbd_assembler_client_.call(rgbd_assembler_srv_))
  {
    abortAction("Call to rgbd assembler service failed");
    return false;
  }
  ROS_INFO_STREAM("Detection backend: assembled data received after " << ros::Time::now() - start_time << " seconds");
  if (rgbd_assembler_srv_.response.result != rgbd_assembler_srv_.response.SUCCESS)
    {
      std::ostringstream s;
      s << "RGB-D Assembler service returned error " << (int)rgbd_assembler_srv_.response.result;
      abortAction( s.str() );
      return false;
    }

  image_ = rgbd_assembler_srv_.response.image;
  disparity_image_ = rgbd_assembler_srv_.response.disparity_image;
  camera_info_ = rgbd_assembler_srv_.response.camera_info;
  point_cloud_ = rgbd_assembler_srv_.response.point_cloud;
  
  return true;
}

bool InteractiveObjDetBackend::doInteractiveSegmentation( )
{
  statusFeedback("Waiting for user input .." );
  
  std::string segm_topic("segmentation_popup");
  interactive_perception_msgs::ObjectSegmentationGuiGoal   segm_goal;
  actionlib::SimpleActionClient<interactive_perception_msgs::ObjectSegmentationGuiAction> 
    os_gui_action_client(segm_topic, true);
  
  while(!os_gui_action_client.waitForServer(ros::Duration(2.0)) && priv_nh_.ok() 
	&& !user_command_server_->isPreemptRequested()) {
    ROS_INFO("Waiting for action client on topic %s", segm_topic.c_str());
  }
  
  if (!priv_nh_.ok()) return false;
  //check for cancel request
  if (user_command_server_->isPreemptRequested()) return false;
  
  segm_goal.disparity_image  = disparity_image_;
  segm_goal.camera_info      = camera_info_;
  segm_goal.point_cloud      = point_cloud_;

  os_gui_action_client.sendGoal(segm_goal);
  ROS_INFO("Send Data as goal");
  while (!os_gui_action_client.waitForResult(ros::Duration(0.5)) && priv_nh_.ok() 
	 && !user_command_server_->isPreemptRequested()) {
    ROS_INFO_STREAM("Waiting for result from action client on topic" << segm_topic);
  }
  
  //make sure we cancel the gui request if we're canceled
  if ( user_command_server_->isPreemptRequested() ) {
    os_gui_action_client.cancelGoal();
    return false;
  }
  
  if (os_gui_action_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    
    abortAction( "The interactive segmentation has not succeeded;");
    return false;
  } 
  
  interactive_perception_msgs::ObjectSegmentationGuiResult segm_result = *(os_gui_action_client.getResult());
  if (segm_result.result != interactive_perception_msgs::ObjectSegmentationGuiResult::SUCCESS) {
    std::ostringstream s;
    s << "Interactive Segmentation returned error " << (int)segm_result.result;
    abortAction( s.str() );
    return false;
  } 
   
  segmentation_result_.result   = segm_result.result;
  segmentation_result_.clusters = segm_result.clusters;
  segmentation_result_.table    = segm_result.table;

  return true;
}


bool InteractiveObjDetBackend::doAutoSegmentation( )
{
  tabletop_object_detector::TabletopSegmentation segmentation_srv;

  if (!auto_seg_client_.call(segmentation_srv))
  {
    abortAction("Call to segmentation service failed");
    return false;
  }

  if (segmentation_srv.response.result != TabletopSegmentationResponse::SUCCESS)
  {
    std::ostringstream s;
    s << "Segmentation service returned error " << (int)segmentation_srv.response.result;
    abortAction( s.str() );
    return false;
  }

  segmentation_result_ = segmentation_srv.response;

  return true;
}


bool InteractiveObjDetBackend::doSegmentation( bool interactive )
{
  if (!getSensorData(ros::Duration(10.0))) return false;

  segmentation_was_interactive_ = interactive;
  recognition_result_.cluster_model_indices.clear();
  recognition_result_.models.clear();

  if ( interactive )
  {
    if ( !doInteractiveSegmentation() ) return false;
  }
  else
  {
    if ( !doAutoSegmentation() ) return false;
  }

  //add the table to the collision map
  addTableToCollisionMap(segmentation_result_.table);

  std::ostringstream s;
  s << "Found " << (int)segmentation_result_.clusters.size() << " clusters.";
  finishAction( s.str() );

  int n;
  publishResult( n );

  return true;
}


bool InteractiveObjDetBackend::doInteractiveRecognition()
{
  //wait for popup server
  actionlib::SimpleActionClient<ObjectRecognitionGuiAction> or_gui_action_client("object_recognition_popup", true);

  statusFeedback("Waiting for object recognition popup service.." );

  while(!or_gui_action_client.waitForServer(ros::Duration(2.0)) && priv_nh_.ok() && !user_command_server_->isPreemptRequested())
  {
    ROS_INFO("Waiting for ObjectRecognitionGuiAction server");
  }

  if (!priv_nh_.ok() || user_command_server_->isPreemptRequested()) return false;

  ObjectRecognitionGuiGoal rec_goal;
  rec_goal.image = image_;
  rec_goal.camera_info = camera_info_;

  //get all the meshes from the database & transforms from tf

  statusFeedback("Getting models and coordinate transforms.." );

  int num_models = recognition_result_.models.size();
  rec_goal.model_hypotheses.resize( num_models );

  for ( int m=0; m<num_models; ++m )
  {
    std::vector<household_objects_database_msgs::DatabaseModelPose> &model_pose_list =
        recognition_result_.models[m].model_list;

    int num_hypotheses = model_pose_list.size();

    rec_goal.model_hypotheses[m].hypotheses.resize( num_hypotheses );

    if ( num_hypotheses > 0 )
    {
      rec_goal.model_hypotheses[m].accept = model_pose_list[0].confidence < min_marker_quality_;
    }
    else
    {
      rec_goal.model_hypotheses[m].accept = false;
    }

    for ( int h=0; h<num_hypotheses; ++h )
    {
      shape_msgs::Mesh mesh;

      if ( !getModelMesh( model_pose_list[h].model_id, rec_goal.model_hypotheses[m].hypotheses[h].mesh ) )
      {
        abortAction( "Failed to get model mesh." );
        return false;
      }

      ROS_INFO_STREAM( "Model " << m << ", hypothesis " << h << " (database id "
          << model_pose_list[h].model_id << ") has " << mesh.triangles.size() << " triangles and "
          << mesh.vertices.size() << " vertices." );

      rec_goal.model_hypotheses[m].hypotheses[h].pose = model_pose_list[h].pose;

      if ( !transformPose( rec_goal.model_hypotheses[m].hypotheses[h].pose, point_cloud_.header.frame_id ) )
      {
        abortAction( "Failed to get transform." );
        return false;
      }

      if ( user_command_server_->isPreemptRequested() )
      {
        return false;
      }
    }
  }

  or_gui_action_client.sendGoal(rec_goal);

  statusFeedback("Waiting for user input.." );

  // wait for gui action server & send request
  while (!or_gui_action_client.waitForResult(ros::Duration(2.0)) && priv_nh_.ok() && !user_command_server_->isPreemptRequested())
  {
    ROS_INFO_STREAM("Waiting for ObjectRecognitionGuiAction result");
  }

  //make sure we cancel the gui request if we're being canceled
  if ( user_command_server_->isPreemptRequested() )
  {
    or_gui_action_client.cancelGoal();
    return false;
  }

  interactive_perception_msgs::ObjectRecognitionGuiResultConstPtr rec_result;
  rec_result = or_gui_action_client.getResult();

  if ( num_models != (int)rec_result->selected_hypothesis_indices.size() )
  {
    abortAction( "GUI returned invalid result (# of models in result don't match request.)" );
    return false;
  }

  // only keep the selected model/pose hypotheses in the recognition result

  for ( int m=0; m<num_models; ++m )
  {
    std::vector<household_objects_database_msgs::DatabaseModelPose> &model_pose_list =
        recognition_result_.models[m].model_list;

    int sel_hyp_index = rec_result->selected_hypothesis_indices[m];

    if ( sel_hyp_index >=0 && sel_hyp_index < (int)model_pose_list.size() )
    {
      household_objects_database_msgs::DatabaseModelPose sel_hyp = model_pose_list[sel_hyp_index];
      sel_hyp.confidence = 0;
      model_pose_list.clear();
      model_pose_list.push_back( sel_hyp );
    }
    else
    {
      model_pose_list.clear();
    }
  }

  if (!priv_nh_.ok()) return false;

  return true;
}


bool InteractiveObjDetBackend::doAutoRecognition( )
{
  statusFeedback("Doing automatic recognition." );

  tabletop_object_detector::TabletopObjectRecognition recognition_srv;

  //call automatic recognition service
  recognition_srv.request.table = segmentation_result_.table;
  recognition_srv.request.clusters = segmentation_result_.clusters;
  recognition_srv.request.num_models = 5;
  recognition_srv.request.perform_fit_merge = !segmentation_was_interactive_;

  if (!auto_obj_rec_client_.call(recognition_srv))
  {
    abortAction("Call to recognition service failed");
    return false;
  }

  //no objects found -> abort
  if (recognition_srv.response.models.size() == 0)
  {
    abortAction( "No objects found." );
    return false;
  }
  else
  {
    std::ostringstream s;
    s << recognition_srv.response.models.size() << " objects recognized.";
    statusFeedback( s.str() );
  }

  recognition_result_ = recognition_srv.response;

  return true;
}

bool InteractiveObjDetBackend::doRecognition( bool interactive )
{
  if ( segmentation_result_.clusters.empty() )
  {
    abortAction("No clusters found or no segmentation done yet.");
    return false;
  }

  // call recognition service
  if ( !doAutoRecognition() ) return false;

  if (user_command_server_->isPreemptRequested()) return false;

  //call gui popup service if in interactive mode
  if ( interactive )
  {
    if ( !doInteractiveRecognition() ) return false;
  }

  if (user_command_server_->isPreemptRequested()) return false;

  // publish result
  ROS_INFO( "Object detection procedure finished. Publishing result." );
  int num_recognized;

  if ( !publishResult( num_recognized ) )
  {
    abortAction( "An error occured while gathering the result data." );
    return false;
  }

  std::ostringstream s;
  s << num_recognized << " of " << segmentation_result_.clusters.size() << " objects recognized.";
  finishAction( s.str() );

  return true;
}


bool InteractiveObjDetBackend::doAutoDetection( )
{
  statusFeedback("Doing automatic detection." );

  tabletop_object_detector::TabletopDetection tabletop_detection_srv;

  //call tabletop detection service
  tabletop_detection_srv.request.return_clusters = false;
  tabletop_detection_srv.request.return_models = true;
  tabletop_detection_srv.request.num_models = 5;

  if (!tabletop_detection_client_.call(tabletop_detection_srv))
  {
    abortAction("Call to tabletop detection service failed");
    return false;
  }

  //no objects found -> abort
  if (tabletop_detection_srv.response.detection.models.size() == 0)
  {
    abortAction( "No objects found." );
    return false;
  }
  else
  {
    std::ostringstream s;
    s << tabletop_detection_srv.response.detection.models.size() << " objects recognized.";
    statusFeedback( s.str() );

    //transform poses to odom_combined frame, so they're no longer head-relative
    for(size_t i=0; i<tabletop_detection_srv.response.detection.models.size(); i++)
    {
      if ( !transformPose( tabletop_detection_srv.response.detection.models[i].model_list[0].pose, "/odom_combined") )
      {
        abortAction( "Failed to get transform to odom_combined." );
        return false;
      }
    }

  }

  ROS_INFO( "Tabletop detection returned %d clusters.", (int)tabletop_detection_srv.response.detection.clusters.size() );

  segmentation_result_.clusters = tabletop_detection_srv.response.detection.clusters;
  recognition_result_.cluster_model_indices = tabletop_detection_srv.response.detection.cluster_model_indices;
  recognition_result_.models = tabletop_detection_srv.response.detection.models;

  //tod hack
//  segmentation_result_.clusters.clear();
//  recognition_result_.cluster_model_indices.clear();

  return true;
}


bool InteractiveObjDetBackend::doDetection( bool interactive )
{
  // call recognition service
  if ( !doAutoDetection() ) return false;

  if (user_command_server_->isPreemptRequested()) return false;

  if (!getSensorData(ros::Duration(10.0))) return false;

  //call gui popup service if in interactive mode
  if ( interactive )
  {
    if ( !doInteractiveRecognition() ) return false;
  }

  if (user_command_server_->isPreemptRequested()) return false;

  // publish result
  ROS_INFO( "Object detection procedure finished. Publishing result." );
  int num_recognized;

  if ( !publishResult( num_recognized ) )
  {
    abortAction( "An error occured while gathering the result data." );
    return false;
  }

  std::ostringstream s;
  s << num_recognized << " of " << segmentation_result_.clusters.size() << " objects recognized.";
  finishAction( s.str() );

  return true;
}

bool InteractiveObjDetBackend::publishResult( int &num_recognized )
{
  if ( segmentation_result_.clusters.size() == 0 && recognition_result_.models.size() == 0 )
  {
    ROS_ERROR( "No results available for publishing!" );
    return false;
  }

  num_recognized = 0;
  GraspableObjectList grasp_obj_list;

  std::string reference_frame_id;

  if ( segmentation_result_.clusters.size() > 0 )
  {
    reference_frame_id = segmentation_result_.clusters[0].header.frame_id;
  }
  else
  {
    if ( recognition_result_.models[0].model_list.size() == 0 )
    {
      ROS_ERROR( "Model list 0 from recognition result is empty!" );
      return false;
    }
    reference_frame_id = recognition_result_.models[0].model_list[0].pose.header.frame_id;
  }

  ROS_INFO( "The reference frame is %s", reference_frame_id.c_str() );

  grasp_obj_list.camera_info = camera_info_;
  grasp_obj_list.image = image_;
  getPose( grasp_obj_list.reference_to_camera, reference_frame_id, point_cloud_.header.frame_id );

  unsigned num_clusters = segmentation_result_.clusters.size();
  unsigned num_models = recognition_result_.models.size();

  std::set<unsigned> used_model_indices;
  std::set<unsigned> used_cluster_indices;

  ROS_INFO( "Number of clusters: %d models: %d", num_clusters, num_models );

  //first, go through all clusters and see if they match any model
  //store the info as graspable object
  for ( unsigned cluster_index=0; cluster_index<num_clusters; ++cluster_index )
  {
    manipulation_msgs::GraspableObject graspable_object;
    shape_msgs::Mesh mesh;

    //populate cluster
    graspable_object.cluster = segmentation_result_.clusters[cluster_index];
    graspable_object.reference_frame_id = reference_frame_id; // should be robot_reference_frame_id_;

    //if we have recognition results, populate model data
    if ( recognition_result_.cluster_model_indices.size() == num_clusters )
    {
      unsigned model_index = recognition_result_.cluster_model_indices[cluster_index];

      //skip models that we have already processed
      //a better way would be to actually merge all clusters that point to the same model,
      //but we'll go with that for now
      if ( used_model_indices.find( model_index ) != used_model_indices.end() )
      {
        continue;
      }

      used_model_indices.insert( model_index );

      if ( model_index > recognition_result_.models.size() )
      {
        ROS_ERROR( "Invalid model index for cluster %d!", cluster_index );
        return false;
      }

      graspable_object.cluster = segmentation_result_.clusters[cluster_index];
      graspable_object.reference_frame_id = robot_reference_frame_id_;

      //only get the mesh if we have recognition results and
      //the first one is good enough
      if ( recognition_result_.models[ model_index ].model_list.size() == 0 ||
          recognition_result_.models[ model_index ].model_list[0].confidence > min_marker_quality_ )
      {
        ROS_INFO( "Model %d was not recognized.", model_index );

        //get pose of cluster relative to the camera frame
        if ( graspable_object.cluster.header.frame_id != reference_frame_id )
        {
          ROS_ERROR( "Point cluster has wrong frame id (%s)", graspable_object.cluster.header.frame_id.c_str() );
          return false;
        }
      }
      else
      {
        graspable_object.potential_models = recognition_result_.models[ model_index ].model_list;

        if ( !getModelMesh( graspable_object.potential_models[0].model_id, mesh ) )
        {
          ROS_ERROR( "Failed to get model mesh." );
          return false;
        }

        ROS_INFO_STREAM( "Model " << model_index << " (database id "
            << graspable_object.potential_models[0].model_id << ") has " << mesh.triangles.size() << " triangles and "
            << mesh.vertices.size() << " vertices." );

        if ( graspable_object.potential_models[0].pose.header.frame_id != reference_frame_id )
        {
          ROS_ERROR( "Model pose has wrong frame id (%s)", 
                     graspable_object.potential_models[0].pose.header.frame_id.c_str() );
          return false;
        }

        num_recognized++;
      }
    }

    grasp_obj_list.meshes.push_back( mesh );
    grasp_obj_list.graspable_objects.push_back( graspable_object );
  }

  //there might be recognized object models without associated cluster (e.g. the stuff that TOD returns)
  //so we need to collect those
  for ( unsigned model_index=0; model_index<num_models; ++model_index )
  {
    //skip all models that we've already stored
    if ( used_model_indices.find( model_index ) != used_model_indices.end() )
    {
      continue;
    }

    //only get the mesh if we have recognition results and
    //the first one is good enough
    if ( recognition_result_.models[ model_index ].model_list.size() == 0 ||
         recognition_result_.models[ model_index ].model_list[0].confidence > min_marker_quality_ )
    {
      ROS_INFO( "Model %d was not recognized.", model_index );
      continue;
    }

    used_model_indices.insert( model_index );

    manipulation_msgs::GraspableObject graspable_object;
    shape_msgs::Mesh mesh;

    graspable_object.reference_frame_id = reference_frame_id;
    graspable_object.potential_models = recognition_result_.models[ model_index ].model_list;

    if ( !getModelMesh( graspable_object.potential_models[0].model_id, mesh ) )
    {
      ROS_ERROR( "Failed to get mesh for model #%d.", model_index );
      return false;
    }

    ROS_INFO_STREAM( "Model " << model_index << " (database id "
        << graspable_object.potential_models[0].model_id << ") has " << mesh.triangles.size() << " triangles and "
        << mesh.vertices.size() << " vertices." );

    if ( graspable_object.potential_models[0].pose.header.frame_id != reference_frame_id )
    {
      ROS_ERROR( "Model pose has wrong frame id (%s)", graspable_object.potential_models[0].pose.header.frame_id.c_str() );
      return false;
    }

    num_recognized++;

    //hack to take care the timestamp doesn't get too old
    graspable_object.potential_models[0].pose.header.stamp = ros::Time();

    grasp_obj_list.meshes.push_back( mesh );
    grasp_obj_list.graspable_objects.push_back( graspable_object );
  }

  result_publisher_.publish( grasp_obj_list );

  printObjects( grasp_obj_list.graspable_objects );

  publishFitMarkers( recognition_result_.models, segmentation_result_.table );

  return true;
}

void InteractiveObjDetBackend::resetMarkers()
{
  //delete all published fit markers
  try
  {
    for ( size_t i=0; i<delete_markers_.size(); i++ )
    {
      delete_markers_[i].header.stamp = ros::Time::now();
      marker_pub_.publish( delete_markers_[i] );
    }
  }
  catch (...)
  {
    ROS_ERROR("exception when trying to clear old fit markers!");
  }
  delete_markers_.clear();

  //publish an empty list of objects so interactive_marker_node clears its object_handlers_ list
  GraspableObjectList grasp_obj_list;
  result_publisher_.publish( grasp_obj_list );

  finishAction( "reset all segmentation and detection markers" );
}

void InteractiveObjDetBackend::publishFitMarkers(
    const std::vector<household_objects_database_msgs::DatabaseModelPoseList> &potential_models,
    const tabletop_object_detector::Table &table )
{
  try
  {
    //first, clear the old markers
    for ( size_t i=0; i<delete_markers_.size(); i++ )
    {
      delete_markers_[i].header.stamp = ros::Time::now();
      marker_pub_.publish( delete_markers_[i] );
    }
  }
  catch (...)
  {
    ROS_ERROR("exception when trying to clear old fit markers!");
  }
  delete_markers_.clear();

  for (size_t i=0; i<potential_models.size(); i++)
  {
    const std::vector<household_objects_database_msgs::DatabaseModelPose> models = potential_models[i].model_list;
    for (size_t j=0; j<models.size(); j++)
    {
      shape_msgs::Mesh mesh;

      if ( !getModelMesh( models[j].model_id, mesh ) )
      {
        ROS_ERROR("Failed to call database get mesh service for marker display");
      }
      else
      {
        double rank = ((double)j) / std::max( (int)(models.size())-1, 1 );
        visualization_msgs::Marker fitMarker =  MarkerGenerator::getFitMarker(mesh, rank);
        fitMarker.header = models[j].pose.header;
        fitMarker.pose = models[j].pose.pose;
        fitMarker.ns = "pr2_interactive_object_detection_model_" + boost::lexical_cast<std::string>(j);
        fitMarker.id = current_marker_id_++;
        marker_pub_.publish(fitMarker);

        //save a marker that will delete this one later..
        visualization_msgs::Marker delete_marker;
        delete_marker.header.frame_id = fitMarker.header.frame_id;
        delete_marker.id = fitMarker.id;
        delete_marker.ns = fitMarker.ns;
        delete_marker.action = visualization_msgs::Marker::DELETE;
        delete_markers_.push_back( delete_marker );
      }
    }
  }
}

bool InteractiveObjDetBackend::getPose( geometry_msgs::Pose &pose, std::string from_frame, std::string to_frame )
{
  //we expect the robot to not move during the whole procedure,
  //so time::now should do it
  ros::Time pose_time = ros::Time::now();

  tf::StampedTransform pose_transform;
  try
  {
    tf::Transformer transformer;
    ROS_INFO_STREAM( "Looking up transform from " << from_frame << " to " << to_frame );
    tf_listener_.waitForTransform(to_frame, from_frame, pose_time, ros::Duration(3.0));
    tf_listener_.lookupTransform(to_frame, from_frame, pose_time, pose_transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }

  //convert back
  tf::poseTFToMsg ( pose_transform , pose );
  ROS_INFO( "Position: %f, %f, %f / orientation: %f %f %f %f ",
      pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z );

  return true;
}



bool InteractiveObjDetBackend::transformPose( geometry_msgs::PoseStamped &pose, std::string target_frame )
{
  // convert object pose to tf format
  tf::Transform old_pose;
  tf::poseMsgToTF ( pose.pose, old_pose );

  //get transformation from object's parent frame to camera frame
  std::string old_frame = pose.header.frame_id;

  //we expect the robot to not move during the whole procedure,
  //so time::now should do it
  ros::Time pose_time = ros::Time::now();

  tf::StampedTransform old_to_target;
  try
  {
    tf::Transformer transformer;
    ROS_INFO_STREAM( "Looking up transform from " << old_frame << " to " << target_frame );
    tf_listener_.waitForTransform(target_frame, old_frame, pose_time, ros::Duration(3.0));
    tf_listener_.lookupTransform(target_frame, old_frame, pose_time, old_to_target);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }

  //concatenate transformations
  tf::Transform target_pose = old_to_target * old_pose;

  ROS_INFO( "Old position %f, %f, %f / orientation %f %f %f %f ",
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z );

  //convert back
  tf::poseTFToMsg ( target_pose , pose.pose );
  pose.header.frame_id = target_frame;

  ROS_INFO( "New position %f, %f, %f / orientation %f %f %f %f ",
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
      pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z );

  return true;
}


bool InteractiveObjDetBackend::getModelMesh( int model_id, shape_msgs::Mesh& mesh )
{
  GetModelMesh mesh_srv;

  mesh_srv.request.model_id = model_id;
  if ( !get_model_mesh_client_.call(mesh_srv) )
  {
    ROS_ERROR("Failed to call get model mesh service");
    return false;
  }

  if (mesh_srv.response.return_code.code != DatabaseReturnCode::SUCCESS)
  {
    ROS_ERROR("Model mesh service reports an error (code %d)", mesh_srv.response.return_code.code);
    return false;
  }

  mesh = mesh_srv.response.mesh;
  return true;
}


bool InteractiveObjDetBackend::getModelInfo(const DatabaseModelPose &model_pose,
        std::string &name, std::string &all_tags)
{
  GetModelDescription desc;
  desc.request.model_id = model_pose.model_id;
  if ( !get_model_description_client_.call(desc) )
  {
    ROS_ERROR("Failed to call get model description service");
    return false;
  }

  if (desc.response.return_code.code != desc.response.return_code.SUCCESS )
  {
    ROS_ERROR("Model description service reports an error (code %d)", desc.response.return_code.code);
    return false;
  }
  name = desc.response.name;
  for (size_t i=0; i<desc.response.tags.size(); i++)
  {
    if (!all_tags.empty()) all_tags.append(",");
    all_tags.append(desc.response.tags.at(i));
  }
  return true;
}


int InteractiveObjDetBackend::printObjects(const std::vector<manipulation_msgs::GraspableObject> &objects)
{
  ROS_INFO_STREAM( "Detected " << objects.size() << " graspable object(s):\n" );
  for (size_t m=0; m<objects.size(); m++)
  {
    std::string name, all_tags;
    if (!objects[m].potential_models.empty())
    {
      if (getModelInfo(objects[m].potential_models[0], name, all_tags))
      {
        ROS_INFO("  (%d): %s (tags: %s) in frame %s", (int)m, name.c_str(), all_tags.c_str(),
                 objects[m].potential_models[0].pose.header.frame_id.c_str());
      }
      else
      {
        ROS_INFO("  (%d): database object, details not available.", (int)m );
      }
    }
    else
    {
      ROS_INFO("  (%d): unrecognized cluster with %d points", (int)m,
               (unsigned int)objects[m].cluster.points.size());
    }
  }
  return 0;
}


bool InteractiveObjDetBackend::addTableToCollisionMap(tabletop_object_detector::Table table)
{
  // make sure collision services are available
  ROS_INFO("interactive object detection: waiting for collision map services");
  ros::Time start_time = ros::Time::now();
  while (!collision_map_interface_.connectionsEstablished(ros::Duration(1.0))) 
  {
    if (ros::Time::now() - start_time >= ros::Duration(5.0))
    {
      ROS_ERROR("collision map services not found");
      return false;
    }
  }
  collision_map_interface_.processCollisionGeometryForTable(table, "table");
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_interactive_object_detection_backend");
  InteractiveObjDetBackend node;
  ros::spin();
  return 0;
}
