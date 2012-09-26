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

#include <tabletop_object_detector/TabletopDetection.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#include <household_objects_database_msgs/DatabaseModelPoseList.h>
#include <household_objects_database_msgs/TranslateRecognitionId.h>
#include <object_manipulator/tools/service_action_wrappers.h>

using namespace tabletop_object_detector;
using namespace object_recognition_msgs;
using namespace household_objects_database_msgs;
using namespace object_manipulator;

namespace pr2_interactive_object_detection {

class RecognitionTranslator
{
private:
  ros::NodeHandle root_nh_;
  ros::ServiceServer server_;
  ServiceWrapper<TranslateRecognitionId> translation_client_;
  ActionWrapper<ObjectRecognitionAction> action_client_;

  bool getIdFromRecognitionId(std::string recognition_id, int &db_id)
  {
    try
    {
      TranslateRecognitionId srv;
      srv.request.recognition_id = recognition_id;
      if (!translation_client_.client().call(srv))
      {
        ROS_ERROR("Call to translation service failed");
        return false;
      }
      if (srv.response.result == srv.response.SUCCESS)
      {
        db_id = srv.response.household_objects_id;
        return true;
      }
      else if(srv.response.result == srv.response.ID_NOT_FOUND)
      {
        ROS_ERROR("Recognition id %s not found", recognition_id.c_str());
      }
      else
      {
        ROS_ERROR("Translate is service returned an error");
      }
      return false;
    }
    catch (ServiceNotFoundException &ex)
    {
      ROS_ERROR("Translation service not found");
      return false;
    }
  }

  bool serviceCB(TabletopDetection::Request &request, TabletopDetection::Response &response)
  {
    response.detection.result = response.detection.OTHER_ERROR;
    try
    {
      if (request.return_clusters || !request.return_models || request.num_models < 1)
      {
        ROS_INFO("TOD recognition can not return clusters; set return_clusters to false and return_models to true");
        return true;
      }
      //RecognizeObjectsGoal goal;
      ObjectRecognitionGoal goal;
      ROS_INFO("Calling recognition action");
      action_client_.client().sendGoal(goal);
      while (!action_client_.client().waitForResult(ros::Duration(2.0)) && root_nh_.ok())
      {
        ROS_INFO("Waiting for recognition action...");
      }
      if (!root_nh_.ok()) return true;
      if (action_client_.client().getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_ERROR("Object recognition action failed");
        return true;
      }
      ObjectRecognitionResult result = *(action_client_.client().getResult());

      response.detection.result = response.detection.SUCCESS;
      ROS_INFO("Recognition action returned %zd results", result.recognized_objects.objects.size());
      for (size_t i=0; i<result.recognized_objects.objects.size(); i++)
      {
        DatabaseModelPose model_pose;
        if (!getIdFromRecognitionId(result.recognized_objects.objects[i].id.id, model_pose.model_id))
        {
          ROS_ERROR("Failed to get household ID for recognition ID %s", result.recognized_objects.objects[i].id.id.c_str());
          continue;
        }
        model_pose.pose.header = result.recognized_objects.objects[i].pose.header;
        model_pose.pose.pose = result.recognized_objects.objects[i].pose.pose.pose;
        //model_pose.confidence = result.confidence[i];
        //The world's most horrible hack
        //the rest of system still uses thresholds from the old tabletop object recognition
        model_pose.confidence = 0.002;
        DatabaseModelPoseList model_pose_list;
        model_pose_list.model_list.push_back(model_pose);
        response.detection.models.push_back(model_pose_list);
      }
      
    }
    catch (ServiceNotFoundException &ex)
    {
      ROS_ERROR("Recognition action server not found");
    }
    return true;
  }

public:
  RecognitionTranslator() : 
    root_nh_(""),
    translation_client_("objects_database_node/translate_id"),
    action_client_("/object_recognition/recognize_objects", true)
  {
    server_ = root_nh_.advertiseService("object_recognition_translated", &RecognitionTranslator::serviceCB, this);    
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recognition_translator");
  pr2_interactive_object_detection::RecognitionTranslator rt;
  ros::spin();
  return 0;  
}
