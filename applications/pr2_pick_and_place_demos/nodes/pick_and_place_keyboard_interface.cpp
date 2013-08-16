/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>

//for getline
#include <iostream>

//for termios
#include <termios.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>

#include <actionlib/client/simple_action_client.h>

#include <household_objects_database_msgs/GetModelDescription.h>

#include <object_manipulator/tools/mechanism_interface.h>

#include <tabletop_object_detector/TabletopDetection.h>

#include <tabletop_collision_map_processing/collision_map_interface.h>
#include <tabletop_collision_map_processing/TabletopCollisionMapProcessing.h>

#include <object_manipulation_msgs/PickupAction.h>
#include <object_manipulation_msgs/PlaceAction.h>

static const std::string OBJECT_DETECTION_SERVICE_NAME = "/object_detection";
static const std::string COLLISION_PROCESSING_SERVICE_NAME = 
  "/tabletop_collision_map_processing/tabletop_collision_map_processing";

static const std::string PICKUP_ACTION_NAME = "/object_manipulator/object_manipulator_pickup";
static const std::string PLACE_ACTION_NAME = "/object_manipulator/object_manipulator_place";
static const std::string GET_MODEL_DESCRIPTION_NAME = "/objects_database_node/get_model_description";

//for termios
int kfd = 0;
struct termios cooked, raw;

class PickAndPlaceKeyboardInterface
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::ServiceClient object_detection_srv_;
  ros::ServiceClient collision_processing_srv_;
  ros::ServiceClient get_model_description_srv_;

  actionlib::SimpleActionClient<object_manipulation_msgs::PickupAction> pickup_client_;
  actionlib::SimpleActionClient<object_manipulation_msgs::PlaceAction> place_client_;

  object_manipulator::MechanismInterface mech_interface_;
  tabletop_collision_map_processing::CollisionMapInterface collision_map_interface_;

  struct GraspInfo
  {
    geometry_msgs::PoseStamped pickup_location_;
    object_manipulation_msgs::Grasp grasp_;
    std::string collision_object_name_;
  };
  GraspInfo grasp_info_right_;
  GraspInfo grasp_info_left_;

  tabletop_collision_map_processing::TabletopCollisionMapProcessing::Response objects_info_;

public:
  PickAndPlaceKeyboardInterface(ros::NodeHandle &nh) : 
    nh_(nh),
    priv_nh_("~"),
    pickup_client_(PICKUP_ACTION_NAME, true),
    place_client_(PLACE_ACTION_NAME, true),
    mech_interface_()
  {
   //wait forever for connections
    collision_map_interface_.connectionsEstablished(ros::Duration(-1.0));
 
    while ( !ros::service::waitForService(OBJECT_DETECTION_SERVICE_NAME, ros::Duration(2.0)) && nh_.ok() ) 
    {
      ROS_INFO("Waiting for object detection service to come up");
    }
    if (!nh_.ok()) exit(0);
    object_detection_srv_ = nh_.serviceClient<tabletop_object_detector::TabletopDetection>
      (OBJECT_DETECTION_SERVICE_NAME, true);

    while ( !ros::service::waitForService(COLLISION_PROCESSING_SERVICE_NAME, ros::Duration(2.0)) && nh_.ok() ) 
    {
      ROS_INFO("Waiting for collision processing service to come up");
    }
    if (!nh_.ok()) exit(0);
    collision_processing_srv_ = nh_.serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
      (COLLISION_PROCESSING_SERVICE_NAME, true);

    while(!pickup_client_.waitForServer(ros::Duration(2.0)) && priv_nh_.ok())
    {
      ROS_INFO_STREAM("Waiting for action client " << PICKUP_ACTION_NAME);
    }
    if (!priv_nh_.ok()) exit(0);    

    while(!place_client_.waitForServer(ros::Duration(2.0)) && priv_nh_.ok())
    {
      ROS_INFO_STREAM("Waiting for action client " << PLACE_ACTION_NAME);
    }
    if (!priv_nh_.ok()) exit(0);    

    bool use_database;
    priv_nh_.param<bool>("use_database", use_database, false);

    if (use_database)
    {
      while ( !ros::service::waitForService(GET_MODEL_DESCRIPTION_NAME, ros::Duration(2.0)) && nh_.ok() ) 
      {
	ROS_INFO("Waiting for %s service to come up", GET_MODEL_DESCRIPTION_NAME.c_str());
      }
      if (!nh_.ok()) exit(0);
      get_model_description_srv_ = nh_.serviceClient<household_objects_database_msgs::GetModelDescription>
	(GET_MODEL_DESCRIPTION_NAME, true);
    }
  }

  ~PickAndPlaceKeyboardInterface() {}

  bool getModelInfo(const household_objects_database_msgs::DatabaseModelPose &model_pose, 
		    std::string &name, std::string &all_tags)
  {
    household_objects_database_msgs::GetModelDescription desc;
    desc.request.model_id = model_pose.model_id;
    if ( !get_model_description_srv_.call(desc) )
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

  int printObjects(const std::vector<object_manipulation_msgs::GraspableObject> &objects,
		   const std::vector<std::string> &collision_names)
  {
    for (size_t m=0; m<objects.size(); m++)
    { 
      std::string name, all_tags;
      std::string collision_name = collision_names.at(m);
      if (!objects[m].potential_models.empty())
      {
        if (getModelInfo(objects[m].potential_models[0], name, all_tags))
        {
          ROS_INFO("  (%d): %s (%s) in frame %s (%s)", (int)m, name.c_str(), all_tags.c_str(), 
                   objects[m].potential_models[0].pose.header.frame_id.c_str(), collision_name.c_str());
        }
        else
        {
          ROS_INFO("  (%d): database object, details not available (%s)", (int)m,
                   collision_name.c_str());
        }	  
      }
      else
      {
        ROS_INFO("  (%d): unrecognized cluster with %d points (%s)", (int)m, 
                 (unsigned int)objects[m].cluster.points.size(), collision_name.c_str());
      }
    }
    return 0;
  }

  int chooseObject(const std::vector<object_manipulation_msgs::GraspableObject> &objects,
		   const std::vector<std::string> &collision_names)
  {
    if (objects.empty()) 
    {
      ROS_INFO("Detection result contains no graspable objects");
      return -1;
    }
    //print object names and tags
    ROS_INFO("Graspable objects:");
    if (printObjects(objects, collision_names)) return -1;

    //ask which one should be picked up
    char c;
    ROS_INFO("Enter number of object for selection:");
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    int sel;
    sel = atoi(&c);
    std::cout << "Selected " << sel << std::endl;
    if (sel < 0 || sel >= (int)objects.size())
    {
      ROS_INFO("No object selected");
      return -1;
    }    
    return sel;
  }

  geometry_msgs::Vector3Stamped chooseDirection(std::string arm_name, bool grasp)
  {
    char c;
    while (1)
    {
      std::cout << "  press 'u' for lift or place along up/down direction\n";
      std::cout << "  press 'g' for lift or place along gripper direction\n";
      if(read(kfd, &c, 1) < 0)
      {
	perror("read():");
	exit(-1);
      }
      std::cout << "\n";
      if ( c=='u' || c=='g') break;
      else std::cout << "  invalid selection\n";
    }
    geometry_msgs::Vector3Stamped direction;
    direction.header.stamp = ros::Time::now();
    if ( c=='u' )
    {
      direction.header.frame_id = "base_link";
      direction.vector.x = 0;
      direction.vector.y = 0;
      direction.vector.z = 1;
      if (!grasp) direction.vector.z = -1;
    }
    else
    {
      if (arm_name == "right_arm") direction.header.frame_id = "r_wrist_roll_link";
      else direction.header.frame_id = "l_wrist_roll_link";
      direction.vector.x = -1;
      direction.vector.y = 0;
      direction.vector.z = 0;
      if (!grasp) direction.vector.x = 1;
    }
    return direction;
  }

  std::string chooseArm()
  {
    bool done = false;
    char c;
    while ( !done )
    {
      std::cout << "  press 'r' for right arm\n";
      std::cout << "  press 'l' for left arm\n";
      if(read(kfd, &c, 1) < 0)
      {
	perror("read():");
	exit(-1);
      }
      std::cout << "\n";
      if ( c=='r' || c=='l') done = true;
      else std::cout << "  invalid selection\n";
    }
    if (c=='l') return "left_arm";
    return "right_arm";
  }

  void detectAndProcessCollisionMap()
  {
    ROS_INFO("Calling detection service");
    tabletop_object_detector::TabletopDetection detection_srv;
    detection_srv.request.return_clusters = true;
    detection_srv.request.return_models = true;
    detection_srv.request.num_models = 5;
    if (!object_detection_srv_.call(detection_srv))
    {
      ROS_ERROR("Tabletop detection service failed");
      return;
    }
    ROS_INFO("Detection complete; calling collision map processing");
    tabletop_collision_map_processing::TabletopCollisionMapProcessing processing_srv;
    processing_srv.request.detection_result = detection_srv.response.detection;
    processing_srv.request.reset_collision_models = true;
    processing_srv.request.reset_attached_models = true;
    processing_srv.request.desired_frame = "base_link";
    if (!collision_processing_srv_.call(processing_srv))
    {
      ROS_ERROR("Collision map processing service failed");
      return;
    }
    objects_info_ = processing_srv.response;    
    std::cout << "Detected " << objects_info_.graspable_objects.size() << " graspable object(s):\n";
    printObjects(objects_info_.graspable_objects, objects_info_.collision_object_names);
  }

  void printOptions() 
  {
    std::cout << "\n--Object manipulation options:\n";
    std::cout << "Use 'd' re-detect and process collision map\n";
    std::cout << "Use 'p' to select an object and attempt to grasp it\n";
    std::cout << "Use 'w' to put the grasped object back down from where we took it\n";
    std::cout << "Use 'a' to transfer a grasped object to the other gripper\n";
    std::cout << "--Debug options:\n";
    std::cout << "Use 'o' to open gripper\n";
    std::cout << "Use 'c' to close gripper\n";
    std::cout << "Use 'r' to reset collision map\n";
    std::cout << "Use 'e' to detach all objects from gripper\n";
    std::cout << "Use 'm' to move arm to side\n";
    std::cout << "Use 'z' for Cartesian movement\n";
    std::cout << "--Misc options:\n";
    std::cout << "Use 'h' to get this menu\n";
    std::cout << "Use 'q' to quit\n";
  }

  void printMovementOptions()
  {
    std::cout << "Use 'u' and 'd' to move up or down 5 cm\n";
    std::cout << "Use 'l' and 'r' to move left or right 5 cm\n";
    std::cout << "Use 'f' and 'b' to move forward or back 5 cm\n";
    std::cout << "Use 'q' to quit back to the main menu\n";
  }

  void translateGripperCartesian(std::string arm_name, int axis, double dist)
  {
    geometry_msgs::Vector3Stamped direction;
    direction.header.frame_id = "base_link";
    direction.header.stamp = ros::Time::now();
    if(axis == 0) direction.vector.x = dist;
    else if(axis == 1) direction.vector.y = dist;
    else if(axis == 2) direction.vector.z = dist;
    else return;
    std::cout << "desired direction:" << direction;
    mech_interface_.translateGripperCartesian(arm_name, direction, ros::Duration(2.0), .015, .09, .02, .16, .1);
  }

  void execute()
  {
    char c;
    while(nh_.ok()) 
    {
      printOptions();
      std::cout << ">> \n";
      if(read(kfd, &c, 1) < 0)
      {
	perror("read():");
	exit(-1);
      }
      std::cout << "\n";
      switch(c){
      case 'd':
	detectAndProcessCollisionMap();
	break;
      case 'p':
      {
        if(objects_info_.graspable_objects.empty()) 
	{
          std::cout << "No graspable objects available.  Detect again.\n";
	  break;
        }
	int pickup = chooseObject(objects_info_.graspable_objects, objects_info_.collision_object_names);
	if (pickup < 0 || pickup >= (int)objects_info_.graspable_objects.size()) 
	{
	  std::cout << "Not an allowable value for object selection\n";
	  break;
	}
	object_manipulation_msgs::PickupGoal goal;
	goal.target = objects_info_.graspable_objects.at(pickup); 
	goal.arm_name = chooseArm();
	goal.collision_object_name = objects_info_.collision_object_names.at(pickup);
	goal.collision_support_surface_name = objects_info_.collision_support_surface_name;

	goal.lift.direction = chooseDirection(goal.arm_name, true);
	goal.lift.desired_distance = 0.1;
	goal.lift.min_distance = 0.05;

	goal.use_reactive_lift = true;
	goal.use_reactive_execution = true;
	if (!objects_info_.graspable_objects.at(pickup).potential_models.empty())
	{
	  goal.use_reactive_lift = false;
	  goal.use_reactive_execution = false;
	}
	pickup_client_.sendGoal(goal);
	while (!pickup_client_.waitForResult(ros::Duration(10.0)))
	{
	  ROS_INFO("Waiting for the pickup action...");
	}

	object_manipulation_msgs::PickupResult result = *(pickup_client_.getResult());
	if (pickup_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	  std::cout << "The pickup action has failed with result code " << result.manipulation_result.value << "\n";
	  break;
	}

	std::cout << "Pickup reports success\n";
	geometry_msgs::PoseStamped pickup_location;
        //for now, the cluster's reference frame is the frame of the object
        pickup_location.header = objects_info_.graspable_objects.at(pickup).cluster.header;
        pickup_location.pose.position.x = pickup_location.pose.position.y = pickup_location.pose.position.z = 0;
        pickup_location.pose.orientation.x = pickup_location.pose.orientation.y = 
          pickup_location.pose.orientation.z = 0;
        pickup_location.pose.orientation.w = 1;
        
	GraspInfo grasp_info;
	grasp_info.pickup_location_ = pickup_location;
	grasp_info.grasp_ = result.grasp;
	grasp_info.collision_object_name_ = goal.collision_object_name;

	if (goal.arm_name == "right_arm") grasp_info_right_ = grasp_info;
	else grasp_info_left_ = grasp_info;
      }
      break;
      case 'w':
      {
	object_manipulation_msgs::PlaceGoal goal;
	goal.arm_name = chooseArm();
	goal.place_padding = 0.02;
	goal.collision_support_surface_name = objects_info_.collision_support_surface_name;
	goal.use_reactive_place = true;

	GraspInfo grasp_info = grasp_info_left_;
	if (goal.arm_name ==  "right_arm") grasp_info = grasp_info_right_;
	goal.grasp = grasp_info.grasp_;
	goal.place_locations.push_back(grasp_info.pickup_location_);
	goal.collision_object_name = grasp_info.collision_object_name_;

	goal.desired_retreat_distance = 0.1;
	goal.min_retreat_distance = 0.05;
	goal.approach.direction = chooseDirection(goal.arm_name, false);
	goal.approach.desired_distance = 0.1;
	goal.approach.min_distance = 0.05;

	place_client_.sendGoal(goal);
	while (!place_client_.waitForResult(ros::Duration(10.0)))
	{
	  ROS_INFO("Waiting for the place action...");
	}
	object_manipulation_msgs::PlaceResult result = *(place_client_.getResult());
	if (place_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
	  std::cout << "Place failed with error code " << result.manipulation_result.value << "\n";
	  break;
	}
	std::cout << "Object placed successfully\n";
      }
      break;
      case 'o':
      {
	std::string arm_name = chooseArm();
	object_manipulation_msgs::Grasp grasp;
	try
	{
	  mech_interface_.handPostureGraspAction(arm_name, grasp, 
						 object_manipulation_msgs::GraspHandPostureExecutionGoal::RELEASE, 100);
	}
	catch (object_manipulator::GraspException &ex) 
	{
	  std::cout << ex.what() << std::endl;
	}
      }
      break;
      case 'c':
      {
	std::string arm_name = chooseArm();
	object_manipulation_msgs::Grasp grasp;
	grasp.grasp_posture.position.push_back(0.0);
	grasp.grasp_posture.effort.push_back(10000);
	try
	{
	  mech_interface_.handPostureGraspAction(arm_name, grasp, 
                         object_manipulation_msgs::GraspHandPostureExecutionGoal::GRASP, 50);

	}
	catch (object_manipulator::GraspException &ex) 
	{
	  std::cout << ex.what() << std::endl;
	}
      }
      break;
      case 'r':
	try 
	{
	  collision_map_interface_.resetCollisionModels();
	  collision_map_interface_.resetAttachedModels();
	} 
	catch (tabletop_collision_map_processing::CollisionMapException &ex) 
	{
	  std::cout << ex.what() << std::endl;
	}
	break;
      case 'e':
	try
	{
	  collision_map_interface_.resetAttachedModels();
	}
	catch (object_manipulator::GraspException &ex) 
	{
	  std::cout << ex.what() << std::endl;
	}
	break;
      case 'm':
      {
	std::string arm_name = chooseArm();
	std::vector<double> side_position(7, 0.0);
	if (arm_name=="right_arm") 
	{
	  side_position[0] = -2.135;
	  side_position[1] = 0.803;
	  side_position[2] = -1.732;
	  side_position[3] = -1.905;
	  side_position[4] = -2.369;
	  side_position[5] = -1.680;
	  side_position[6] = 1.398;
	} 
	else 
	{
	  side_position[0] = 2.135;
	  side_position[1] = 0.803;
	  side_position[2] = 1.732;
	  side_position[3] = -1.905;
	  side_position[4] = 2.369;
	  side_position[5] = -1.680;
	  side_position[6] = 1.398;
	}    	  
	try
	{
          arm_navigation_msgs::OrderedCollisionOperations empty_col;
          std::vector<arm_navigation_msgs::LinkPadding> empty_pad;
	  if ( !mech_interface_.attemptMoveArmToGoal(arm_name, side_position, empty_col, empty_pad) )  
	    std::cout << "Failed to move arm to side\n";
	  else
	    std::cout << "Moved arm to side\n";
	}
	catch (object_manipulator::MoveArmStuckException &ex)
	{
	  std::cout << "Arm is stuck!\n";
	}
      }
      break;
      case 'h':
        printOptions();
        break;
      case 'q':
        return;
      case 'z':
      {
	std::string arm_name = chooseArm();
	bool done = 0;
	while(nh_.ok() && !done)
	{
	  char d;
	  printMovementOptions();
	  if(read(kfd, &d, 1) < 0)
	  {
	    perror("read():");
	    exit(-1);
	  }
	  std::cout << "\n";
	  switch(d){
	  case 'u':
	    translateGripperCartesian(arm_name, 2, .05);
	    break;
	  case 'd':
	    translateGripperCartesian(arm_name, 2, -.05);
	    break;
	  case 'r':
	    translateGripperCartesian(arm_name, 1, -.05);
	    break;
	  case 'l':
	    translateGripperCartesian(arm_name, 1, .05);
	    break;
	  case 'f':
	    translateGripperCartesian(arm_name, 0, .05);
	    break;
	  case 'b':
	    translateGripperCartesian(arm_name, 0, -.05);
	    break;
	  case 'q':
	    done = 1;
	    break;
	  }
	}
	break;
      }
      default:
        std::cout << "not a recognized option\n";
        printOptions();
        break;	
      }
    }
  }
};

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) 
{
  signal(SIGINT,quit);

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  ros::init(argc, argv, "pick_and_place_keyboard_interface_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  PickAndPlaceKeyboardInterface node(nh);
  node.execute();
 
  tcsetattr(kfd, TCSANOW, &cooked);
  return 0;
}

