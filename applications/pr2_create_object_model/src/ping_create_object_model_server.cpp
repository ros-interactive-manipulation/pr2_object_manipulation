#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "pr2_create_object_model/ModelObjectInHandAction.h"


// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const pr2_create_object_model::ModelObjectInHandResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Number of points in resulting cluster: %d", result->cluster.width);
  ROS_INFO("collision_name: %s", result->collision_name.c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const pr2_create_object_model::ModelObjectInHandFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback, phase %d, rotate_ind %d", feedback->phase, feedback->rotate_ind);
}


int main (int argc, char **argv)
{
  ros::init(argc, argv, "ping_create_object_model_server");

  // create the action client
  // true causes the client to spin it's own thread
  actionlib::SimpleActionClient<pr2_create_object_model::ModelObjectInHandAction> ac("create_object_model_server/model_object_in_hand_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  pr2_create_object_model::ModelObjectInHandGoal goal;
  goal.arm_name = "right_arm";
  goal.clear_move.header.frame_id = "base_link";
  goal.clear_move.vector.z = .5;
  goal.rotate_object = true;
  goal.add_to_collision_map = true;
  goal.keep_level = false;

  //gripper points (x-axis) towards base_link y, gripper y-axis towards base_link -x, z-axis towards base-link +z 
  goal.rotate_pose.header.frame_id = "torso_lift_link";
  goal.rotate_pose.pose.position.x = .7;
  goal.rotate_pose.pose.position.z = .4;
  goal.rotate_pose.pose.orientation.z = 0.707;
  goal.rotate_pose.pose.orientation.w = 0.707;

  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ros::spin();

  //exit
  return 0;
}
