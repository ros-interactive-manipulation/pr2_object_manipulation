
# collision checking enabled
bool collision_checked

# 0=call gripper click
# 1=grasp the provided graspable object
int32 grasp_selection

# 0=right, 1=left arm
int32 arm_selection

# for RESET commands
# 0=reset collision objects
# 1=reset attached objects
int32 reset_choice

# for MOVE_ARM commands
# 0=side
# 1=front
# 2=side handoff
int32 arm_action_choice

# for MOVE_ARM commands
# 0=open-loop
# 1=with planner
int32 arm_planner_choice

# for MOVE_GRIPPER commands
# opening of gripper (0=closed..100=open)
int32 gripper_slider_position

# used if grasp_selection == 1
object_manipulation_msgs/GraspableObject selected_object

# indicates obstacles that can be moved during grasping
# presumably, the operator has marked these in some fashion
object_manipulation_msgs/GraspableObject[] movable_obstacles

# more options..
IMGUIAdvancedOptions adv_options
