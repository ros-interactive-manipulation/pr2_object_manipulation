# Time the data was recorded at
time stamp

# the force experienced by the finger Pads  (N)
# NOTE:this ignores data from the edges of the finger pressure
float64 left_fingertip_pad_force
float64 right_fingertip_pad_force

# the current gripper virtual parallel joint effort (in N)
float64 joint_effort

# true when the gripper is no longer moving
# and we have reached the desired force level
bool force_achieved


# the control state of our realtime controller
PR2GripperSensorRTState rtstate
