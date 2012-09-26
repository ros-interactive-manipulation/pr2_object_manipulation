# time the data was recorded at
time stamp

# the amount of deformation from action start (in meters)
float64 deformation

# the force experinced by the finger Pads  (N)
# NOTE:this ignores data from the edges of the finger pressure
float64 left_fingertip_pad_force
float64 right_fingertip_pad_force

# the current virtual parallel joint effort of the gripper (in N)
float64 joint_effort

# true if the object recently slipped
bool slip_detected

# true if we are at or exceeding the deformation limit
# (see wiki page and param server for more info)
bool deformation_limit_reached

# true if we are at or exceeding our force 
# (see wiki page and param server for more info)
bool fingertip_force_limit_reached

# true if the controller thinks the gripper is empty
# (see wiki page for more info)
bool gripper_empty

# the control state of our realtime controller
PR2GripperSensorRTState rtstate