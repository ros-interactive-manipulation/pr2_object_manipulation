# Time the data was recorded at
time stamp

# true when our contact conditions have been met
# (see PR2GripperFindContact command)
bool contact_conditions_met

# the finger contact conditions 
# true if the finger experienced a contact event
#
# contact events are defined as contact with the fingerpads
# as either steady-state or high-freq force events
bool left_fingertip_pad_contact
bool right_fingertip_pad_contact

# the force experinced by the finger Pads  (N)
# NOTE:this ignores data from the edges of the finger pressure
float64 left_fingertip_pad_force
float64 right_fingertip_pad_force

# the current joint position (m)
float64 joint_position

# the virtual (parallel) joint effort (N)
float64 joint_effort

# the control state of our realtime controller
PR2GripperSensorRTState rtstate