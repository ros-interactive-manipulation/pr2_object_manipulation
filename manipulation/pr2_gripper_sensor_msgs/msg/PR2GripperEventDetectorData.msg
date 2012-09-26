# Time the data was recorded at
time stamp

# true if the trigger conditions have been met 
# (see PR2GripperEventDetectorCommand)
bool trigger_conditions_met

# true if the pressure sensors detected a slip event
# slip events occur when the finger pressure sensors
# high-freq. content exceeds the slip_trigger_magnitude variable
# (see PR2GripperEventDetectorCommand)
bool slip_event

# true if the hand-mounted accelerometer detected a contact acceleration
# acceleration events occur when the palm accelerometer
# high-freq. content exceeds the acc_trigger_magnitude variable
# (see PR2GripperEventDetectorCommand)
bool acceleration_event

# the high-freq acceleration vector that was last seen (x,y,z)
float64[3] acceleration_vector