# NOTE: This message is only for debugging purposes. It is not intended for API usage - and is not published under release code.

# Standard ROS header
time stamp

# corrected for zero contact
float64 left_finger_pad_force
float64 right_finger_pad_force

# filtered values : high pass filter at 5 Hz after correcting for zero contact
float64 left_finger_pad_force_filtered
float64 right_finger_pad_force_filtered

# corrected for zero contact
float64[22] left_finger_pad_forces
float64[22] right_finger_pad_forces

# filtered values : high pass filter at 5 Hz after correcting for zero contact
float64[22] left_finger_pad_forces_filtered
float64[22] right_finger_pad_forces_filtered

# raw acceleration values
float64 acc_x_raw
float64 acc_y_raw
float64 acc_z_raw

# filtered acceleration values
float64 acc_x_filtered
float64 acc_y_filtered
float64 acc_z_filtered

# boolean variables indicating whether contact exists or not
bool left_contact
bool right_contact