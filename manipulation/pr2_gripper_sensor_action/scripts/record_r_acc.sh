# record $2 samples to file defined by $1
# example: record_r_acc my_filename 30000
rostopic echo -p /r_gripper_sensor_controller/event_detector_state | head -n $2 > "$1.rtp"