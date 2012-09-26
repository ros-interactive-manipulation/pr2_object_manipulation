# the control state of our realtime controller
int8 realtime_controller_state

# predefined values to indicate our realtime_controller_state
int8 DISABLED = 0
int8 POSITION_SERVO = 3
int8 FORCE_SERVO = 4
int8 FIND_CONTACT = 5
int8 SLIP_SERVO = 6