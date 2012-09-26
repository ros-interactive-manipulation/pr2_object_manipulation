# set true if you want to calibrate the fingertip sensors on the start
# of the find_contact action. While this is not necessary (and
# the default value will not calibrate the sensors) for best 
# performance it is recommended that you set this to true each time 
# you are calling find_contact and are confident the fingertips are 
# not touching anything
# NOTE: SHOULD ONLY BE TRUE WHEN BOTH FINGERS ARE TOUCHING NOTHING
bool zero_fingertip_sensors

# the finger contact conditions that determine what our goal is
# Leaving this field blank will result in the robot closing until
# contact on BOTH fingers is achieved
int8 contact_conditions

# predefined values for the above contact_conditions variable
int8 BOTH = 0   # both fingers must make contact
int8 LEFT = 1   # just the left finger 
int8 RIGHT = 2  # just the right finger
int8 EITHER = 3 # either finger, we don't care which
