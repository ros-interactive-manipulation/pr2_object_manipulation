# state variable that defines what events we would like to trigger on
# Leaving this field blank will result in the robot triggering when 
# anything touches the sides of the finger or an impact is detected
# with the hand/arm.
int8 trigger_conditions
# definitions for our various trigger_conditions values
# trigger on either acceleration contact or finger sensor side impact
int8 FINGER_SIDE_IMPACT_OR_ACC = 0
# tigger once  both slip and acceleration signals occur
int8 SLIP_AND_ACC = 1 
#  trigger on either slip, acceleration, or finger sensor side impact
int8 FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC = 2
# trigger only on slip information
int8 SLIP = 3
# trigger only on acceleration contact information
int8 ACC = 4 


# the amount of acceleration to trigger on (acceleration vector magnitude)
# Units = m/s^2
# The user needs to be concerned here about not setting the trigger too
# low so that is set off by the robot's own motions.
#
# For large rapid motions, say by a motion planner, 5 m/s^2 is a good level
# For small delicate controlled motions this can be set MUCH lower (try 2.0)
#
# NOTE: When moving the gripper joint (opening/closing the grippr)
# the high gearing of the PR2 gripper causes large acceleration vibrations
# which will cause triggering to occur. This is a known drawback of the PR2.
#
# NOTE: Leaving this value blank will result in a 0 m/s^2 trigger. If you
# are using a trigger_conditions value that returns on acceleration contact
# events then it will immediately exceed your trigger and return
float64 acceleration_trigger_magnitude


# the slip detector gain to trigger on (either finger) : try 0.01
# higher values decrease slip sensitivty (to a point)
# lower values increase sensitivity (to a point)
#
# NOTE: Leaving this value blank will result in the most sensitive slip level.
float64 slip_trigger_magnitude