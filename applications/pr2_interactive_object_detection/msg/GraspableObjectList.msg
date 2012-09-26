object_manipulation_msgs/GraspableObject[] graspable_objects

#Information required for visualization

sensor_msgs/Image image
sensor_msgs/CameraInfo camera_info

#Holds a single mesh for each recognized graspable object, an empty mesh otherwise
arm_navigation_msgs/Shape[] meshes

#pose to transform the frame of the clusters/object poses into camera coordinates
geometry_msgs/Pose reference_to_camera
