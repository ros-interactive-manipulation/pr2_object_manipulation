#!/usr/bin/python

import sys
import yaml
#from stgit.git import switch

def print_usage():
    print "usage:\nadjust_rviz_config.py in_file out_file stereo nav_enabled nav_local sim\n(where stereo, nav_enabled, nav_local, and sim are true or false)\n"

def str2bool(name,v):
    if v not in ['true', 'false']:
        print name + "was not true or false!"
        print_usage()
        sys.exit(1)
        
    vb = v.lower() == 'true'
    print name, " = ", vb
    return vb
  

if len(sys.argv) < 7:
    print_usage()
    sys.exit(1)

# parse cmdline params

stereo = str2bool( 'stereo', sys.argv[3] )
nav_enabled = str2bool( 'nav_enabled', sys.argv[4] )
nav_local = str2bool( 'nav_local', sys.argv[5] )
sim = str2bool( 'sim', sys.argv[6] )
track_people = str2bool( 'track_people', sys.argv[7] )

# set derived params

if nav_enabled and not nav_local:
    fixed_frame = '/map'
    map_enabled = True
else:
    fixed_frame = '/odom_combined'
    map_enabled = False
    
if sim:
    kinect_color_topic = '/head_mount_kinect/rgb/image_raw'
    kinect_color_hint = 'raw'
    kinect_depth_topic = '/head_mount_kinect_rgb/depth/image_raw'
    kinect_depth_hint = 'raw'
else:
    kinect_color_topic = '/head_mount_kinect/rgb/image_color'
    kinect_color_hint ='compressed'
    kinect_depth_topic = '/head_mount_kinect/depth_registered/image'
    kinect_depth_hint = 'compressedDepth'

# read yaml and modify properties 

in_stream = open(sys.argv[1], 'r')
rviz_yaml = yaml.load(in_stream)

rviz_yaml['Visualization Manager']['Global Options']['Fixed Frame'] = fixed_frame

for display in rviz_yaml['Visualization Manager']['Displays']:
    
    if display['Name'] == 'People Tracking':
        display['Enabled'] = track_people

    if display['Name'] == 'Navigation':
        for nav_display in display['Displays']:

            if nav_display['Name'] == 'Map':
                nav_display['Enabled'] = map_enabled

            if nav_display['Name'] in ['Nav Global Path','Nav Obstacles']:
                nav_display['Enabled'] = nav_enabled
                if nav_local:
                    nav_display['Topic'] = nav_display['Topic'].replace('move_base_node','move_base_local_node')

    if display['Name'] == 'Kinect Stream':
        display['Color Image Topic'] = kinect_color_topic
        display['Color Transport Hint'] = kinect_color_hint
        display['Depth Map Topic'] = kinect_depth_topic
        display['Depth Map Transport Hint'] = kinect_depth_hint

    if display['Name'] == 'Head Camera':
        display['Image Topic'] = kinect_color_topic
        display['Transport Hint'] = kinect_color_hint

# dump yaml to output file

out_stream = open(sys.argv[2], 'w')
yaml.dump(rviz_yaml, out_stream)
