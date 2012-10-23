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
    
# read yaml and modify properties 

in_stream = open(sys.argv[1], 'r')
rviz_yaml = yaml.load(in_stream)

rviz_yaml['Visualization Manager']['Global Options']['Fixed Frame'] = fixed_frame

for display in rviz_yaml['Visualization Manager']['Displays']:
    
    if display['Name'] == 'Map':
        display['Enabled'] = map_enabled

    if display['Name'] == 'People Tracking':
        display['Enabled'] = track_people

    if display['Name'] in ['Nav Global Path','Nav Obstacles']:
        display['Enabled'] = nav_enabled
        if nav_local:
            display['Topic'] = display['Topic'].replace('move_base_node','move_base_local_node')

# dump yaml to output file

out_stream = open(sys.argv[2], 'w')
yaml.dump(rviz_yaml, out_stream)
