# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/indigo/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/indigo/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/af/ros_demo/devel_isolated/crab_joint_publisher;/home/af/ros_demo/devel_isolated/crab_body_kinematics;/home/af/ros_demo/devel_isolated/crab_msgs;/home/af/ros_demo/devel_isolated/crab_leg_kinematics;/home/af/ros_demo/devel_isolated/crab_gait;/home/af/ros_demo/devel_isolated/dynamxiel_action_client;/home/af/ros_demo/devel_isolated/dynamixel_tutorials;/home/af/ros_demo/devel_isolated/dynamixel_msgs;/home/af/ros_demo/devel_isolated/dynamixel_motor;/home/af/ros_demo/devel_isolated/dynamixel_driver;/home/af/ros_demo/devel_isolated/dynamixel_controllers;/home/af/ros_demo/devel_isolated/rrbot_gazebo;/home/af/ros_demo/devel_isolated/rrbot_description;/home/af/ros_demo/devel_isolated/rrbot_control;/home/af/ros_demo/devel_isolated/pr2_controller_manager;/home/af/ros_demo/devel_isolated/pr2_mechanism_diagnostics;/home/af/ros_demo/devel_isolated/my_controller_pkg;/home/af/ros_demo/devel_isolated/pr2_controller_interface;/home/af/ros_demo/devel_isolated/pr2_mechanism_model;/home/af/ros_demo/devel_isolated/pr2_mechanism;/home/af/ros_demo/devel_isolated/pr2_hardware_interface;/home/af/ros_demo/devel_isolated/pluginlib_calculator;/home/af/ros_demo/devel_isolated/my_pluginlib_calculator;/home/af/ros_demo/devel_isolated/gazebo_tutorials;/home/af/moveit/devel;/home/af/hexapod_ws/devel;/home/af/catkin_ws/devel;/opt/ros/indigo".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/af/hexapod_ws/src/crab_proj/qtbuild/devel/env.sh')

output_filename = '/home/af/hexapod_ws/src/crab_proj/qtbuild/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
