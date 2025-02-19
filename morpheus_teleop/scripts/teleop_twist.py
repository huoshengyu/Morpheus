#! /usr/bin/env python3

# General Packages
import numpy as np
import subprocess
# ROS Packages
import rospy
import tf2_ros
import moveit_commander
# ROS Messages
import std_msgs.msg
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg
# Individual Imports
from threading import Lock
from copy import deepcopy
from collections import deque
from scipy.spatial.transform import Rotation as R
from scipy.linalg import inv
from scipy import pi
# Local Imports
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand
from utils import switch_controller, list_controllers, command_robotiq2F85, command_onrobotRG2FT

class TeleopTwist():
    def __init__(self):
        return NotImplementedError

    def publish(self, rotated_axes, msg):
        return NotImplementedError

    def update(self, msg):
        return NotImplementedError
    
    def loop_once(self):
        return NotImplementedError

    def callback(self, msg):
        return NotImplementedError

    def moveto(self, joint_pos=None):
        return NotImplementedError

if __name__ == '__main__':
    rospy.init_node('teleop_twist')

    teleop_twist = TeleopTwist()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist.loop_once()
        rate.sleep()
    
    rospy.spin()
