#! /usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
import trajectory_msgs.msg

from controller_manager_msgs.srv import SwitchController, ListControllers

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand    

def switch_controller(start_controllers=[], stop_controllers=[], strictness=2, start_asap=False, timeout=5):
    rospy.wait_for_service('controller_manager/switch_controller')
    success = False
    try:
        switch_controller = rospy.ServiceProxy(
                            'controller_manager/switch_controller', SwitchController)
        success = switch_controller(start_controllers, 
                                    stop_controllers,
                                    strictness,
                                    start_asap,
                                    timeout)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return success
    
def list_controllers():
    rospy.wait_for_service('controller_manager/list_controllers')
    success = False
    try:
        list_controllers = rospy.ServiceProxy(
                            'controller_manager/list_controllers', ListControllers)
        success = list_controllers()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    return success

def publish_joint_pos(publisher=None, joint_names=[], joint_pos=[], duration=5):
    # Publish a position command
    point = trajectory_msgs.msg.JointTrajectoryPoint()
    point.positions = joint_pos
    msg = trajectory_msgs.msg.JointTrajectory()
    msg.joint_names = joint_names
    msg.points = [point]
    publisher.publish(msg)

def command_robotiq2F85(publisher=None, position=0, force=0, speed=0, publish=True):
    # Command Robotiq 2F-85 gripper
    # Inputs:
    #   position        float [0,50]        [0,50] mm
    #   force           float [0,40]        [20,235] N, capped to 40
    #   speed           float [0,150]       [20,150] mm/s

    # Enforce limits
    position = np.clip(position, 0, 50)
    force = np.clip(force, 0, 40)
    speed = np.clip(speed, 0, 150)

    # Write command
    command = Robotiq2FGripper_robot_output()
    command.rACT = 0x1 # Deactivate / Activate {0,1}
    command.rGTO = 0x1 # Stop / "Go To" {0,1}
    command.rATR = 0x0 # Normal / Emergency auto-release {0,1}
    command.rSP = int(255 * speed / 150) # Minimum / Maximum speed [0,255] (20 to 150 mm/s)
    command.rPR = int(255 * (1 - (position / 50))) # Open / Closed position [0,255] (50 to 0 mm)
    command.rFR = int(255 * force / 255) # Minimum / Maximum force [0,255] (20 to 235 N)
    if publish:
        try:
            publisher.publish(command)
        except ... as e:
            print("Failed to publish Robotiq 2F85 command")
    return command

def command_onrobotRG2FT(publisher=None, position=0, force=0, publish=True):
    # Command OnRobot RG2FT gripper
    # Inputs:
    #   position        float [0,100]       [0,100] mm
    #   force           float [0,40]        [0,40] N 
    #   ### No direct speed control due to hardware constraints, ranges ~[180, 25] mm/s as position increases ###
    #   ### See manual with MODBUS register information below: ###
    #   ### https://onrobot.com/sites/default/files/documents/User_Manual_for_TECHMAN_OMRON_TM_v1.05_EN_0.pdf ###

    # Enforce limits
    position = np.clip(position, 0, 100)
    force = np.clip(force, 0, 40)

    # Make a stop command to interrupt the current motion
    stop = RG2FTCommand()
    stop.Control = 0x0000 # Stop / Grip {0,1} (Gripper completes command before starting next one)
    stop.TargetWidth = 950 # Closed / Open position [0,1000] (0 to 100 mm)
    stop.TargetForce = 0 # Minimum Maximum force [0,400] (0 to 40 N)
    # Make a motion command to move the OnRobot RG2FT gripper
    command = RG2FTCommand()
    command.Control = 0x0001 # Stop / Grip {0,1} (Gripper completes command before starting next one)
    command.TargetWidth = int(10 * position) # Closed / Open position [0,1000] (0 to 100 mm)
    command.TargetForce = int(10 * force) # Minimum / Maximum force [0,400] (0 to 40 N)
    publisher.publish(stop)
    publisher.publish(command)
    if publish:
        try:
            publisher.publish(command)
        except ... as e:
            print("Failed to publish OnRobot RG2FT command")
    return command