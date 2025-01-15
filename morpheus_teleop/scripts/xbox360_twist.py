#! /usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
import std_msgs.msg
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
import trajectory_msgs.msg
from threading import Lock
from copy import deepcopy
from collections import deque
from scipy.spatial.transform import Rotation as R
from scipy.linalg import inv
from scipy import pi

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output
from onrobot_rg2ft_msgs.msg import RG2FTCommand
from utils import switch_controller, publish_joint_pos, command_robotiq2F85, command_onrobotRG2FT

class TeleopTwistJoy():
    def __init__(self):
        self.twist_topic = rospy.get_param("~twist_topic", "/joy/twist")

        self.twist_pub = rospy.Publisher(self.twist_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.gripper_pub_robotiq = rospy.Publisher('/robotiq_2f_85_gripper/control', Robotiq2FGripper_robot_output, queue_size=1)
        self.gripper_pub_onrobot = rospy.Publisher('/onrobot_rg2ft/command', RG2FTCommand, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)

        self.joy_msg = None
        self.joy_msg_mutex = Lock()

        self.linear_scale = 0.05
        self.angular_scale = 0.05

        self.input_min = 0.15
        self.input_max = 1.0
        self.output_max = 0.5

        self.twist_filter_size = 5  # Adjust this value for the moving average window size
        self.linear_x_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_y_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_z_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_x_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_y_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_z_buffer = deque(maxlen=self.twist_filter_size)
        self.buffer_list = [self.linear_x_buffer, self.linear_y_buffer, self.linear_z_buffer, self.angular_x_buffer, self.angular_y_buffer, self.angular_z_buffer]

        self.joint_pos_topic = "/joint_group_pos_controller"
        self.following_trajectory = False
        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.home = [0, -1.5708, 1.5708, -1.5708, -1.5708, 3.14]
        self.joint_pos_pub = rospy.Publisher(self.joint_pos_topic + "/command", std_msgs.msg.Float64MultiArray, queue_size=1)

    def get_joy_msg(self):
        # Retrieve joy_msg and return a safe copy
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()
        return msg

    def msg_to_axes(self, msg, linear_scale=None, angular_scale=None):
        # Convert joy_msg inputs to command outputs in 6 DOF
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        # Set a deadzone and a limit in input values
        for i in range(len(axes)):
            if abs(axes[i]) < self.input_min:
                axes[i] = 0
            if abs(axes[i]) > self.input_max:
                axes[i] = axes[i] / abs(axes[i])

        if linear_scale == None:
            linear_scale = self.linear_scale
        if angular_scale == None:
            angular_scale = self.angular_scale

        # Button remapping so that left joystick/buttons = translation, right joystick/buttons = rotation
        scaled_axes = np.array([ axes[0] * linear_scale, 
                                -axes[1] * linear_scale, 
                                ((buttons[4]) - (axes[2] < 0.0)) * linear_scale, 
                                -axes[4] * angular_scale, 
                                -axes[3] * angular_scale, 
                                -((buttons[5]) - (axes[5] < 0.0)) * angular_scale])

        # Enforce a safety limit on speed
        scaled_axes = np.clip(scaled_axes, -self.output_max, self.output_max)
        
        return scaled_axes

    def rearrange_axes(self, scaled_axes):
        rearranged_axes = scaled_axes
        # Rearrange the control axes to make the controls more intuitive
        try:
            r_control_linear = np.array([[ 1,  0,  0],
                                            [ 0,  1,  0],
                                            [ 0,  0,  1]])
            r_control_angular = np.array([[ 1,  0,  0],
                                            [ 0,  1,  0],
                                            [ 0,  0,  1]])

            rearranged_axes = np.concatenate((np.matmul(r_control_linear, scaled_axes[:3]), np.matmul(r_control_angular, scaled_axes[3:])))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Control axis rearrangement failed!")
        return rearranged_axes

    def publish(self, rotated_axes, msg):
        # Publish pre-processed commands to the twist topic
        buttons = list(msg.buttons)

        # If menu button is pressed, move to home
        if buttons[7] == 1:
            self.moveto(publisher=self.joint_pos_pub, joint_pos=self.home)
            return

        # Append inputs on each axis to the respective buffers
        buffer_zip = zip(self.buffer_list, rotated_axes)
        [buffer.append(axis) for buffer, axis in buffer_zip]

        # Obtain a moving average from each buffer and assign it to a new twist command
        twist = geometry_msgs.msg.Twist()
        twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z = [sum(buffer) / max(1,len(buffer)) for buffer in self.buffer_list]
        self.twist_pub.publish(twist)

        # Convert Xbox button press into open/close command
        # Inputs: 
        #   name            type                limits (Robotiq 2F-85)          limits(OnRobot RG2FT)
        #   position        float [0,50]        [0,50] mm                       [0,100] mm
        #   force           float [0,40]        [20,235] N, capped to 40        [0,40] N
        #   speed           float [0,150]       [20,150] mm/s                   (No speed control, ranges ~[180, 25] mm/s as position increases)
        position = None
        if buttons[0] == 1:
            position = 0 # Closed position, mm
        elif buttons[1] == 1:
            position = 50 # Open position, mm
        force = 20 # N
        speed = 100 # mm/s

        # Command Robotiq 2F-85 gripper
        if position is not None:
            command_robotiq2F85(publisher=self.gripper_pub_robotiq, position=position, force=force, speed=speed)

        # Command OnRobot RG2FT gripper
        if position is not None:
            command_onrobotRG2FT(publisher=self.gripper_pub_robotiq, position=position, force=force)

    def update(self, msg):
        if msg is not None:
            axes = self.msg_to_axes(msg)
            axes = self.rearrange_axes(axes)
            return axes
        else:
            return None

    def loop_once(self):
        if self.joy_msg is not None:
            axes = self.update(self.joy_msg)
            if axes is not None:
                self.publish(axes, self.joy_msg)

    def callback(self, msg):
        # Retrieve joy_msg
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()

    def moveto(self, publisher=None, joint_pos=None, duration=5):
        # Command robot to move to specified joint position, such as home position

        # Switch to position controller
        twist_controllers = ["cartesian_compliance_controller"]
        pos_controllers = ["pos_joint_traj_controller"]
        switch_controller(stop_controllers=twist_controllers, start_controllers=pos_controllers)
        self.following_trajectory = True

        # Publish a position command
        rospy.set_param(self.joint_pos_topic + "/joints", self.joint_names)
        joint_pos_msg = std_msgs.msg.Float64MultiArray()
        joint_pos_msg.data = joint_pos
        publisher.publish(joint_pos_msg)

        # Wait for completion
        rospy.sleep(duration)

        # Switch back to twist controller
        switch_controller(stop_controllers=pos_controllers, start_controllers=twist_controllers)
        self.following_trajectory = False

if __name__ == '__main__':
    rospy.init_node('xbox360_twist')

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        teleop_twist_joy.loop_once()
        rate.sleep()
    
    rospy.spin()
