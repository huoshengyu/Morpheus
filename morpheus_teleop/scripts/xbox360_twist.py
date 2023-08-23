#! /usr/bin/env python3

import rospy
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
from threading import Lock
from copy import deepcopy
from collections import deque

from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

class TeleopTwistJoy():
    def __init__(self):
        twist_topic = rospy.get_param("~twist_topic", "/joy/twist")

        self.twist_pub = rospy.Publisher(twist_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.gripper_pub = rospy.Publisher('/robotiq_2f_85_gripper/control', Robotiq2FGripper_robot_output, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)
        
        self.joy_msg = None
        self.joy_msg_mutex = Lock()

        self.twist_filter_size = 5  # Adjust this value for the moving average window size
        self.linear_x_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_y_buffer = deque(maxlen=self.twist_filter_size)
        self.linear_z_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_x_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_y_buffer = deque(maxlen=self.twist_filter_size)
        self.angular_z_buffer = deque(maxlen=self.twist_filter_size)

    def publish(self):
        self.joy_msg_mutex.acquire()
        msg = deepcopy(self.joy_msg)
        self.joy_msg_mutex.release()

        if msg is not None:
            axes = list(self.joy_msg.axes)
            buttons = list(self.joy_msg.buttons)

            for i in range(len(axes)):
                if abs(axes[i]) < 0.15:
                    axes[i] = 0

            linear_scale = 0.05
            angular_scale = 0.05

            self.linear_x_buffer.append(axes[1] * linear_scale)
            self.linear_y_buffer.append(axes[0] * linear_scale)
            self.linear_z_buffer.append(((1 - (axes[5] + 1) / 2) - (1 - (axes[2] + 1) / 2)) * linear_scale)
            self.angular_x_buffer.append(-axes[3] * angular_scale)
            self.angular_y_buffer.append(axes[4] * angular_scale)
            self.angular_z_buffer.append((buttons[2] - buttons[3]) * angular_scale)

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = sum(self.linear_x_buffer) / len(self.linear_x_buffer)
            twist.linear.y = sum(self.linear_y_buffer) / len(self.linear_y_buffer)
            twist.linear.z = sum(self.linear_z_buffer) / len(self.linear_z_buffer)
            twist.angular.x = sum(self.angular_x_buffer) / len(self.angular_x_buffer)
            twist.angular.y = sum(self.angular_y_buffer) / len(self.angular_y_buffer)
            twist.angular.z = sum(self.angular_z_buffer) / len(self.angular_z_buffer)

            command = Robotiq2FGripper_robot_output()
            command.rACT = 0x1
            command.rGTO = 0x1 # go to position
            command.rATR = 0x0 # No emergency release
            command.rSP = 128 # speed
            command.rPR = 0x0 # position
            command.rFR = 30 # effort
            if buttons[0] == 1:
                command.rPR = 230 # position
                self.gripper_pub.publish(command)
            elif buttons[1] == 1:
                command.rPR = 0 # position
                self.gripper_pub.publish(command)

            self.twist_pub.publish(twist)

    def callback(self, msg):
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()

if __name__ == '__main__':
    rospy.init_node('xbox360_twist')

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60)  # 10hz
    while not rospy.is_shutdown():
        teleop_twist_joy.publish()
        rate.sleep()
    
    rospy.spin()
