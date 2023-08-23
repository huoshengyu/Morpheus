#! /usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
from threading import Lock
from copy import deepcopy
from collections import deque
from scipy.spatial.transform import Rotation as R
from scipy.linalg import inv
from scipy import pi

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
                if abs(axes[i]) > 1:
                    axes[i] = axes[i] / abs(axes[i])

            linear_scale = 0.05
            angular_scale = 0.05

            scaled_axes = [axes[1] * linear_scale, 
                            axes[0] * linear_scale, 
                            ((1 - (axes[5] + 1) / 2) - (1 - (axes[2] + 1) / 2)) * linear_scale, 
                            -axes[3] * angular_scale, 
                            axes[4] * angular_scale,
                            (buttons[2] - buttons[3]) * angular_scale]

            for i in range(len(scaled_axes)):
                if abs(scaled_axes[i]) > 0.5:
                    scaled_axes[i] = 0.5 * scaled_axes[i] / abs(scaled_axes[i])

            try:
                # Reorder controls to work properly with the rotation matrix (and be more intuitive)
                temp = scaled_axes[2]
                scaled_axes[2] = scaled_axes[0]
                scaled_axes[0] = temp

                # Lookup the transform from source_frame to target_frame
                transform = tf_buffer.lookup_transform("base_link", "tcp_link", rospy.Time(0), rospy.Duration(1.0))
                
                effector_rotation = [transform.transform.rotation.x, -transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

                r = R.from_quat(effector_rotation)
                r = r.inv()
                #print(r.as_matrix())

                scaled_axes[:3] = list(r.apply(scaled_axes[:3]))
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Transform lookup failed!")

            self.linear_x_buffer.append(scaled_axes[0])
            self.linear_y_buffer.append(scaled_axes[1])
            self.linear_z_buffer.append(scaled_axes[2])
            self.angular_x_buffer.append(scaled_axes[3])
            self.angular_y_buffer.append(scaled_axes[4])
            self.angular_z_buffer.append(scaled_axes[5])

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

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    teleop_twist_joy = TeleopTwistJoy()

    rate = rospy.Rate(60)  # 10hz
    while not rospy.is_shutdown():
        teleop_twist_joy.publish()
        rate.sleep()
    
    rospy.spin()
