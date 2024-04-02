#! /usr/bin/env python3

import numpy as np
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

class GoalHaptics():
    def __init__(self):
        joy_feedback_topic = rospy.get_param("~joy_feedback_topic", "/joy/set_feedback")
        goal_haptics_topic = rospy.get_param("~goal_haptics_topic", "/goal_haptics")

        self.joy_feedback_pub = rospy.Publisher(joy_feedback_topic, sensor_msgs.msg.JoyFeedbackArray, queue_size=5)
        self.goal_haptics_pub = rospy.Publisher(goal_haptics_topic, sensor_msgs.msg.JoyFeedbackArray, queue_size=5)
        self.joy_sub = rospy.Subscriber("/joy", sensor_msgs.msg.Joy, self.callback)

        self.joy_msg = None
        self.joy_msg_mutex = Lock()

        self.on = False
        self.already_toggled = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.goal_translation = [0, 0, 0] # (x, y, z) 
        self.goal_quaternion = [0, 0, 0, 0] # (X, Y, Z, W)

        # Attempt to retrieve the name of a goal preset from parameter server
        self.goal_name = rospy.get_param("/goal_name", None)
        if self.goal_name is not None:
            rospy.loginfo("Using goal name from parameter server")
        else:
            self.goal_name = "keyhole"
            rospy.loginfo("Using default goal name")

        # Attempt to retrieve ideal goal pose from parameter server
        self.goal_transform_dict = rospy.get_param("/goal_transform", None)
        if self.goal_transform_dict is not None:
            rospy.loginfo("Using goal transform from parameter server")
            # Retrieve translation and quaternion from the goal transform dict
            goal_transform = self.goal_transform_dict[self.goal_name]
            self.goal_translation = [goal_transform['translation']['x'], 
                                    goal_transform['translation']['y'], 
                                    goal_transform['translation']['z']]
            self.goal_quaternion = [goal_transform['rotation']['x'], 
                                    goal_transform['rotation']['y'], 
                                    goal_transform['rotation']['z'], 
                                    goal_transform['rotation']['w'],]
        else:
            rospy.loginfo("Using default goal transform")

        self.translation_weight = 0.25
        self.rotation_weight = 0.0
        self.cost = None

        self.force_feedback_msg = None
        self.force_feedback_msg_mutex = Lock()

        self.force_feedback_filter_size = 5  # Adjust this value for the moving average window size
        self.force_feedback_buffer = deque(maxlen=self.force_feedback_filter_size)


    def update(self):
        transform = None
        while transform == None:
            try:
                transform = self.tf_buffer.lookup_transform("world", "tcp_link", rospy.Time(0), rospy.Duration(5.0))
            except:
                rospy.loginfo("Waiting for tcp_link transform")
        
        # Calculate final - initial translation
        translation_as_list = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
        translation_to_goal = np.subtract(self.goal_translation, translation_as_list)
        translation_to_goal_magnitude = np.linalg.norm(translation_to_goal)

        # Calculate final - initial rotation
        rotation_as_quat = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        rotation_to_goal = R.concatenate((R.from_quat(self.goal_quaternion), R.from_quat(rotation_as_quat).inv()))
        rotation_to_goal_magnitude = rotation_to_goal.magnitude()
        
        # Calculate a cost for the current pose, as a function of distance to goal in translation and rotation
        self.cost = np.sum(self.translation_weight * translation_to_goal_magnitude + self.rotation_weight * rotation_to_goal_magnitude)

    def publish(self):
        if self.cost is not None:
            # Clear force feedback message and make new array
            self.joy_feedback_array_msg = sensor_msgs.msg.JoyFeedbackArray()
            joy_feedback_array = []

            # Make new force feedback message and add to array
            joy_feedback_msg = sensor_msgs.msg.JoyFeedback()
            joy_feedback_msg.type = 1 # Rumble
            joy_feedback_msg.id = 0
            # If cost is below threshold, set feedback intensity to 0
            rospy.loginfo("Cost was " + str(self.cost))
            if self.cost < 0.01 or not self.on:
                joy_feedback_msg.intensity = 0.0
            # Else, make sure intensity does not exceed 1.0
            else:
                joy_feedback_msg.intensity = min(1.0, self.cost)
            joy_feedback_array.append(joy_feedback_msg)

            # Make new LED feedback message and add to array (Current controller does not have LED)
            led_feedback_msg = sensor_msgs.msg.JoyFeedback()
            led_feedback_msg.type = 0 # LED
            led_feedback_msg.id = 0
            # If cost is below threshold, set feedback intensity to 0
            if self.cost < 0.01 or not self.on:
                led_feedback_msg.intensity = 0.0
            # Else, make sure intensity does not exceed 1.0
            else:
                led_feedback_msg.intensity = min(1.0, self.cost)
            joy_feedback_array.append(led_feedback_msg)

            # Assign new array to the joy_feedback_array_msg
            self.joy_feedback_array_msg.array = joy_feedback_array

            # Publish message to joy feedback topic
            self.joy_feedback_pub.publish(self.joy_feedback_array_msg)
            # Publish message to goal haptics topic (change message if non-joystick haptic device needs to convey different info)
            self.goal_haptics_pub.publish(self.joy_feedback_array_msg)

    def loop_once(self):
        if self.joy_msg is not None:
            buttons = list(self.joy_msg.buttons)
            # Check if control frame needs to be swapped
            # buttons[6] = share button
            if (buttons[6]==1) and (not self.already_toggled):
                self.already_toggled = True
                self.on = not self.on
            elif (buttons[6]==0):
                self.already_toggled = False
        self.update()
        self.publish()
    
    def callback(self, msg):
        # Retrieve joy_msg
        self.joy_msg_mutex.acquire()
        self.joy_msg = msg
        self.joy_msg_mutex.release()

if __name__ == '__main__':
    rospy.init_node('goal_haptics')

    goal_haptics = GoalHaptics()

    rate = rospy.Rate(60)  # hz
    while not rospy.is_shutdown():
        goal_haptics.loop_once()
        rate.sleep()
    
    rospy.spin()
