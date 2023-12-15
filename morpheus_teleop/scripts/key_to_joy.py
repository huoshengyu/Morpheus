import rospy
import tf2_ros
import geometry_msgs.msg   
import sensor_msgs.msg 
import control_msgs.msg
import keyboard


class KeyToJoy():

    def __init__(self):
        self.joy_pub = rospy.Publisher("/joy", sensor_msgs.msg.Joy, queue_size=5)