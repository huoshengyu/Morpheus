#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState, PlanningScene as PlanningSceneMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from pymoveit_core import load_robot_model, collision_detection

import tf2_ros
import geometry_msgs.msg

class Node:

    def __init__(self, group_name=None):
        # Init node and commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('collision_adder')

        # Init RobotCommmander, PlanningSceneInterface, MoveGroupCommander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")

        # Add mesh to PlanningScene
        rospy.sleep(1)
        filename = "/root/catkin_ws/src/morpheus/morpheus_teleop/meshes/components/collision/teapot.obj"
        self.add_mesh(filename, "teapot", x=1)

        #while not rospy.is_shutdown():
            #self.a.publish('help')
            #self.update(self.planning_scene)
            #self.publish_distance()
            #rate.sleep()

    def add_mesh(self, filename, name="", x=0, y=0, z=0, rx=0, ry=0, rz=0, rw=0):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.robot.get_planning_frame()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = rx
        pose.pose.orientation.y = ry
        pose.pose.orientation.z = rz
        pose.pose.orientation.w = rw
        self.scene.add_mesh(name, pose, filename)

def main(group_name="teapot"):
    Node(group_name=group_name)

if __name__ == '__main__':
    main()
