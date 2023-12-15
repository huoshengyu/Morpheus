#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

class Node:

    def __init__(self):

        # Init node and commander
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("collision_checker", anonymous=True)

        # Init RobotCommmander, PlanningSceneInterface, MoveGroupCommander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")

        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")

        print(self.scene.get_known_object_names())

def main():
    Node()

if __name__=='__main__':
    main()