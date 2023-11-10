#!/usr/bin/env python3

import sys
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
        rospy.init_node('collision')
        self.rate = rospy.Rate(10)

        # Save urdf and srdf to temp file so moveit robot model can be loaded
        urdf = rospy.get_param('/robot_description')
        srdf = rospy.get_param('/robot_description_semantic')

        urdf_path = "/root/catkin_ws/src/morpheus/morpheus_teleop/config/temp.urdf"
        srdf_path = "/root/catkin_ws/src/morpheus/morpheus_teleop/config/temp.srdf"

        with open(urdf_path,"w") as File:
            File.write(urdf)
        with open(srdf_path,"w") as File:
            File.write(srdf)

        # Load robot model so planning scene can be loaded
        self.robot_model = load_robot_model(urdf_path, srdf_path)

        # Load planning scene so collision request can be made
        self.planning_scene = PlanningScene(self.robot_model)

        # Load collision request so nearest distances can be queried
        self.collision_request = collision_detection.CollisionRequest()
        # Set the collision request to return distances, not just collisions
        self.collision_request.distance = True
        self.collision_request.contacts = True
        self.collision_request.verbose = True
        # Set the collision request to check on a specific link/group
        try:
            self.collision_request.group_name = rospy.get_param('/collision/group_name')
        except:
            self.collision_request.group_name = group_name
            rospy.set_param('/collision/group_name', group_name)
        # Set collision request to check for multiple bodies so we can isolate the correct pair
        self.collision_request.max_contacts = 20
        self.collision_request.max_contacts_per_pair = 1

        # Load collision result so queries can be stored
        self.collision_result = collision_detection.CollisionResult()
    
        # Set up distance publishers so the callback can forward values through them
        self.checking_publisher = rospy.Publisher('/collision/check', String, queue_size=10)
        self.distance_publisher = rospy.Publisher('/collision/distance', Float64, queue_size=10)
        self.contacts_publisher = rospy.Publisher('/collision/contacts', String, queue_size=10)

        # Set up scene subscriber to query scene for current robot state
        self.planning_scene_msg = rospy.wait_for_message('/move_group/monitored_planning_scene', PlanningSceneMsg, timeout=5)
        self.scene_subscriber = rospy.Subscriber('/move_group/monitored_planning_scene', PlanningSceneMsg, self.callback)
        self.check_out = None

        #while not rospy.is_shutdown():
            #self.a.publish('help')
            #self.update(self.planning_scene)
            #self.publish_distance()
            #rate.sleep()
        

    def update(self, planning_scene_msg):

        try:
            # Check for target update
            self.collision_request.group_name = rospy.get_param('/collision/group_name')
            # Update planning scene
            self.planning_scene_msg = planning_scene_msg
            self.planning_scene.setCurrentState(self.planning_scene_msg.robot_state)
            # Input a collision request to check collision
            self.check_out = self.planning_scene.checkCollision(self.collision_request, self.collision_result)
            
        except:
            rospy.logwarn("Collision lookup failed!")


    def callback(self, planning_scene_msg):
        self.update(planning_scene_msg)
        self.publish()
        

    def publish(self):
        distance = self.collision_result.distance
        if (distance > 2**1023 * (2**53 - 1) / 2**53):
            distance = distance - (2**1023 * (2**53 - 1) / 2**52)
        
        self.checking_publisher.publish(str(self.check_out))
        self.distance_publisher.publish(distance)
        self.contacts_publisher.publish(str(self.collision_result.contacts))

        rospy.loginfo(distance)
        rospy.loginfo(str(self.collision_result.contacts))
    

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


    def spin(self):
        rospy.spin()

def main(group_name="teapot"):
    Node(group_name=group_name).spin()

if __name__ == '__main__':
    main()
