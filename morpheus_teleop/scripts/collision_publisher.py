#!/usr/bin/env python3

import rospy
import moveit_commander
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState, PlanningScene as PlanningSceneMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit.core.planning_scene import PlanningScene
from moveit.core.robot_model import RobotModel
from pymoveit_core import load_robot_model, collision_detection

import rospy
import tf2_ros
import geometry_msgs.msg

class Node:

    def __init__(self):
        # Init node
        rospy.init_node('distance_node')
        rate = rospy.Rate(10)

        #self.a = rospy.Publisher('test', String, queue_size=10)
        #self.b = rospy.Subscriber('test', String, self.test_callback)

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

        # Load collision result so queries can be stored
        self.collision_result = collision_detection.CollisionResult()
    
        # Set up distance publisher so the callback can forward values through it
        self.distance_publisher = rospy.Publisher('/collision_distance', String, queue_size=10)

        # Set up scene subscriber to query scene for current robot state
        self.planning_scene_msg = rospy.wait_for_message('/move_group/monitored_planning_scene', PlanningSceneMsg, timeout=5)
        self.scene_subscriber = rospy.Subscriber('/move_group/monitored_planning_scene', PlanningSceneMsg, self.distance_callback)
        self.distance = None

        #while not rospy.is_shutdown():
            #self.a.publish('help')
            #self.update_distance(self.planning_scene)
            #self.publish_distance()
            #rate.sleep()

    def test_callback(self, string):
        rospy.loginfo('oh')

    def update_distance(self, planning_scene_msg):

        try:
            # Update planning scene
            self.planning_scene_msg = planning_scene_msg
            self.planning_scene.setCurrentState(self.planning_scene_msg.robot_state)
            # Input a collision request to check collision
            self.planning_scene.checkCollision(self.collision_request, self.collision_result)
            
        except:
            rospy.logwarn("Collision lookup failed!")

    def distance_callback(self, planning_scene):
        self.distance = self.update_distance(planning_scene)
        self.publish_distance()
        

    def publish_distance(self):
        self.distance_publisher.publish(str(self.collision_result.distance))
    

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
