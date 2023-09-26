import rospy
import moveit_commander
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class Node:

    def __init__(self):

        # Init Node
        rospy.init_node("collision_checker", anonymous=True)

        # Init joint state subscriber
        

        # Init planning scene interface
        moveit_commander.rosccp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = '/move_group'
        move_group = movit_commander.MoveGroupCommander(group_name)

        scene = 

        # Set up distance publisher
        self.pub = rospy.Publisher(distance_topic, Distance, queue_size=10)
    
    def getCollisionDistance(self, group_name=self.group_name, constraints=None)


    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__=='__main__':
    main()