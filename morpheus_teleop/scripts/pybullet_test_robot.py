#!/usr/bin/env python3

import pybullet as p
import rospkg
import time
import pybullet_data
import subprocess
from pathlib import Path
from urdf_parser_py.urdf import URDF

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
xacroPath = "/root/catkin_ws/src/homestri-ur/homestri_description/urdf/scenes/a_bot_scene.xacro"
urdfPath = "/root/catkin_ws/src/morpheus/morpheus_teleop/config/test_urdf.urdf"
urdf = open(urdfPath, "w")
subprocess.call(['rosrun', 'xacro', 'xacro', xacroPath], stdout=urdf)
testId = URDF.from_xml_file(urdfPath)
#testId = p.loadURDF(urdfPath,startPos,startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (100000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(testId)
print(cubePos,cubeOrn)
p.disconnect()
