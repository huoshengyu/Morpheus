#!/usr/bin/env python3

import pybullet as p
import rospkg
import time
import pybullet_data
import subprocess
from custom_ros_tools.config import ros_package_path
from pathlib import Path
from urdf_parser_py.urdf import URDF
from custom_ros_tools.config import replace_package

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)

#print(ros_package_path("ur_description"))

xacroPath = "/root/catkin_ws/src/homestri_ur/homestri_description/urdf/scenes/a_bot_scene.xacro"
urdfPath = "/root/catkin_ws/src/morpheus/morpheus_teleop/config/test_urdf.urdf"
urdf = open(urdfPath, "w")

subprocess.call(['rosrun', 'xacro', 'xacro', xacroPath], stdout=urdf)

#testId = p.loadURDF(urdfPath,startPos,startOrientation)
testId = URDF.from_xml_file(urdfPath)
testStr = testId.to_xml_string()
#print(testId)

rplPath = str(urdfPath) + ".replace"
with open(rplPath,"w") as rplFile:
    for line in testStr.splitlines():

        # Check if package statement in line
        if 'package://' in line:
            # True -> replace package with absolute path
            idx = line.find("package://")
            package_name = line[idx:].split('/')[2]
            abs_path = ros_package_path(package_name)
            old = "package://" + package_name
            new = abs_path
            line_out = line.replace(old, new)
        else:
            # False -> no package statement, use line
            line_out = line

        # Write line to new temp file
        rplFile.write(line_out)

rplId = p.loadURDF(rplPath,startPos,startOrientation)

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (100000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(testId)
print(cubePos,cubeOrn)
p.disconnect()
