#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data
from pathlib import Path
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
testPath = str(Path(__file__).parent.parent.absolute() / 'config/test_scene.urdf')
print(testPath)
testId = p.loadURDF(testPath,startPos,startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (100000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(testId)
print(cubePos,cubeOrn)
p.disconnect()
