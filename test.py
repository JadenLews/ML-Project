import pybullet as p
import time
import pybullet_data

layout = [
    "#####",
    "#A  #",
    "# # #",
    "#  G#",
    "#####"
]



physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
p.loadURDF("wall.urdf", [0, -2, 0.5])
p.loadURDF("cube_small.urdf", [1, 0, 0.5])  # small box
p.loadURDF("cube.urdf", [2, 0, 0.5])        # larger box
p.loadURDF("test.urdf", [3, 0, 0.5])
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
