import pybullet as p
import time
import pybullet_data

# map_meta = {
#     "agent_start": [x, y, z],
#     "goal": [gx, gy, gz],
#     "obstacles": [...],
# }
layyout = []
with open("map.txt") as f:
    layer = []
    for line in f:
        print(line, "hhhhhhh")
        line = line.strip()
        print(line, "efsdfdsfsdfdsfsdsdfs")
        if not line:
            layyout.append(layer)
            layer = []
        else:
            layer.append(line)
    if layer:
        layyout.append(layer)

layout = layyout
print(layout)

object_ids = []
skipCells = ["A", " "]
mapFilePath = "./map-pieces/"
mapFileType = ".urdf"
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
for z, layer in enumerate(layout):
    for y, row in enumerate(layer):
        for x, cell in enumerate(row):
            print(x, y, z)
            piece = ""
            print(cell, "error ceell")
            if cell == "-":
                piece = "width_wall"
            if cell == "#":
                piece = "floor"
            if cell == "|":
                piece = "depth_wall"
            if cell == "1":
                piece = "tl-corner"
            if cell == "2":
                piece = "tr-corner"
            if cell == "3":
                piece = "bl-corner"
            if cell == "4":
                piece = "br-corner"
            if cell == "G":
                piece = "goal"
            elif cell == "A":
                agent_start = [x, y, 0.5]
                agent_id = p.loadURDF("r2d2.urdf", [x, y, z + .5])
                pass
            if cell not in skipCells:
                p.loadURDF(mapFilePath + piece + mapFileType, [x, y, z + .5]) 
                object_ids.append(p.loadURDF(mapFilePath + piece + mapFileType, [x, y, z + .5]) )


#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()
