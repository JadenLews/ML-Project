import pybullet as p
import time
import pybullet_data
import pickle
import copy
import numpy as np

mapFilePath = "./map-pieces/"
mapFileType = ".urdf"

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)

# map_meta = {
#     "agent_start": [x, y, z],
#     "goal": [gx, gy, gz],
#     "obstacles": [...],
# }
# }


#functions
def move_agent(dx, dy, dz, id):
    pos, orn = p.getBasePositionAndOrientation(id)
    new_pos = [pos[0] + dx, pos[1] + dy, pos[2] + dz]
    p.resetBasePositionAndOrientation(id, new_pos, orn)

def check_goal_collision(agent_id, goal_id):
    contacts = p.getContactPoints(bodyA=agent_id, bodyB=goal_id)
    if contacts is None:
        return False
    return len(contacts) > 0

def check_explored_areas(map, radius, agent_position):
    p1 = np.array(agent_position)
    for indx, element in enumerate(map):
        print(element)
        elem_pos = p.getBasePositionAndOrientation(element[0])[0]
        p2 = np.array(elem_pos)
        squared_dist = np.sum((p1-p2)**2, axis=0)
        dist = np.sqrt(squared_dist)
        if element[-1] == "?":
            if dist <= radius:
                if element[1] == "G":
                    p.removeBody(element[0])
                    goal_id = p.loadURDF(mapFilePath + "goal" + mapFileType, elem_pos)
                    map[indx][0] = goal_id
                    map[indx][-1] = "G"
                else:
                    p.removeBody(element[0])
                    map[indx][0] = p.loadURDF(mapFilePath + mapCells.get(element[1]) + mapFileType, elem_pos)
                    map[indx][-1] = element[1]
    return map_objects


#layout initialize
layout = []
agent_pos = [0, 0, 0]

with open("map.txt") as f:
    layer = []
    tempz = 0
    tempy = 0
    for line in f:
        line = list(line.strip())
        if 'A' in line:
            agent_pos = [line.index('A'), tempy, tempz]

        if not line:
            layout.append(layer)
            layer = []
            tempy = 0
            tempz += 1
        else:
            layer.append(line)
            tempy += 1
    if layer:
        layout.append(layer)



#map to 3d space
agent_file = "agent"
object_ids = []
mapCells = {"#": "floor", "-": "width_wall", "|": "depth_wall", "1": "tl-corner", "2": "tr-corner", "3": "bl-corner", "4": "br-corner", "?": "unknown", "G": "goal"}
skipCells = {"A": agent_file, " ": "nothing"}

goal_id = ""
agent_id = ""

hidden = mapFilePath + "unknown" + mapFileType



# load mapbh
map_objects = []

for z, layer in enumerate(layout):
    for y, row in enumerate(layer):
        for x, cell in enumerate(row):
            if cell in skipCells.keys():
                if cell == "A":
                    agent_pos = [x, y, z]
                    piece = mapFilePath + skipCells.get("A") + mapFileType
                    agent_id = p.loadURDF(piece, agent_pos)
            elif cell in mapCells.keys():
                piece = mapFilePath + mapCells.get(cell) + mapFileType
                map_objects.append([p.loadURDF(hidden, [x, y, z]), cell, "?"])


print(map_objects)


print("Controls: w = up, s = down, a = left, d = right, q = quit")

trajectory = []
action_map = {"w":0, "a":1, "s":2, "d":3}

state = agent_pos
while True:
    state = list(p.getBasePositionAndOrientation(agent_id)[0])

    map_objects = check_explored_areas(map_objects, 1.6, state)
    cmd = input("Enter move: ").strip().lower()

    if cmd == 'w':
        move_agent(0, 1, 0, agent_id)
    elif cmd == 's':
        move_agent(0, -1, 0, agent_id)
    elif cmd == 'a':
        move_agent(-1, 0, 0, agent_id)
    elif cmd == 'd':
        move_agent(1, 0, 0, agent_id)
    elif cmd == 'q':
        print("Exiting")
        break
    else:
        print("Invalid command")
    p.stepSimulation()

    next_state = list(p.getBasePositionAndOrientation(agent_id)[0])
    action = action_map.get(cmd)

    goal_check = False
    print(goal_id, "goal hjere")
    if goal_id:
        goal_check = check_goal_collision(agent_id, goal_id)
    reward = 1 if goal_check else -0.01
    done = goal_check

    time.sleep(1/240)

    if done:
        print("goal")
        with open("manual_trajectory.pkl", "wb") as f:
            pickle.dump(trajectory, f)
        break


print(layout)
p.disconnect()
