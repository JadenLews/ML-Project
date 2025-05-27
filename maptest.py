import pybullet as p
import time
import pybullet_data
import pickle
import copy

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

def check_explored_areas(hidden, radius, location, map_ids):
    x0, y0, z0 = location

    for dz in range(-radius, radius + 1):
        z = z0 + dz
        if 0 <= z < len(hidden):
            for dy in range(-radius, radius + 1):
                y = y0 + dy
                if 0 <= y < len(hidden[z]):
                    for dx in range(-radius, radius + 1):
                        x = x0 + dx
                        if 0 <= x < len(hidden[z][y]):
                            if hidden[z][y][x] == "?" and layout[z][y][x] not in skipCells.keys():
                                p.removeBody(id_layout[z][y][x])
                                print(z,y,x, layout[z][y][x])
                                print(layout)
                                piece = mapFilePath + mapCells.get(layout[z][y][x]) + mapFileType
                                print(piece)
                                id_layout[z][y][x] = p.loadURDF(piece, [x, y, z])
                            if layout[z][y][x] == ' ' and hidden[z][y][x] == "?":
                                p.removeBody(id_layout[z][y][x])
                            hidden[z][y][x] = layout[z][y][x]
    print(hidden)

    return hidden


#layout initialize
layout = []
agent_z = 0
agent_y = 0
agent_x = 0
with open("map.txt") as f:
    layer = []
    tempz = 0
    tempy = 0
    for line in f:
        line = list(line.strip())
        if 'A' in line:
            agent_x = line.index('A')
            agent_y = tempy
            agent_z = tempz

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
print(layout)
print("i think agent index is ", agent_x, agent_y, agent_z)

hidden = []
for layer in layout:
    hidden_inner = []
    for row in layer:
        string_inner = ["?"] * len(row)
        hidden_inner.append(string_inner)
    hidden.append(hidden_inner)


    


#map to 3d space
agent_file = "agent"
object_ids = []
mapCells = {"#": "floor", "-": "width_wall", "|": "depth_wall", "1": "tl-corner", "2": "tr-corner", "3": "bl-corner", "4": "br-corner", "?": "unknown"}
skipCells = {"A": agent_file, "G": "goal", " ": "nothing"}

goal_id = ""
agent_id = ""





# load map
id_layout = copy.deepcopy(hidden) #start with hidden

print(hidden, "bruhhhhhhfhsdfhsd")


for z, layer in enumerate(id_layout):
    for y, row in enumerate(layer):
        for x, cell in enumerate(row):
            if cell in mapCells:
                piece = mapFilePath + mapCells.get(cell) + mapFileType
                id_layout[z][y][x] = p.loadURDF(piece, [x, y, z])
            if cell == "G":
                piece = mapFilePath + skipCells.get(cell) + mapFileType
                goal_id = p.loadURDF(piece, [x, y, z])
                id_layout[z][y][x] = " "
            if cell == "A":
                id_layout[z][y][x] = " "
                pass
piece = mapFilePath + skipCells.get("A") + mapFileType
agent_id = p.loadURDF(piece, [agent_x, agent_y, agent_z])
       
assert agent_id is not None, "Agent was not initialized!"
assert goal_id is not None, "Goal was not initialized!"

print("Controls: w = up, s = down, a = left, d = right, q = quit")

trajectory = []
action_map = {"w":0, "a":1, "s":2, "d":3}

print(hidden, 434343343)

while True:
    state = list(p.getBasePositionAndOrientation(agent_id)[0])
    rounded_state = list(map(round, state))

    hidden = check_explored_areas(hidden, 1, rounded_state, id_layout)
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
