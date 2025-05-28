import pybullet as p
import time
import pybullet_data
import pickle
import numpy as np

# === CONFIG ===
MOVE_STEP = 1
RADIUS = 1.6
MAP_FILE = "map.txt"
MAP_FILE_PATH = "./map-pieces/"
MAP_FILE_TYPE = ".urdf"

# === PYBULLET INIT ===
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# === HELPERS ===
def move_agent(dx, dy, dz, agent_id):
    pos, orn = p.getBasePositionAndOrientation(agent_id)
    new_pos = [pos[0] + dx * MOVE_STEP, pos[1] + dy * MOVE_STEP, pos[2] + dz * MOVE_STEP]
    p.resetBasePositionAndOrientation(agent_id, new_pos, orn)

def check_goal_collision(agent_id, goal_id):
    contacts = p.getContactPoints(bodyA=agent_id, bodyB=goal_id)
    return contacts and len(contacts) > .1

def check_explored_areas(map_objects, radius, agent_position):
    p1 = np.array(agent_position)
    for entry in map_objects:
        body_id, char, visible = entry
        if visible != "?":
            continue
        elem_pos = p.getBasePositionAndOrientation(body_id)[0]
        dist = np.linalg.norm(p1 - np.array(elem_pos))
        if dist <= radius:
            p.removeBody(body_id)
            new_id = p.loadURDF(MAP_FILE_PATH + mapCells[char] + MAP_FILE_TYPE, elem_pos)
            entry[0] = new_id
            entry[2] = char
    return map_objects

def check_goal_loaded(map):
    for entry in map:
        body_id, char, visible = entry
        if char == "G":
            if visible != "?":
                return body_id
            else:
                return None

# === LOAD MAP ===
layout = []
agent_pos = [0, 0, 0]
with open(MAP_FILE) as f:
    layer, tempz, tempy = [], 0, 0
    for line in f:
        line = list(line.strip())
        if 'A' in line:
            agent_pos = [line.index('A'), tempy, tempz]
        if not line:
            layout.append(layer)
            layer, tempy, tempz = [], 0, tempz + 1
        else:
            layer.append(line)
            tempy += 1
    if layer: layout.append(layer)

# === OBJECT LOADING ===
mapCells = {
    "#": "floor", "-": "width_wall", "|": "depth_wall", "1": "tl-corner", "2": "tr-corner",
    "3": "bl-corner", "4": "br-corner", "?": "unknown", "G": "goal"
}
skipCells = {"A": "agent", " ": "nothing"}

map_objects = []
agent_id = goal_id = None

for z, layer in enumerate(layout):
    for y, row in enumerate(layer):
        for x, cell in enumerate(row):
            pos = [x, y, z]
            if cell in skipCells:
                if cell == "A":
                    agent_id = p.loadURDF(MAP_FILE_PATH + "agent" + MAP_FILE_TYPE, pos)
            elif cell in mapCells:
                body_id = p.loadURDF(MAP_FILE_PATH + "unknown" + MAP_FILE_TYPE, pos)
                map_objects.append([body_id, cell, "?"])
                if cell == "G":
                    goal_pos = pos  # Store goal pos to confirm later

assert agent_id is not None, "Agent not loaded!"
print("Controls: w = up, s = down, a = left, d = right, q = quit")

# === MAIN LOOP ===
action_map = {"w": (0, 1, 0), "s": (0, -1, 0), "a": (-1, 0, 0), "d": (1, 0, 0)}
trajectory = []

while True:
    state = list(p.getBasePositionAndOrientation(agent_id)[0])
    grid_state = [round(s) for s in state]
    map_objects = check_explored_areas(map_objects, RADIUS, state)

    cmd = input("Enter move: ").strip().lower()
    if cmd == "q":
        print("Exiting.")
        break
    if cmd in action_map:
        dx, dy, dz = action_map[cmd]
        move_agent(dx, dy, dz, agent_id)
        trajectory.append((grid_state, cmd))
    else:
        print("Invalid command.")

    if goal_id == None:
        goal_id = check_goal_loaded(map_objects)

    p.stepSimulation()
    if goal_id and check_goal_collision(agent_id, goal_id):
        print("Goal reached!")
        with open("manual_trajectory.pkl", "wb") as f:
            pickle.dump(trajectory, f)
        break

    time.sleep(1 / 240)

p.disconnect()