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

CELL_TYPE_MAP = {
    "floor": 0,
    "width_wall": 1,
    "depth_wall": 2,
    "tl-corner": 3,
    "tr-corner": 4,
    "bl-corner": 5,
    "br-corner": 6,
    "goal": 7,
    "unknown": 8,
    "nothing": 9,
    "agent": 10
}

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
    for indx, entry in enumerate(map_objects):
        pos = entry["pos"]
        typ = entry["true_type"]
        vis_id = entry["visual_id"]
        real_id = entry["real_id"]
        visible = entry["visible"]
        if visible != False:
            continue
        dist = np.linalg.norm(p1 - np.array(pos))
        if dist <= radius:
            p.resetBasePositionAndOrientation(vis_id, [1000, 1000, 1000], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(real_id, pos, [0, 0, 0, 1])
            map_objects[indx]["visual_id"] = real_id
            map_objects[indx]["visible"] = True

    return map_objects


def generate_observations(map_objs, agent_pos, map_size):
    width, height, depth = map_size
    obs = np.zeros((width, height, depth, len(CELL_TYPE_MAP)))

    for entry in map_objs:
        x, y, z = [round(i) for i in entry["pos"]]
        if entry["visible"]:
            type_idx = CELL_TYPE_MAP[entry["true_type"]]
        else:
            type_idx = CELL_TYPE_MAP["unknown"]
        obs[x][y][z][type_idx] = 1
    ax, ay, az = [round(i) for i in agent_pos]
    obs[ax][ay][az][CELL_TYPE_MAP["agent"]] = 1

    return obs.flatten()






# === LOAD MAP ===
layout = []
agent_pos = [0, 0, 0]
max_x = max_y = 0

with open(MAP_FILE) as f:
    layer = []
    for line in f:
        line = line.strip()
        if not line:#
            if layer:
                layout.append(layer)
                max_y = max(max_y, len(layer))
                max_x = max(max_x, max(len(row) for row in layer))
                layer = []
        else:
            layer.append(list(line))
    if layer:
        layout.append(layer)
        max_y = max(max_y, len(layer))
        max_x = max(max_x, max(len(row) for row in layer))

max_z = len(layout)

# === SET MAP SIZE ===
MAP_SIZE = [max_x, max_y, max_z]



#print(f"Map dimensions (x, y, z): ({max_x}, {max_y}, {max_z})")



# === OBJECT LOADING ===


# map_objects = [
#   {
#     "pos": [x, y, z],
#     "true_type": "#",         # what it actually is
#     "visual_id": id1,         # what the agent sees now (e.g., unknown block)
#     "real_id": id2,           # the actual map object to show when discovered
#     "visible": False          # has it been revealed?
#   },
#   ...
# ]



mapCells = {
    "#": "floor", "-": "width_wall", "|": "depth_wall", "1": "tl-corner", "2": "tr-corner",
    "3": "bl-corner", "4": "br-corner", "?": "unknown", "G": "goal"
}
skipCells = {"A": "agent", " ": "nothing"}

map_objects = []
agent_id = goal_id = None

max_height = 0
max_width = 0

for z, layer in enumerate(layout):
    for y, row in enumerate(layer):
        for x, cell in enumerate(row):
            pos = [x, y, z]
            if cell in skipCells:
                if cell == "A":
                    agent_id = p.loadURDF(MAP_FILE_PATH + "agent" + MAP_FILE_TYPE, pos)
            elif cell in mapCells:
                vis_id = p.loadURDF(MAP_FILE_PATH + "unknown" + MAP_FILE_TYPE, pos)
                real_id = p.loadURDF(MAP_FILE_PATH + mapCells.get(cell) + MAP_FILE_TYPE, [1000,1000,1000])
                entry = {
                    "pos": pos,
                    "true_type": mapCells.get(cell),
                    "visual_id": vis_id,
                    "real_id": real_id,
                    "visible": False
                }
                map_objects.append(entry)
                if cell == "G":
                    goal_id = real_id
                    goal_pos = pos 





assert agent_id is not None, "Agent not loaded!"
print("Controls: w = up, s = down, a = left, d = right, q = quit")







# === MAIN LOOP ===
action_map = {"w": (0, 1, 0), "s": (0, -1, 0), "a": (-1, 0, 0), "d": (1, 0, 0)}
trajectory = []
done = False

# Initial reveal
state = list(p.getBasePositionAndOrientation(agent_id)[0])
map_objects = check_explored_areas(map_objects, RADIUS, state)
for _ in range(5):
    p.stepSimulation()
    time.sleep(0.01)

while not done:
    begin_obs = generate_observations(map_objects, p.getBasePositionAndOrientation(agent_id)[0], MAP_SIZE)

    cmd = input("Enter move: ").strip().lower()
    if cmd == "q":
        print("Exiting.")
        break

    if cmd in action_map:
        dx, dy, dz = action_map[cmd]
        move_agent(dx, dy, dz, agent_id)
    else:
        print("Invalid command.")
        continue

    state = list(p.getBasePositionAndOrientation(agent_id)[0])
    map_objects = check_explored_areas(map_objects, RADIUS, state)
    p.stepSimulation()
    time.sleep(0.05)

    next_obs = generate_observations(map_objects, state, MAP_SIZE)

    # Goal check
    done = goal_id and check_goal_collision(agent_id, goal_id)
    reward = 1 if done else 0

    trajectory.append((begin_obs, cmd, reward, next_obs, done))

    if done:
        print("Goal reached!")
        break

    time.sleep(1 / 240)

with open("manual_trajectory.pkl", "wb") as f:
    pickle.dump(trajectory, f)

p.disconnect()