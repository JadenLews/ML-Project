import gym
from gym import spaces
import numpy as np
import pybullet as p
import time
import pybullet_data

# === CONFIG ===
MOVE_STEP = 1
RADIUS = 1.6
MAP_FILE_PATH = "./map-pieces/"
MAP_FILE_TYPE = ".urdf"
CELL_TYPE_MAP = {
    "floor": 0, "width_wall": 1, "depth_wall": 2,
    "tl-corner": 3, "tr-corner": 4, "bl-corner": 5, "br-corner": 6,
    "goal": 7, "unknown": 8, "nothing": 9, "agent": 10
}

# === HELPERS ===
def move_agent(dx, dy, dz, agent_id, map_size):
    pos, orn = p.getBasePositionAndOrientation(agent_id)
    new_pos = [pos[0] + dx * MOVE_STEP, pos[1] + dy * MOVE_STEP, pos[2] + dz * MOVE_STEP]
    x, y, z = [round(i) for i in new_pos]

    if not (0 <= x < map_size[0] and 0 <= y < map_size[1] and 0 <= z < map_size[2]):
        return False  # Invalid move, don't update position

    p.resetBasePositionAndOrientation(agent_id, new_pos, orn)
    return True  # Valid move

def check_goal_collision(agent_id, goal_id):
    contacts = p.getContactPoints(bodyA=agent_id, bodyB=goal_id)
    return bool(contacts)

def check_explored_areas(map_objects, radius, agent_position):
    p1 = np.array(agent_position)
    for indx, entry in enumerate(map_objects):
        pos = entry["pos"]
        if entry["visible"]:
            continue
        dist = np.linalg.norm(p1 - np.array(pos))
        if dist <= radius:
            p.resetBasePositionAndOrientation(entry["visual_id"], [1000, 1000, 1000], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(entry["real_id"], pos, [0, 0, 0, 1])
            entry["visual_id"] = entry["real_id"]
            entry["visible"] = True
    return map_objects

def generate_observations(map_objs, agent_pos, map_size):
    width, height, depth = map_size
    obs = np.zeros((width, height, depth, len(CELL_TYPE_MAP)), dtype=np.float32)
    for entry in map_objs:
        x, y, z = [round(i) for i in entry["pos"]]
        idx = CELL_TYPE_MAP[entry["true_type"] if entry["visible"] else "unknown"]
        obs[x][y][z][idx] = 1
    ax, ay, az = [round(i) for i in agent_pos]
    obs[ax][ay][az][CELL_TYPE_MAP["agent"]] = 1
    return obs


class GridExploreEnv(gym.Env):
    def __init__(self, map_file, gui=True):
        super().__init__()
        self.map_file = map_file
        self.gui = gui

        self.mapCells = {
            "#": "floor", "-": "width_wall", "|": "depth_wall",
            "1": "tl-corner", "2": "tr-corner", "3": "bl-corner", "4": "br-corner", "G": "goal"
        }
        self.skipCells = {"A": "agent", " ": "nothing"}
        self.action_map = {"w": (0, 1, 0), "s": (0, -1, 0), "a": (-1, 0, 0), "d": (1, 0, 0)}
        self.actions = list(self.action_map.keys())

        self._load_map()
        obs_shape = (self.MAP_SIZE[0], self.MAP_SIZE[1], self.MAP_SIZE[2], len(CELL_TYPE_MAP))
        self.observation_space = spaces.Box(0, 1, shape=obs_shape, dtype=np.float32)
        self.action_space = spaces.Discrete(len(self.actions))

    def _load_map(self):
        if p.getConnectionInfo()['isConnected'] == 0:
            p.connect(p.GUI if self.gui else p.DIRECT)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        layout = []
        self.agent_pos = [0, 0, 0]
        max_x = max_y = 0

        with open(self.map_file) as f:
            layer = []
            for line in f:
                line = line.strip()
                if not line:
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
        self.MAP_SIZE = [max_x, max_y, max_z]

        self.map_objects = []
        self.goal_id = self.agent_id = None

        for z, layer in enumerate(layout):
            for y, row in enumerate(layer):
                for x, cell in enumerate(row):
                    pos = [x, y, z]
                    if cell in self.skipCells:
                        if cell == "A":
                            self.agent_id = p.loadURDF(MAP_FILE_PATH + "agent" + MAP_FILE_TYPE, pos)
                    elif cell in self.mapCells:
                        vis_id = p.loadURDF(MAP_FILE_PATH + "unknown" + MAP_FILE_TYPE, pos)
                        real_id = p.loadURDF(MAP_FILE_PATH + self.mapCells[cell] + MAP_FILE_TYPE, [1000, 1000, 1000])
                        entry = {
                            "pos": pos,
                            "true_type": self.mapCells[cell],
                            "visual_id": vis_id,
                            "real_id": real_id,
                            "visible": False
                        }
                        self.map_objects.append(entry)
                        if cell == "G":
                            self.goal_id = real_id

    def reset(self):
        self._load_map()
        for _ in range(5):
            p.stepSimulation()
            time.sleep(0.01)
        self.done = False
        self.trajectory = []
        agent_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        self.map_objects = check_explored_areas(self.map_objects, RADIUS, agent_pos)
        return generate_observations(self.map_objects, agent_pos, self.MAP_SIZE)

    def step(self, action):
        dx, dy, dz = self.action_map[self.actions[action]]
        valid_move = move_agent(dx, dy, dz, self.agent_id, self.MAP_SIZE)
        if not valid_move:
            obs = generate_observations(self.map_objects, p.getBasePositionAndOrientation(self.agent_id)[0], self.MAP_SIZE)
            return obs, -0.1, False, {}  # Penalty for invalid move

        for _ in range(2):
            p.stepSimulation()
            time.sleep(0.01)

        agent_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        self.map_objects = check_explored_areas(self.map_objects, RADIUS, agent_pos)
        obs = generate_observations(self.map_objects, agent_pos, self.MAP_SIZE)

        reached = check_goal_collision(self.agent_id, self.goal_id)
        reward = 1.0 if reached else -0.01
        done = reached

        return obs, reward, done, {"agent_pos": agent_pos}

    def render(self, mode='human'):
        pass