import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import time
import pybullet_data

# === CONFIG ===
MOVE_STEP = 1
RADIUS = 1.6
MAP_FILE_PATH = "./map-pieces/"
MAP_FILE_TYPE = ".urdf"
MAX_STEPS = 250  # or even 150 while debugging
CELL_TYPE_MAP = {
    "floor": 0, "width_wall": 1, "depth_wall": 2,
    "tl-corner": 3, "tr-corner": 4, "bl-corner": 5, "br-corner": 6,
    "goal": 7, "unknown": 8, "nothing": 9, "agent": 10
}

# === HELPERS ===
def move_agent(dx, dy, dz, agent_id, map_size, map_grid):
    pos, orn = p.getBasePositionAndOrientation(agent_id)
    new_pos = [pos[0] + dx * MOVE_STEP, pos[1] + dy * MOVE_STEP, pos[2] + dz * MOVE_STEP]
    x, y, z = [round(i) for i in new_pos]

    if not (0 <= x < map_size[0] and 0 <= y < map_size[1] and 0 <= z < map_size[2]):
        return False  # Out of bounds

    try:
        target_cell = map_grid[z][y][x]
        if target_cell not in {"#", "G", " ", "A"}:
            return False  # Blocked by wall or corner
    except IndexError:
        return False

    p.resetBasePositionAndOrientation(agent_id, new_pos, orn)
    return True

def check_goal_collision(agent_id, goal_id):
    return bool(p.getContactPoints(bodyA=agent_id, bodyB=goal_id))

def check_explored_areas(map_objects, radius, agent_position):
    for entry in map_objects:
        if entry["visible"]:
            continue
        dist = np.linalg.norm(np.array(agent_position) - np.array(entry["pos"]))
        if dist <= radius:
            p.resetBasePositionAndOrientation(entry["visual_id"], [1000, 1000, 1000], [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(entry["real_id"], entry["pos"], [0, 0, 0, 1])
            entry["visual_id"] = entry["real_id"]
            entry["visible"] = True
    return map_objects

def generate_observations(map_objs, agent_pos, map_size):
    width, height, depth = map_size
    obs = np.zeros((width, height, depth, len(CELL_TYPE_MAP)))
    for entry in map_objs:
        x, y, z = [round(i) for i in entry["pos"]]
        if 0 <= x < width and 0 <= y < height and 0 <= z < depth:
            idx = CELL_TYPE_MAP[entry["true_type"] if entry["visible"] else "unknown"]
            obs[x][y][z][idx] = 1
    ax, ay, az = [round(i) for i in agent_pos]
    if 0 <= ax < width and 0 <= ay < height and 0 <= az < depth:
        obs[ax][ay][az][CELL_TYPE_MAP["agent"]] = 1
    return obs.flatten().astype(np.float32)

class GridExploreEnv(gym.Env):
    def __init__(self, map_file, gui=False):
        super().__init__()
        self.map_file = map_file
        self.gui = gui
        self.step_count = 0
        self.mapCells = {
            "#": "floor", "-": "width_wall", "|": "depth_wall",
            "1": "tl-corner", "2": "tr-corner", "3": "bl-corner", "4": "br-corner", "G": "goal"
        }
        self.skipCells = {"A": "agent", " ": "nothing"}
        self.action_map = {"w": (0, 1, 0), "s": (0, -1, 0), "a": (-1, 0, 0), "d": (1, 0, 0)}
        self.actions = list(self.action_map.keys())
        self._load_map()
        self.observation_space = spaces.Box(0, 1, shape=(np.prod(self.MAP_SIZE) * len(CELL_TYPE_MAP),), dtype=np.float32)
        self.action_space = spaces.Discrete(len(self.actions))

    def _load_map(self):
        if not p.getConnectionInfo()['isConnected']:
            p.connect(p.GUI if self.gui else p.DIRECT)
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        layout, self.agent_pos, self.map_objects = [], [0, 0, 0], []
        self.goal_id = self.agent_id = None

        with open(self.map_file) as f:
            layer = []
            for line in f:
                line = line.strip()
                if not line:
                    if layer:
                        layout.append(layer)
                        layer = []
                else:
                    layer.append(list(line))
            if layer: layout.append(layer)

        self.map_grid = layout
        self.MAP_SIZE = [max(len(row) for layer in layout for row in layer), max(len(layer) for layer in layout), len(layout)]

        for z, layer in enumerate(layout):
            for y, row in enumerate(layer):
                for x, cell in enumerate(row):
                    pos = [x, y, z]
                    if cell in self.skipCells and cell == "A":
                        self.agent_id = p.loadURDF(MAP_FILE_PATH + "agent" + MAP_FILE_TYPE, pos)
                    elif cell in self.mapCells:
                        vis_id = p.loadURDF(MAP_FILE_PATH + "unknown" + MAP_FILE_TYPE, pos)
                        real_id = p.loadURDF(MAP_FILE_PATH + self.mapCells[cell] + MAP_FILE_TYPE, [1000, 1000, 1000])
                        self.map_objects.append({"pos": pos, "true_type": self.mapCells[cell], "visual_id": vis_id, "real_id": real_id, "visible": False})
                        if cell == "G":
                            self.goal_id = real_id

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            np.random.seed(seed)
        p.resetSimulation()
        self._load_map()
        self.step_count = 0
        agent_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        self.map_objects = check_explored_areas(self.map_objects, RADIUS, agent_pos)
        return generate_observations(self.map_objects, agent_pos, self.MAP_SIZE), {}

    def step(self, action):
        dx, dy, dz = self.action_map[self.actions[action]]
        agent_prev_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        moved = move_agent(dx, dy, dz, self.agent_id, self.MAP_SIZE, self.map_grid)

        # for _ in range(2):
        #     p.stepSimulation()
        #     time.sleep(0.01)

        agent_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        self.map_objects = check_explored_areas(self.map_objects, RADIUS, agent_pos)
        obs = generate_observations(self.map_objects, agent_pos, self.MAP_SIZE)

        reached = check_goal_collision(self.agent_id, self.goal_id)
        if reached:
            reward = 10.0  # make reaching the goal very attractive
        else:
            reward = -0.1  # penalize every step clearly
        if not moved:
            reward -= 0.1

        self.step_count += 1
        terminated = reached
        truncated = self.step_count >= MAX_STEPS

        return obs, reward, terminated, truncated, {"agent_pos": agent_pos}

    def render(self, mode='human'):
        pass
