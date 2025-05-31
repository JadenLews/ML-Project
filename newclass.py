import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import time
import pybullet_data
from collections import deque

# === CONFIG ===
MOVE_STEP = 1
RADIUS = 1.8
MAP_FILE_PATH = "./map-pieces/"
MAP_FILE_TYPE = ".urdf"
MAX_STEPS = 250
MAPCELLS = {
            "#": "wall", "?": "unknown", " ": "nothing", "-": "width_wall", "|": "depth_wall",
            "1": "tl-corner", "2": "tr-corner", "3": "bl-corner", "4": "br-corner", "G": "goal"
        }
SKIPCELLS = {"A": "agent", " ": "nothing"}
ACTIONMAP = {"w": (0, -1, 0), "s": (0, 1, 0), "a": (-1, 0, 0), "d": (1, 0, 0)}
ACTIONS = list(ACTIONMAP.keys())
BARRIOR = ["#", "W"]
CHAR_TO_FLOAT = {
    "?": 0.0,
    " ": 0.1,
    "#": 0.2,
    "A": 0.3,
    "G": 0.4,
    "f": 0.5  # if you're using this in path visualization
}



class GridExploreEnv(gym.Env):
    def __init__(self, map_file, output=False):
        super().__init__()
        self.map_file = map_file
        self.gui = output
        self.step_count = 0
        self.round_reward = 0
        self.total_reward = 0
        self.round_areas_explored = 0
        self.terminated = False


        self._load_map()

        self.observation_space = spaces.Box(0, 1, shape=(np.prod(self.MAP_SIZE),), dtype=np.float32)
        self.action_space = spaces.Discrete(len(ACTIONS))

# === HELPERS ===

    def arr_to_str(self, arr):
        temp = ""
        for z in arr:
            for y in z:
                for x in y:
                    temp += x
                temp += "\n"
            temp += "\n"
        return temp


    def check_explored_areas(self):
        for z, panel in enumerate(self.visionmap):
            for y, row in enumerate(panel):
                for x, cell in enumerate(row):
                    cell_pos = [x, y, z]
                    dist = np.linalg.norm(np.array(self.agent_pos) - np.array(cell_pos))
                    if dist <= RADIUS:
                        if self.visionmap[z][y][x] == "?":
                            self.round_areas_explored += 1
                            self.visionmap[z][y][x] = self.truemap[z][y][x]

    def move_agent(self, dx, dy, dz):
        x, y, z = self.agent_pos
        nposx = x + dx
        nposy = y + dy
        nposz = z + dz

        if nposx >= 0 and nposx < self.MAP_SIZE[0]:
            if nposy >= 0 and nposy < self.MAP_SIZE[1]:
                if nposz >= 0 and nposz < self.MAP_SIZE[2]:
                    if self.truemap[nposz][nposy][nposx] not in BARRIOR:
                        self.visionmap[z][y][x] = self.truemap[z][y][x]
                        self.visionmap[nposz][nposy][nposx] = "A"
                        self.agent_pos = [nposx, nposy, nposz]
                        return

        #collision with something or out of bounds
        self.round_reward -= .04
        if self.gui:
            print("collide with", nposx, nposy, nposz)

    def check_goal_collision(self):
        return self.agent_pos == self.goal_pos

    def score_path_bfs(self, start ):
        truemap = self.truemap
        x_len, y_len, z_len = len(truemap[0][0]), len(truemap[0]), len(truemap)
        visited = np.full((z_len, y_len, x_len), False)
        
        queue = deque()
        queue.append((start, 0, []))  # (position, path length, path taken)

        while queue:
            (x, y, z), dist, path = queue.popleft()

            if not (0 <= x < x_len and 0 <= y < y_len and 0 <= z < z_len):
                continue
            if visited[z][y][x]:
                continue
            if truemap[z][y][x] == "#":
                continue

            visited[z][y][x] = True
            path = path + [[x, y, z]]

            if truemap[z][y][x] == "G":
                temp = self.truemap.copy()
                for item in path:
                    nx, ny, nz = item
                    temp[nz][ny][nx] = "f"
                
                temp[self.agent_pos[2]][self.agent_pos[1]][self.agent_pos[0]] = "A"
                temp[self.goal_pos[2]][self.goal_pos[1]][self.goal_pos[0]] = "G"
                return dist, path, temp

            # Explore neighbors
            neighbors = [
                (x + 1, y, z), (x - 1, y, z),
                (x, y + 1, z), (x, y - 1, z),
                (x, y, z + 1), (x, y, z - 1),
            ]
            for nx, ny, nz in neighbors:
                queue.append(((nx, ny, nz), dist + 1, path))

        return float('inf'), []  # Goal not found
        
    def _generate_observation(self):
        flat_obs = []
        for z in range(self.MAP_SIZE[2]):
            for y in range(self.MAP_SIZE[1]):
                for x in range(self.MAP_SIZE[0]):
                    char = self.visionmap[z][y][x]
                    flat_obs.append(CHAR_TO_FLOAT.get(char, 0.0))  # fallback to 0.0 for unknown
        return np.array(flat_obs, dtype=np.float32)
                        




    def _load_map(self):
        layout, self.agent_pos, self.map_objects = [], [0, 0, 0], []
        self.goal_pos = self.agent_pos = None

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

        self.MAP_SIZE = [max(len(row) for layer in layout for row in layer), max(len(layer) for layer in layout), len(layout)]
        self.truemap = np.full((self.MAP_SIZE[2], self.MAP_SIZE[1], self.MAP_SIZE[0]), " ", dtype=None)
        self.visionmap = np.full((self.MAP_SIZE[2], self.MAP_SIZE[1], self.MAP_SIZE[0]), "?", dtype=None)

        for z, layer in enumerate(layout):
            for y, row in enumerate(layer):
                for x, cell in enumerate(row):
                    pos = [x, y, z]
                    if cell == "A":
                        self.agent_pos = pos
                        self.truemap[z][y][x] = " "
                        self.visionmap[z][y][x] = "A"
                    elif cell in MAPCELLS.keys():
                        cell_pos = pos
                        cell_type = cell
                        cell_obj = {"pos": pos, "true_type": MAPCELLS[cell], "visual_type": MAPCELLS["?"], "visible": False}
                        self.truemap[z][y][x] = cell
                        self.map_objects.append(cell_obj)
                        if cell == "G":
                            self.goal_pos = pos
        
        self.check_explored_areas()
        if self.gui:
            dist, path, bpath = self.score_path_bfs(self.agent_pos)
            print(self.arr_to_str(self.visionmap), "initial vision map")
            print(self.arr_to_str(bpath), "initial best path")




    def reset(self, seed=None, options=None):
        super().reset(seed=seed)  # Optional but good to include for seeding

        self.step_count = 0
        self.round_reward = 0
        self.total_reward = 0
        self.agent_pos = [0, 0, 0]
        self.goal_pos = [0, 0, 0]
        self.truemap = []
        self.visionmap = []
        self.MAP_SIZE = [0, 0, 0]
        self.map_objects = []
        self.round_areas_explored = 0
        self.terminated = False



        self._load_map()

        # Generate initial observation (from visionmap or other method)
        obs = self._generate_observation()
        return obs, {}


    def step(self, action):
        self.step_count += 1
        dx, dy, dz = ACTIONMAP.get(ACTIONS[action])
        agent_prev_pos = self.agent_pos.copy()

        self.move_agent(dx, dy, dz)

        agent_post_pos = self.agent_pos


        dist, path, f_path = self.score_path_bfs(agent_prev_pos)

        moved = agent_post_pos != agent_prev_pos

        if agent_post_pos in path and moved and self.gui:
            print("Fastest choice")


        self.check_explored_areas()
        

        dist, path, f_path = self.score_path_bfs(agent_post_pos)


        # if self.gui:
        #     print(self.visionmap, "vision map")

        if self.gui:
            print(self.arr_to_str(f_path), "best path now")
        

        reward = 0.0

        if self.check_goal_collision():
            reward += 10
            self.terminated = True
        else:
            self.terminated = False

        if moved:
            # Encourage moving along shortest path
            if agent_post_pos in path:
                reward += 0.05
            else:
                reward -= 0.05
        else:
            reward -= 0.1  # hitting a wall

        # Bonus for exploration
        reward += 0.01 * self.round_areas_explored
        self.round_areas_explored = 0

        truncated = self.step_count >= MAX_STEPS
        obs = self._generate_observation()
        return obs, reward, self.terminated, truncated, {}



    def render(self, mode='human'):
        pass
