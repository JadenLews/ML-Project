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






# def generate_observations(map_objs, agent_pos, map_size):
#     width, height, depth = map_size
#     obs = np.zeros((width, height, depth, len(CELL_TYPE_MAP)))
#     for entry in map_objs:
#         x, y, z = [round(i) for i in entry["pos"]]
#         if 0 <= x < width and 0 <= y < height and 0 <= z < depth:
#             idx = CELL_TYPE_MAP[entry["true_type"] if entry["visible"] else "unknown"]
#             obs[x][y][z][idx] = 1
#     ax, ay, az = [round(i) for i in agent_pos]
#     if 0 <= ax < width and 0 <= ay < height and 0 <= az < depth:
#         obs[ax][ay][az][CELL_TYPE_MAP["agent"]] = 1
#     return obs.flatten().astype(np.float32)

class GridExploreEnv(gym.Env):
    def __init__(self, map_file, output=False):
        super().__init__()
        self.map_file = map_file
        self.gui = output
        self.step_count = 0
        self.round_reward = 0
        self.total_reward = 0


        self._load_map()

        #self.observe_space = spaces.Box(0, 1, shape=(np.prod(self.MAP_SIZE) * len(CELL_TYPE_MAP),), dtype=np.float32)
        self.action_space = spaces.Discrete(len(ACTIONS))

# === HELPERS ===

    def check_explored_areas(self):
        for z, panel in enumerate(self.visionmap):
            for y, row in enumerate(panel):
                for x, cell in enumerate(row):
                    cell_pos = [x, y, z]
                    dist = np.linalg.norm(np.array(self.agent_pos) - np.array(cell_pos))
                    if dist <= RADIUS:
                        if self.visionmap[z][y][x] == "?":
                            self.visionmap[z][y][x] = self.truemap[z][y][x]

    def move_agent(self, dx, dy, dz):
        x, y, z = self.agent_pos
        nposx = x + dx
        nposy = y + dy
        nposz = z + dz

        if nposx >= 0 and nposx < self.MAP_SIZE[0]:
            if nposy >= 0 and nposy < self.MAP_SIZE[1]:
                if nposz >= 0 and nposz < self.MAP_SIZE[2]:
                    if self.gui:
                        print("good location")
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
            print(self.visionmap, "initial vision map")
            print(bpath, "initial best path")




    def reset(self, seed=None, options=None):
        self.step_count = 0
        self.round_reward = 0
        self.total_reward = 0
        self.agent_pos = [0, 0, 0]
        self.goal_pos = [0, 0, 0]
        self.truemap = []
        self.visionmap = []
        self.MAP_SIZE = [0, 0, 0]
        self.map_objects = []
        
        self._load_map()

        pass
        # super().reset(seed=seed)
        # if seed is not None:
        #     np.random.seed(seed)
        # p.resetSimulation()
        # self._load_map()
        # self.step_count = 0
        # agent_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        # self.map_objects = check_explored_areas(self.map_objects, RADIUS, agent_pos)
        # return generate_observations(self.map_objects, agent_pos, self.MAP_SIZE), {}

    def step(self, action):
        self.step_count += 1
        dx, dy, dz = ACTIONMAP.get(action)
        agent_prev_pos = self.agent_pos.copy()

        self.move_agent(dx, dy, dz)

        agent_post_pos = self.agent_pos


        dist, path, f_path = self.score_path_bfs(agent_prev_pos)

        moved = agent_post_pos != agent_prev_pos

        if agent_post_pos in path and moved and self.gui:
            print("Fastest choice")


        self.check_explored_areas()
        if self.gui:
            print(self.visionmap)
        

        moved = False
        dist, path, f_path = self.score_path_bfs(agent_post_pos)


        if self.gui:
            print(self.visionmap, "vision map")

        if self.gui:
            print(f_path, "best path now")
        





        # dx, dy, dz = self.action_map[self.actions[action]]
        # agent_prev_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        # moved = move_agent(dx, dy, dz, self.agent_id, self.MAP_SIZE, self.map_grid)

        # # for _ in range(2):
        # #     p.stepSimulation()
        # #     time.sleep(0.01)

        # agent_pos = p.getBasePositionAndOrientation(self.agent_id)[0]
        # self.map_objects = check_explored_areas(self.map_objects, RADIUS, agent_pos)
        # obs = generate_observations(self.map_objects, agent_pos, self.MAP_SIZE)

        # reached = check_goal_collision(self.agent_id, self.goal_id)
        # if reached:
        #     reward = 10.0  # make reaching the goal very attractive
        # else:
        #     reward = -0.1  # penalize every step clearly
        # if not moved:
        #     reward -= 0.1

        # self.step_count += 1
        # terminated = reached
        # truncated = self.step_count >= MAX_STEPS

        # return obs, reward, terminated, truncated, {"agent_pos": agent_pos}

    def render(self, mode='human'):
        pass
