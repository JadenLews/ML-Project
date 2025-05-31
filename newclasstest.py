#from stable_baselines3 import PPO
#from stable_baselines3.common.env_checker import check_env
from newclass import GridExploreEnv

# # Create environment
# env = GridExploreEnv("map.txt", True)
# #check_env(env)

# while True:
#     cmd = input("Enter move: ").strip().lower()
#     env.step(cmd)

# # env.step("s")
# # env.step("a")
# # env.step("a")

# # env.step("w")
# # env.step("w")
# # env.step("w")
# # env.step("w")
# # env.step("w")


# # env.step("d")
# # env.step("d")
# # env.step("d")
# # env.step("d")
# # env.step("d")
# # env.step("d")
ACTIONMAP = {"w": (0, -1, 0), "s": (0, 1, 0), "a": (-1, 0, 0), "d": (1, 0, 0)}
ACTIONS = list(ACTIONMAP.keys())
MAP_FP = "./maps/"
MAPS = ["map.txt", "sarah.txt"]
env = GridExploreEnv(MAP_FP + "jaden.txt", True)
obs, _ = env.reset()
done = False
while not done:
    print("========= NEW STEP ===============")
    action = env.action_space.sample()
    print("action:", ACTIONS[action])
    obs, reward, terminated, truncated, _ = env.step(action)
    print(f"Reward: {reward}, Done: {terminated or truncated}")
    if terminated or truncated:
        break
print("========= DONE ===============")








