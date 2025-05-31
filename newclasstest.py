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


env = GridExploreEnv("map.txt", True)
obs, _ = env.reset()
done = False
while not done:
    action = env.action_space.sample()
    obs, reward, terminated, truncated, _ = env.step(action)
    print(f"Reward: {reward}, Done: {terminated or truncated}")
    if terminated or truncated:
        break







