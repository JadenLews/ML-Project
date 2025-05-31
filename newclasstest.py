#from stable_baselines3 import PPO
#from stable_baselines3.common.env_checker import check_env
from newclass import GridExploreEnv

# Create environment
env = GridExploreEnv("map.txt", True)
#check_env(env)

while True:
    cmd = input("Enter move: ").strip().lower()
    env.step(cmd)

# env.step("s")
# env.step("a")
# env.step("a")

# env.step("w")
# env.step("w")
# env.step("w")
# env.step("w")
# env.step("w")


# env.step("d")
# env.step("d")
# env.step("d")
# env.step("d")
# env.step("d")
# env.step("d")








