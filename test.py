import gym
import numpy as np
from mlclass import GridExploreEnv

env = GridExploreEnv("map.txt")

obs = env.reset()
print("Initial observation shape:", obs.shape)

done = False
total_reward = 0
step_count = 0

while not done and step_count < 10:
    action = env.action_space.sample()
    obs, reward, done, info = env.step(action)
    print(f"Step {step_count}: Action={action}, Reward={reward}, Done={done}")
    step_count += 1
    total_reward += reward

print("Episode finished. Total reward:", total_reward)