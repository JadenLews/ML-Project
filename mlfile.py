from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from mlclass import GridExploreEnv

# Create environment
env = GridExploreEnv("map.txt")
check_env(env)  # Optional: checks Gym compatibility

# Train model
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=200000)

# Test trained model
obs, _ = env.reset()
terminated = False
truncated = False

while not (terminated or truncated):
    action, _ = model.predict(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"Step: {env.step_count}, Action: {action}, Reward: {reward}, Terminated: {terminated}")
    # env.render()  # Only if you implemented rendering in your class