from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from shut_the_box_environment import ShutTheBoxEnv

# Create the environment with render_mode='human'
env = ShutTheBoxEnv(render_mode='human')
env = DummyVecEnv([lambda: env])

# Load the trained agent
model = PPO.load("ppo_shut_the_box")

# Reset the environment
reset_return = env.reset()
# Handle whether reset returns one or two values
if isinstance(reset_return, tuple):
    obs, _ = reset_return  # Expect two values: observation and info
else:
    obs = reset_return  # Only one value returned
done = False
step_count = 0

# Run one episode
while not done:
    step_count += 1
    print(f"\nStep: {step_count}")
    print(f"Observation (Before action): {obs}")

    # Get action from the trained model
    action, _ = model.predict(obs)
    print(f"Action chosen by agent: {bin(action[0])[2:] }")

    # Take the action and get new state and reward
    obs, reward, done, info = env.step(action)
    
    # Render the current state and print details
    env.render()
    print(f"Reward: {reward}")
    print(f"Done: {done}")
    print(f"Invalid Actions So Far: {env.envs[0].invalid_actions_count}")

print(f"\nGame over! Final score: {reward}")
