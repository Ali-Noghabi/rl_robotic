from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from shut_the_box_environment import ShutTheBoxEnv
from masking_wrapper import ActionMaskWrapper
import torch

# Check if using GPU
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Training on: {device}")

# Create the environment with debug=False for training (no debug prints)
env = ShutTheBoxEnv(debug=False)

# Apply action masking wrapper (to single environment)
env = ActionMaskWrapper(env)

# Now wrap the masked environment in DummyVecEnv
env = DummyVecEnv([lambda: env])

# Hyperparameters to improve value function learning
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    device=device,
    learning_rate=0.0001,  # Lower learning rate for more stable updates
    vf_coef=0.5,  # Increase focus on value function
    gamma=0.95,  # Reduce discount factor to emphasize immediate rewards
)

# Train the agent for 1,000,000 steps
model.learn(total_timesteps=1000000)

# Save the trained model
model.save("ppo_shut_the_box")
