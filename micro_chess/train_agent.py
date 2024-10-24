# train_agent.py

import ray
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
from micro_chess_env import MicroChessEnv
from models import register_custom_model
import os

def make_micro_chess_env(config):
    return MicroChessEnv()

def main():
    # Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # Register the environment
    register_env("MicroChessEnv-v0", make_micro_chess_env)
    
    # Register the custom model
    register_custom_model()
    
    # Define the PPO configuration
    config = {
        "env": "MicroChessEnv-v0",
        "model": {
            "custom_model": "micro_chess_model",
        },
        "num_gpus": 0,  # Set to 1 if GPU is available
        "num_workers": 1,  # Parallelism
        "framework": "torch",  # or "tf"
        "lr": 1e-4,
        "gamma": 0.99,
        "lambda": 1.0,
        "clip_param": 0.2,
        "entropy_coeff": 0.01,
        "train_batch_size": 1000,
        "sgd_minibatch_size": 100,
        "num_sgd_iter": 10,
        "env_config": {},  # Additional environment config if needed
    }
    
    # Initialize the PPO trainer
    trainer = PPOTrainer(config=config)
    
    # Training loop
    num_iterations = 1000  # Adjust as needed
    checkpoint_dir = "checkpoints"
    os.makedirs(checkpoint_dir, exist_ok=True)
    
    for i in range(num_iterations):
        result = trainer.train()
        
        if i % 10 == 0:
            print(f"Iteration {i}: episode_reward_mean={result['episode_reward_mean']}")
            checkpoint = trainer.save(checkpoint_dir)
            print(f"Checkpoint saved at {checkpoint}")
    
    # Save the final model
    final_checkpoint = trainer.save(checkpoint_dir)
    print(f"Training completed. Final checkpoint saved at {final_checkpoint}")
    
    # Shutdown Ray
    ray.shutdown()

if __name__ == "__main__":
    main()
