# human_vs_ai.py

import gym
from micro_chess_env import MicroChessEnv
from action_encoding import ACTION_TO_INT, INT_TO_ACTION
import torch
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.registry import register_env
from models import register_custom_model

def make_micro_chess_env(config):
    return MicroChessEnv()

def load_trained_agent(checkpoint_path):
    # Initialize Ray
    import ray
    ray.init(ignore_reinit_error=True)
    
    # Register the environment and model
    register_env("MicroChessEnv-v0", make_micro_chess_env)
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
    }
    
    # Initialize the PPO trainer
    trainer = PPOTrainer(config=config)
    
    # Restore from checkpoint
    trainer.restore(checkpoint_path)
    
    return trainer

def human_vs_ai(env, trainer):
    """
    Allows a human to play against the AI.
    """
    obs = env.reset()
    env.render()
    done = False
    while not done:
        # Human move
        print("Your move:")
        move = input("Enter move as from_row from_col to_row to_col (e.g., 0 0 1 0): ")
        try:
            from_r, from_c, to_r, to_c = map(int, move.strip().split())
            action = ACTION_TO_INT.get((from_r, from_c, to_r, to_c), None)
            if action is None:
                print("Invalid move encoding. Please enter four integers between 0 and 4 for rows and 0 and 3 for columns.")
                continue
            # Ensure the move is legal
            piece = env.board[from_r, from_c]
            if piece * env.current_turn <= 0:
                print("You can only move your own pieces.")
                continue
            if not env._is_valid_move(piece, from_r, from_c, to_r, to_c):
                print("Invalid move based on piece movement rules.")
                continue
        except ValueError:
            print("Invalid input format. Please enter four integers separated by spaces.")
            continue

        # Execute human move
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            if reward > 0:
                print("You win!")
            elif reward < 0:
                print("You lose!")
            else:
                print("Game ended.")
            break

        # AI move
        ai_action = trainer.compute_action(obs)
        print(f"AI selects action: {INT_TO_ACTION[ai_action]}")
        obs, reward, done, info = env.step(ai_action)
        env.render()
        if done:
            if reward < 0:
                print("AI wins!")
            elif reward > 0:
                print("You lose!")
            else:
                print("Game ended.")
            break

def main():
    # Path to the trained agent's checkpoint
    checkpoint_path = "checkpoints/micro_chess/checkpoint_XXX"  # Replace XXX with actual checkpoint number

    # Load the trained agent
    trainer = load_trained_agent(checkpoint_path)
    
    # Initialize the environment
    env = MicroChessEnv()
    
    # Start the human vs AI game
    human_vs_ai(env, trainer)
    
    # Shutdown Ray
    import ray
    ray.shutdown()

if __name__ == "__main__":
    main()
