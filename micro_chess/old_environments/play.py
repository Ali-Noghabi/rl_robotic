from environment import MicroChessEnv
if __name__ == "__main__":
    env = MicroChessEnv()
    observation = env.reset()
    env.render()
    
    # Example move 1: White Knight from (4,2) to (2,1)
    # Encoding: from_pos=4*4 + 2 = 18, to_pos=2*4 +1 = 9
    action1 = 18 * 20 + 9  # from=18, to=9
    observation, reward, done, info = env.step(action1)
    print("After White Knight moves from (4,2) to (2,1):")
    env.render()
    print(f"Reward: {reward}, Done: {done}\n")
    
    # Example move 2: Black Knight from (0,1) to (2,0)
    # Encoding: from_pos=0*4 +1 =1, to_pos=2*4 +0 =8
    action2 = 1 * 20 + 8  # from=1, to=8
    observation, reward, done, info = env.step(action2)
    print("After Black Knight moves from (0,1) to (2,0):")
    env.render()
    print(f"Reward: {reward}, Done: {done}\n")
