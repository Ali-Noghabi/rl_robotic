import gymnasium as gym

class SeedCompatibleWrapper(gym.Wrapper):
    def reset(self, seed=None, options=None):
        # Ensure the seed argument is passed and handled properly
        return self.env.reset(seed=seed, options=options)
