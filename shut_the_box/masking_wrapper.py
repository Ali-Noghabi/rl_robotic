from gym import Env
import numpy as np

class ActionMaskWrapper(Env):
    def __init__(self, env):
        self.env = env
        self.observation_space = env.observation_space  # Delegate observation space
        self.action_space = env.action_space  # Delegate action space

    def step(self, action):
        return self.env.step(action)

    def reset(self):
        return self.env.reset()

    def action_mask(self):
        valid_actions = self.env.get_valid_actions(self.env.dice_sum)
        action_mask = np.zeros(512, dtype=np.float32)  # Change to float32 for compatibility
        for action in valid_actions:
            action_mask[action] = 1.0  # Mark valid actions as 1.0
        return action_mask

    def render(self, mode='human'):
        return self.env.render()

    def close(self):
        return self.env.close()
