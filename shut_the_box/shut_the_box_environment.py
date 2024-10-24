import gym
import torch
import numpy as np
from gym import spaces
from itertools import combinations

class ShutTheBoxEnv(gym.Env):
    def __init__(self, render_mode=None, debug=False):
        super(ShutTheBoxEnv, self).__init__()

        self.render_mode = render_mode
        self.debug = debug  # Add debug flag to control debug prints
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Tiles are from 1 to 9, and can either be open (1) or shut (0)
        self.tiles = torch.ones(9, device=self.device)
        self.dice_sum = torch.tensor(0, device=self.device)  # Initialize dice_sum

        # Observation space: 9 for tiles + 1 for dice_sum, use np.float32
        self.observation_space = spaces.Box(low=0, high=1, shape=(10,), dtype=np.float32)

        # Action space: 512 possible actions (2^9 combinations of tiles to shut)
        self.action_space = spaces.Discrete(512)

        # To track invalid actions during training
        self.invalid_actions_count = 0

        # Precompute valid actions for all dice sums and all tile states
        self.valid_action_map = self.precompute_valid_actions()

    def precompute_valid_actions(self):
        valid_action_map = {dice_sum: {} for dice_sum in range(2, 13)}

        # For every possible tile state (there are 2^9 = 512 combinations)
        for tile_state in range(512):
            open_tiles = [i + 1 for i in range(9) if tile_state & (1 << i)]

            for dice_sum in range(2, 13):
                valid_actions = []
                for r in range(1, len(open_tiles) + 1):
                    for combo in combinations(open_tiles, r):
                        if sum(combo) == dice_sum:
                            action = sum(1 << (i - 1) for i in combo)
                            valid_actions.append(action)

                valid_action_map[dice_sum][tile_state] = valid_actions

        return valid_action_map

    def reset(self, seed=None, **kwargs):
        if seed is not None:
            self._np_random, seed = gym.utils.seeding.np_random(seed)
            np.random.seed(seed)
            torch.manual_seed(seed)

        self.tiles = torch.ones(9, device=self.device)
        self.dice_sum = self.roll_dice()
        self.invalid_actions_count = 0  # Reset invalid action counter

        obs = self._get_obs()
        return obs, {}

    def roll_dice(self):
        return np.random.randint(1, 7) + np.random.randint(1, 7)

    def get_valid_actions(self, dice_sum):
        tile_state = self.get_tile_state()
        return self.valid_action_map[dice_sum][tile_state]

    def get_tile_state(self):
        return int(''.join(str(int(x)) for x in self.tiles.tolist()[::-1]), 2)

    def step(self, action):
        valid_actions = self.get_valid_actions(self.dice_sum)

        if action not in valid_actions:
            self.invalid_actions_count += 1
            return self._get_obs(), -10, True, {}

        selected_tiles = [i + 1 for i in range(9) if action & (1 << i)]

        for tile in selected_tiles:
            self.tiles[tile - 1] = 0

        self.dice_sum = self.roll_dice()

        valid_moves = self.get_valid_actions(self.dice_sum)
        if not valid_moves:
            return self._get_obs(), -sum(self.tiles).item(), True, {}

        return self._get_obs(), 1, False, {}

    def _get_obs(self):
        tiles_array = self.tiles.cpu().numpy().flatten().astype(np.float32)
        dice_sum_array = np.array([float(self.dice_sum)], dtype=np.float32)
        obs = np.concatenate([tiles_array, dice_sum_array], axis=0)
        print(f"Observation shape: {obs.shape}, Observation: {obs}")  # Debugging print
        return obs

    def render(self):
        if self.render_mode == 'human':
            print(f"Tiles: {self.tiles.cpu().numpy()}, Dice Sum: {self.dice_sum}")

    def action_mask(self):
        valid_actions = self.get_valid_actions(self.dice_sum)
        action_mask = np.zeros(512, dtype=np.bool)
        for action in valid_actions:
            action_mask[action] = True
        return action_mask
