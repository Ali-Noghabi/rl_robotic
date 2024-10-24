# models.py

from ray.rllib.models.torch.torch_modelv2 import TorchModelV2
from ray.rllib.models import ModelCatalog
import torch
import torch.nn as nn
import torch.nn.functional as F

class MicroChessModel(TorchModelV2, nn.Module):
    """
    Custom neural network model for Micro Chess.
    """
    def __init__(self, obs_space, action_space, num_outputs, model_config, name):
        TorchModelV2.__init__(self, obs_space, action_space, num_outputs, model_config, name)
        nn.Module.__init__(self)
        
        # Convolutional layers
        self.conv1 = nn.Conv2d(1, 32, kernel_size=3, stride=1, padding=1)  # Output: 32x5x4
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=1)  # Output: 64x5x4
        self.bn2 = nn.BatchNorm2d(64)
        
        # Fully connected layers
        self.flatten = nn.Flatten()
        self.fc1 = nn.Linear(64 * 5 * 4, 128)
        self.fc2 = nn.Linear(128, 64)
        
        # Policy and Value heads
        self.policy_head = nn.Linear(64, num_outputs)
        self.value_head = nn.Linear(64, 1)
        
    def forward(self, input_dict, state, seq_lens):
        """
        Forward pass for the network.
        """
        x = input_dict["obs"].unsqueeze(1).float()  # Add channel dimension
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = self.flatten(x)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        self._features = x
        return self.policy_head(x), state

    def value_function(self):
        """
        Computes the value function.
        """
        return self.value_head(self._features).squeeze(1)

# Register the custom model
def register_custom_model():
    ModelCatalog.register_custom_model("micro_chess_model", MicroChessModel)
