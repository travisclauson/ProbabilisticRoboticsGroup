import torch
from torch import nn
import torch.nn.functional as F
import numpy as np

# this is our neural network module used to define our actor & critic:

# basic feed forward nearl network
class FeedForwardNN(nn.Module):
    def __init__(self):
        super(FeedForwardNN, self).__init__()
    

    # define NNN layers, parameters are input/output dimensions
    def __init__(self, in_dim, out_dim):
        super(FeedForwardNN, self).__init__()
        # if isinstance(in_dim, np.ndarray):
        #     in_dim = torch.tensor(in_dim, dtype=torch.float)    
        # if isinstance(out_dim, np.ndarray):
        #     out_dim = torch.tensor(out_dim, dtype=torch.float)
        self.layer1 = nn.Linear(in_dim, 64)
        self.layer2 = nn.Linear(64, 64)
        self.layer3 = nn.Linear(64, out_dim)

    # takes parameter of observation (needs to be processed as a tensor) and returns either an action or a value
    def forward(self, obs):
        # Convert observation to tensor if it's a numpy array
        if isinstance(obs, np.ndarray):
            obs = torch.tensor(obs, dtype=torch.float)
        
        # rectified linear unit (ReLU) is an activation function that introduces the property of non-linearity to a deep learning model and solves the vanishing gradients issue
        activation1 = F.relu(self.layer1(obs))
        activation2 = F.relu(self.layer2(activation1))
        output = self.layer3(activation2)
        return output