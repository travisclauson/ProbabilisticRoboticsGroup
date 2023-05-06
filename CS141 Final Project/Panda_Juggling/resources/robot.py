import pybullet as p
import os
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces

class Robot:
    def __init__(self,client):
        self.client = client
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        f_name = os.path.join(os.path.dirname(__file__), 'panda_arm_hand.urdf')
        self.robot = p.loadURDF(f_name, startPos, startOrientation, useFixedBase = 1)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3), dtype=np.float32)      # 3 for x,y,z -- not doing joint action space yet might be too dificult
    
    def get_ids(self):
        return self.robot, self.client
    
    def apply_action():
        pass

    def get_observation(self):
        pass