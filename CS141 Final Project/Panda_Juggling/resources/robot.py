import pybullet as p
import os
import math


class Robot:
    def __init__(self,client):
        self.client = client
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        f_name = os.path.join(os.path.dirname(__file__), 'panda_arm_hand.urdf')
        self.robot = p.loadURDF(fileName=f_name, startPos, startOrientation, useFixedBase = 1)
    
    def get_ids(self):
        return self.robot, self.client
    
    def apply_action():
        pass

    def get_observation(self):
        pass