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
        f_name = os.path.join(os.path.dirname(__file__), 'franka_description/robots/panda_arm_hand.urdf')
        self.robot = p.loadURDF(f_name, startPos, startOrientation, useFixedBase = 1)
        # self.numJoints = p.getNumJoints(self.robot, physicsClientId=self.client)
        self.numJoints = 9 #number of joints to end effector
    def get_ids(self):
        return self.robot, self.client
    
    def apply_action(self, action):
        targetOrientation = p.getQuaternionFromEuler([0,0,0])
        targetPosJoints = p.calculateInverseKinematics(self.robot, self.numJoints, action)  
        p.setJointMotorControlArray(self.robot, range(7), p.POSITION_CONTROL, targetPositions=targetPosJoints)

    def get_observation(self):
        return p.getLinkState(self.robot,9,0,1)[0]