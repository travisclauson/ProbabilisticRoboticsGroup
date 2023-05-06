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
        p.setJointMotorControlArray(self.robot, range(self.numJoints), p.POSITION_CONTROL, targetPositions=targetPosJoints)
       # p.setJointMotorControl2(self.robot, 9, p.POSITION_CONTROL, grip1_targetPos)
       # p.setJointMotorControl2(self.robot, 10, p.POSITION_CONTROL, grip2_targetPos)

    def get_observation(self):
        # return ("EMPTY CODE")
        return p.getBasePositionAndOrientation(self.robot, self.client)