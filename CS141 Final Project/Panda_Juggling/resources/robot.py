import pybullet as p
import os
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from contextlib import contextmanager
import os
import sys

class Robot:
    def __init__(self,client):
        self.client = client
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        f_name = os.path.join(os.path.dirname(__file__), 'franka_description/robots/panda_arm_hand.urdf')
        # with suppress_stdout():
        #     self.robot = p.loadURDF(f_name, startPos, startOrientation, useFixedBase = 1)
        self.robot = p.loadURDF(f_name, startPos, startOrientation, useFixedBase = 1)
        # self.numJoints = p.getNumJoints(self.robot, physicsClientId=self.client)
        self.numJoints = 9 #number of joints to end effector
        p.changeDynamics(self.robot, 9, restitution=.8)

        targetOrientation = p.getQuaternionFromEuler([0,0,0])
        targetPosJoints = p.calculateInverseKinematics(self.robot, self.numJoints, [0.2, 0.2, .5], targetOrientation=targetOrientation)  
        p.setJointMotorControlArray(self.robot, range(7), p.POSITION_CONTROL, targetPositions=targetPosJoints)
    def get_ids(self):
        return self.robot, self.client
    
    def apply_action(self, action):
        targetOrientation = p.getQuaternionFromEuler([action[2], action[3], action[4]])
        targetPosJoints = p.calculateInverseKinematics(self.robot, self.numJoints, [action[0], action[1], .5], targetOrientation=targetOrientation)  
        p.setJointMotorControlArray(self.robot, range(7), p.POSITION_CONTROL, targetPositions=targetPosJoints)

    def get_observation(self):
        return p.getLinkState(self.robot,9,0,1)[0]
    

# @contextmanager
# def suppress_stdout():
#     fd = sys.stdout.fileno()

#     def _redirect_stdout(to):
#         sys.stdout.close()  # + implicit flush()
#         os.dup2(to.fileno(), fd)  # fd writes to 'to' file
#         sys.stdout = os.fdopen(fd, "w")  # Python writes to fd

#     with os.fdopen(os.dup(fd), "w") as old_stdout:
#         with open(os.devnull, "w") as file:
#             _redirect_stdout(to=file)
#         try:
#             yield  # allow code to be run with the redirected stdout
#         finally:
#             _redirect_stdout(to=old_stdout)  # restore stdout.
#             # buffering and flags such as
#             # CLOEXEC may be different

    