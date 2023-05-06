import pybullet as p
import os
import math

class Robot:
    def __init__(self,client):
        self.client = client
        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        f_name = os.path.join(os.path.dirname(__file__), 'panda_arm_hand.urdf')
        print(f_name)
        self.robot = p.loadURDF(f_name, startPos, startOrientation, useFixedBase = 1)
        self.numJoints = p.getNumJoints(self.robot, physicsClientId=self.client)
    def get_ids(self):
        return self.robot, self.client
    
    def apply_action(self, action = [0,0,0]):
        targetOrientation = p.getQuaternionFromEuler([0,0,0])
        targetPosJoints = p.calculateInverseKinematics(self.robot, self.numJoints, action, targetOrientation=targetOrientation)  
        p.setJointMotorControlArray(self.robot, range(self.numJoints), p.POSITION_CONTROL, targetPositions=targetPosJoints)
       # p.setJointMotorControl2(self.robot, 9, p.POSITION_CONTROL, grip1_targetPos)
       # p.setJointMotorControl2(self.robot, 10, p.POSITION_CONTROL, grip2_targetPos)

    def get_observation(self):
        return ("EMPTY CODE")