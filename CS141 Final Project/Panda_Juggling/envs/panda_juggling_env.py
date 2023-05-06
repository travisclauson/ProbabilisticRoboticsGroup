import gym
import numpy as np
import math
import pybullet as p
from Panda_Juggling.resources.ball import Ball
from Panda_Juggling.resources.plane import Plane
from Panda_Juggling.resources.robot import Robot
import matplotlib.pyplot as plt
# from gymnasium import spaces

class PandaJugglingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.client = p.connect(p.GUI)
        self.np_random, _ = gym.utils.seeding.np_random()
        p.setTimeStep(1/30, self.client)
        p.setPhysicsEngineParameter(restitutionVelocityThreshold=0)
        self.useRealTime = 0
        self.robot = None
        self.ball = None
        self.action_space = gym.spaces.box.Box(0,1, dtype=np.float32)
        self.observation_space = gym.spaces.box.Box(-10 ,10, dtype=np.float32)
        self.reset()
       
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10, physicsClientId=self.client)
        p.setTimeStep(0.01, physicsClientId=self.client)
        p.setRealTimeSimulation(0, physicsClientId=self.client)
        Plane(self.client)
        self.ball = Ball(self.client)
        self.robot = Robot(self.client)
        self.done = False  
        self.reward = 0
        self.observation = self.ball.get_observation()
        return self.observation
    
    def step(self, action):
        #hard code the action for now
        #action = p.getBasePositionAndOrientation(self.robot)[0] + [.01,0,0]
        current_pos = self.robot.get_observation()[0]
        print(current_pos)
        action = [0.5,0.5,0.5] #current_pos + (.01,.02,.03)
        self.robot.apply_action(action)
        p.stepSimulation(physicsClientId=self.client)
        self.observation = self.ball.get_observation()
        self.reward = 1
        self.done = False
        return self.observation, self.reward, self.done, dict()
   
    def render(self, mode='human'):
        pass
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def close(self):
        p.disconnect(self.client)
