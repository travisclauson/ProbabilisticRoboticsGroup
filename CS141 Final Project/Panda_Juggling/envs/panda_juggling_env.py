import gym
import numpy as np
import math
import pybullet as p
from Panda_Juggling.resources.ball import Ball
from Panda_Juggling.resources.plane import Plane
from Panda_Juggling.resources.robot import Robot
import matplotlib.pyplot as plt

class PandaJugglingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.client = p.connect(p.GUI)
        self.reset()
       
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10, physicsClientId=self.client)
        p.setTimeStep(0.01, physicsClientId=self.client)
        p.setRealTimeSimulation(0, physicsClientId=self.client)
        self.plane = Plane(self.client)
        self.ball = Ball(self.client)
        self.robot = Robot(self.client)
        #self.goal = Goal(self.client)
        self.done = False  
        self.reward = 0
        self.observation = self.ball.get_observation()
        return self.observation
    
    def step(self, action):
        self.robot.apply_action(action)
        self.observation = self.ball.get_observation()
        self.reward = self.get_reward()
        self.done = self.get_done()
        return self.observation, self.reward, self.done, dict()
   
    def render(self, mode='human'):
        pass
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        pass

    def close(self):
        p.disconnect(self.client)
