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
        self.collision_count = 0
        self.action_space = gym.spaces.box.Box(low=np.array([-1,-1,0,-1,-1,-10,-1,-1,0]), high=np.array([1,1,1,1,1,10,1,1,1]), dtype=np.float32)
        self.observation_space = gym.spaces.box.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.observation_space = gym.spaces.box.Box(low=np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0]), high=np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), dtype=np.float32)
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
        # self.observation = self.ball.get_observation()
        self.robot_obs = self.robot.get_observation()
        self.ball_obs = self.ball.get_observation()
        self.observation = self.robot_obs + self.ball_obs
        return self.observation
    
    def step(self, action):
        #hard code the action for now
        # apply_action() requires parameter of type list
        current_pos = self.robot.get_observation()[0]
        np_current = np.asarray(current_pos)
        np_action = np_current + np.array([0.01,0.01,0.01])
        action = np_action.tolist()

        # the real code below used by agent
        self.robot.apply_action(action)
        p.stepSimulation(physicsClientId=self.client)   
        self.robot_obs = self.robot.get_observation()
        self.ball_obs = self.ball.get_observation()
        self.observation = self.robot_obs + self.ball_obs
        # print("robot_obs: ", self.robot_obs)
        # print("ball_obs: ", self.ball_obs)
        # print("self.observation:",self.observation)
        self.reward = self.calculateReward(self.observation)
        self.done = False
        return self.observation, self.reward, self.done, dict()
   
    def render(self, mode='human'):
        pass
    
    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def close(self):
        p.disconnect(self.client)

    def calculateReward(self, observation):
        reward = 0
        threshold = 0.2

        # need the collision observation and the x-y coordinates of the ball nad the end effector
        distance = math.sqrt((observation[0] - observation[7])**2 + (observation[1] - observation[8])**2)
        if distance > threshold :
            reward = 0
        else:
            reward = 1 - (distance/threshold)

        curr_contact = p.getContactPoints(self.robot.get_ids()[0],self.ball.get_ids()[0])
        if(curr_contact != ()):
            reward += 10
            self.collision_count += 1

        return reward
