# Simulating Juggling a Ping-Pong Ball with a Robotic Arm Using Reinforcement Learning
## Authors: Selina Spry, Travis Clauson, Srisharan Kolige
### Due Date: 9 May 2023

### Summary:

We completed this project as a part of Professor Jivko Sinapov's Probabilistc Robotics course at Tufts Univeristy. We used Pybullet and OpenAI Gym to create the simulation environment. Then, we trained the system using PyTorch and a hand-crafted Proximity Policy Optimization RL algorithm.  

Acknowledgement: https://github.com/GerardMaggiolino/Gym-Medium-Post

### Packages Required

Pybullet,
Open AI Gym,
Pytorch,
Time,
OS,
sys,
Nmpy,
MatPlotLib


### Structure of Code

### * Main.py
    # Executable Code
    # Get Reccomnded Action from PPO Agent
    # Complete the Action, returns new observation, reward, episode status
    
### * Environment.py
    # What is the Observation Space
    # What is the Action Space
    # How to "Step" the simulatoin
    # How to reset the environment
    # How to render the environment

  ####   - PPOagent.py
    # Keep track of state-action-reward instances
    # Choose an action based on exploration and/or expected rewards
    # Calculate Policy Gradient
    
  ####   - Robot.py
    # Initialize Robot
    # Obtain Observation data
    # Apply an Action
 
  ####   - Plane.py
    # Inititalize Plane

  ####   - Ball.py
    # Initialize Ball
  
