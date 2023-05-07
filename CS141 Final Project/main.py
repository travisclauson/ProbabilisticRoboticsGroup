import gym
import torch
#from agent import PPOAgent
import Panda_Juggling
import time
from PPO_agent.agent import train
from PPO_agent.agent import test

def main():
    # nn = 
    # agent = PPOAgent(policy=nn)
    # agent.load_model('ppo_model.pth')
    # agent.train(Panda_Juggling-v0, seed=0, batch_size=5000, iterations=100, max_episode_length=250, verbose=True))
    # agent.save_model('ppo_model.pth')
    env = gym.make('PandaJuggling-v0')
    ob = env.reset()

    # Train or test, depending on the mode specified
    mode = 'test'
    actor_model = "ppo_actor.pth"
    critic_model = "ppo_critic.pth"
    # actor_model = ""
    # critic_model = ""
    # NOTE: Here's where you can set hyperparameters for PPO. I don't include them as part of
    # ArgumentParser because it's too annoying to type them every time at command line. Instead, you can change them here.
    # To see a list of hyperparameters, look in ppo.py at function _init_hyperparameters
    hyperparameters = {
          'timesteps_per_batch': 20_000, 
          'max_timesteps_per_episode': 2_000, 
          'gamma': 0.99, 
          'n_updates_per_iteration': 10,
          'lr': 3e-4, #
          'clip': 0.2,
          'render': True,
          'render_every_i': 10
          }
    if mode == 'train':
      train(env=env, hyperparameters=hyperparameters, actor_model=actor_model, critic_model=critic_model)
    else:
      test(env=env, actor_model=actor_model)

    for i in range(1000): #will be changed to while True eventually
        #action = env.action_space.sample()
        #action = agent(ob)
        #ob, reward, done, _ = env.step(action)
        
        # For now, the action is hard coded in the environment
        env.step(action= [0,0,0]) #will be changed to env.step(action)
        time.sleep(0.01)
        #if done:
          #  ob = env.reset()
    env.close()


if __name__ == '__main__':
    main()
