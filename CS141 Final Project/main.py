import gym
import torch
#from agent import PPOAgent
import Panda_Juggling
import time
from PPO_agent.agent import train
from PPO_agent.agent import test

def main():
    beginTime = time.time()

## ----------------- RL MODE ----------------- ##
    # 'train' will train a new model. 'test' will test a saved model.
    mode = 'train'

## ----------------- Actor/Critic ----------------- ##
    # Start New or Use Saved actor/critic model
    #actor_model = "ppo_actor.pth"
    #critic_model = "ppo_critic.pth"
    actor_model = ""
    critic_model = ""

## ----------------- Hyperparameters ----------------- ##
    # NOTE: Here's where you can set hyperparameters for PPO. I don't include them as part of
    # ArgumentParser because it's too annoying to type them every time at command line. Instead, you can change them here.
    # To see a list of hyperparameters, look in ppo.py at function _init_hyperparameters
    hyperparameters = {
          'total_timesteps': 10_000_000, 
          'timesteps_per_batch': 5_000, 
          'max_timesteps_per_episode': 2_000, 
          'gamma': 0.90, 
          'n_updates_per_iteration': 10,
          'lr': .01, 
          'clip': 0.2,
          'render': False, #controls whether to render the environment or not
          'render_every_i': 1,
          'train_verbose': False,
          'save_freq': 1,
          }
    
## ----------------- RUN ENVIRONMENT ----------------- ##
    env = gym.make('PandaJuggling-v0')
    env.reset()
    if mode == 'train':
        rewardLog = train(env=env, hyperparameters=hyperparameters, actor_model=actor_model, critic_model=critic_model, writeToFile=True)
    # else:
    #   test(env=env, actor_model="ppo_actor.pth", render = True, num_episodes=10)
    
    #env.reset()
    #test(env=env, actor_model="ppo_actor.pth", num_episodes=10)
    env.close()

## ----------------- FINAL LOGS ----------------- ##
    endTime = time.time()
    totalTime = endTime - beginTime
    print(f"Total time: {int(totalTime/60)} minutes {int(totalTime%60)} seconds")
    try:
      f = open(rewardLog, "a")
      f.write(f"\nTotal time: {int(totalTime/60)} minutes {int(totalTime%60)} seconds")
      f.close()
    except:
      print("Failed to write to file")



if __name__ == '__main__':
    main()

