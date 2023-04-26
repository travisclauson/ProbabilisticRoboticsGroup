import gym
import torch
#from agent import PPOAgent
import Panda_Juggling
import time

def main():
    # nn = 
    # agent = PPOAgent(policy=nn)
    # agent.load_model('ppo_model.pth')
    # agent.train(Panda_Juggling-v0, seed=0, batch_size=5000, iterations=100, max_episode_length=250, verbose=True))
    # agent.save_model('ppo_model.pth')
    env = gym.make('PandaJuggling-v0')
    ob = env.reset()

    for i in range(1000): #will be changed to while True eventually
        #action = env.action_space.sample()
        action = [...] #hard code no action for now...
        #ob, reward, done, _ = env.step(action)
        env.step(action)
        time.sleep(0.01)
        #if done:
          #  ob = env.reset()
    env.close()

if __name__ == '__main__':
    main()