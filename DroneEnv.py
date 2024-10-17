import gym
import numpy as np

class DroneEnv(gym.Env):
    def __init__(self):
        super(DroneEnv, self).__init__()
        self.action_space = gym.spaces.Discrete(4)
        self.observation_space = gym.spaces.Box(low=0, high=1, shape=(10,))

    def reset(self):
        return np.zeros(self.observation_space.shape)

    def step(self, action):
        reward = self._calculate_reward(action)
        done = False
        new_observation = np.random.rand(10)
        return new_observation, reward, done, {}

    def _calculate_reward(self, action):
        return 1 if action == 1 else -1
