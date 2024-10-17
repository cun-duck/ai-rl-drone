import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import gym
import numpy as np
from stable_baselines3 import PPO

class DroneEnv(gym.Env):
    def __init__(self):
        super(DroneEnv, self).__init__()
        self.action_space = gym.spaces.Discrete(6)
        self.observation_space = gym.spaces.Box(low=0, high=1, shape=(10,))
        rospy.init_node('ai_controller_node', anonymous=True)
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def step(self, action):
        if self.detect_obstacle():
            action = self.avoid_obstacle()
        
        if self.check_battery_status() < 20:
            return self.return_to_home()
        
        self.execute_action(action)
        reward = self._calculate_reward(action)
        done = False
        new_observation = np.random.rand(10)
        return new_observation, reward, done, {}

    def reset(self):
        return np.zeros(self.observation_space.shape)

    def detect_obstacle(self):
        return np.random.choice([True, False], p=[0.1, 0.9])

    def avoid_obstacle(self):
        return np.random.choice(self.action_space.n)

    def check_battery_status(self):
        return 25

    def return_to_home(self):
        print("Returning to home due to low battery.")
        return np.zeros(self.observation_space.shape), 0, True, {}

    def execute_action(self, action):
        if action == 0:
            self.move_forward()
        elif action == 1:
            self.move_backward()
        elif action == 2:
            self.turn_right()
        elif action == 3:
            self.turn_left()
        elif action == 4:
            self.ascend()
        elif action == 5:
            self.descend()

    def move_forward(self):
        pass

    def move_backward(self):
        pass

    def turn_right(self):
        pass

    def turn_left(self):
        pass

    def ascend(self):
        pass

    def descend(self):
        pass

    def _calculate_reward(self, action):
        return 1 if action in [0, 4] else -1

env = DroneEnv()
model = PPO("MlpPolicy", env, verbose=1)
episodes = 100

for episode in range(episodes):
    done = False
    obs = env.reset()
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        model.learn(total_timesteps=1000)

print("Training selesai")
rospy.spin()
