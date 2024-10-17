import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import gym
from stable_baselines3 import PPO

rospy.init_node('ai_controller_node', anonymous=True)

arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

env = gym.make("DroneEnv-v0")
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
