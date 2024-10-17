def step(self, action):
    battery_status = self.check_battery_status()
    if battery_status < 20:
        return self.return_to_home()
    
    reward = self._calculate_reward(action)
    done = False
    new_observation = np.random.rand(10)
    return new_observation, reward, done, {}

def check_battery_status(self):
    return 25

def return_to_home(self):
    print("Returning to home due to low battery.")
