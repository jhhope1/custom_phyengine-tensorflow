import numpy as np
import tkinter as tk
import Robotengine as Rengine
'''state initialize:
    concatenate[thata, omega, rs, vs, Q, wb]
    action: numLeg * numsubleg 개의 omega
'''

np.random.seed(1)
vs_goal = np.array([1, 0, 0])
rz_goal = 0.25
rx_finish = 5
action_size = 12
state_size = 42

class Env(tk.Tk):
    def __init__(self):

        super(Env, self).__init__()
        self.R = Rengine.robot()
        self.action = np.zeros((action_size), dtype=np.float64)
        self.state = np.zeros((state_size), dtype=np.float64)
        self.action_size = 12  #action space 는 이제 lenth가 없음.
        self.state_size = 42
        self.counter = 0
        self.reward = 0

    def check_done(self):
        if self.R.body.rs.flatten()[0] > 5.:
            return 2
        if (self.R.joint_done() == 1):
            return 1
        if(self.R.body.rs.flatten()[2] > 0.6):
            return 1
        return 0

    def reset(self):
        self.R = Rengine.robot()
        self.R.set_constants()
        return self.get_state()

    def get_reward(self):
        self.reward = -np.mean(np.square(self.R.body.vs.flatten()-vs_goal))
        self.reward -= np.mean(np.square(self.R.body.rs.flatten()[2])-rz_goal)
        self.reward += 0.1
        if(self.check_done() == 1):
            self.reward = -100.
        if(self.check_done() == 2):
            self.reward = 100.
            ###################################### 앙 꺄르륵띠 여기 done부분과 같이 고쳐야될 듯

    def step(self, action):
        self.counter += 1
        self.move(action)
        self.get_state()
        done = self.check_done()
        self.get_reward()

        return self.state, self.reward, done

    def get_state(self):
        #  concatenate[thata, omega, rs, Q, wb]

        state = []
        for p in range(self.R.numLeg):
            for i in range(self.R.numsubleg):
                state.append(np.array([self.R.leg[p].sub[i].theta]))
        for p in range(self.R.numLeg):
            for i in range(self.R.numsubleg):
                state.append(np.array([self.R.leg[p].sub[i].omega]))
        state.append(self.R.body.rs.flatten())
        state.append(self.R.body.vs.flatten())
        state.append(self.R.body.Q.flatten())
        state.append(self.R.body.wb.flatten())
        self.state = np.concatenate(state).flatten()
        #  ----------------------------------------debugging needed

    def move(self, action):
        self.R.set_omega(action)
        self.R.timeflow()
