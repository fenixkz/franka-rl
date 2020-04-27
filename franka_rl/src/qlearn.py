
import random
import numpy as np
from collections import defaultdict
class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma):
        self.Q = defaultdict(lambda: np.zeros(3))
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions

    def learn(self, state1, state2, action, reward):
        maxQ_next = np.argmax(self.Q[state2])
        td_target = reward + self.gamma * self.Q[state2][maxQ_next]
        td_delta = td_target - self.Q[state1][action]
        self.Q[state1][action] += self.alpha * td_delta

    def chooseAction(self, state):

        best_action = np.argmax(self.Q[state])
        return best_action

    def printQ(self):
        for i, j in self.Q.items():
            print(i+":")
            print(j)
