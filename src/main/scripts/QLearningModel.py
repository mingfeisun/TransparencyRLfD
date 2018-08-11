import numpy
import random
from collections import defaultdict

# code source: https://github.com/rlcode/reinforcement-learning/blob/master/1-grid-world/5-q-learning/q_learning_agent.py
class QLearningModel:
    def __init__(self, actions):
        # actions = [0, 1, 2, 3]
        self.actions = actions
        self.learning_alpha = 0.8
        self.discount_lambda = 0.5
        self.epsilon = 0.5
        self.q_table = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

    # update q function with sample <s, a, r, s'>
    def learn(self, state, action, reward, next_state):
        current_q = self.q_table[state][action]
        # using Bellman Optimality Equation to update q function
        new_q = reward + self.discount_lambda * max(self.q_table[next_state])
        self.q_table[state][action] += self.learning_alpha * (new_q - current_q)
        self.print_Q_table()

    # epsilon-greedy policy
    def get_action(self, state):
        if numpy.random.rand() < self.epsilon:
            # take random action
            action = numpy.random.choice(self.actions)
        else:
            # take action according to the q function table
            state_action = self.q_table[state]
            action = self.arg_max(state_action)
        return action

    # max 
    def get_action_max(self, state):
        # take action according to the q function table
        state_action = self.q_table[state]
        action = self.arg_max(state_action)
        return action

    # vis: 0(left), 1(up), 2(right), 3(down)
    def print_Q_table(self):
        with open('q_table_value.txt', 'w') as fout:
            fout.write("State \t Left \t Up \t Right \t Down \n")
            for i in range(99):
                each = i
            # for each in self.q_table:
                fout.write("%02d \t %.2f \t %.2f \t %.2f \t %.2f \n"
                    %(each, self.q_table[each][0],self.q_table[each][1], self.q_table[each][2],self.q_table[each][3]))

    def reset(self):
        self.q_table = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

    @staticmethod
    def arg_max(state_action):
        max_index_list = []
        max_value = state_action[0]
        for index, value in enumerate(state_action):
            if value > max_value:
                max_index_list = []
                max_value = value
                max_index_list.append(index)
            elif value == max_value:
                max_index_list.append(index)
        return random.choice(max_index_list)
