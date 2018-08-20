import numpy
import random
from collections import defaultdict

# code source: https://github.com/rlcode/reinforcement-learning/blob/master/1-grid-world/5-q-learning/q_learning_agent.py
class QLambdaLearningModel:
    def __init__(self, actions):
        # actions = [0, 1, 2, 3]
        self.actions = actions
        self.learning_alpha = 0.8
        self.discount_lambda = 0.5
        self.e_lambda = 0.9
        self.epsilon = 0.5
        self.anneal_decay = 0.0
        self.q_table = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])
        self.eligibility_traces = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

    # update q function with sample <s, a, r, s'>
    def learn(self, state, action, reward, next_state):
        self.epsilon -= self.anneal_decay
        current_q = self.q_table[state][action]

        self.eligibility_traces[state][action] += 1

        # using Bellman Optimality Equation to update q function
        new_q = reward + self.discount_lambda * max(self.q_table[next_state])

        delta = new_q - current_q

        for i in range(10):
            for j in range(10):
                state = str((i, j))
                for action in range(4):
                    self.q_table[state][action] += self.learning_alpha * delta * self.eligibility_traces[state][action]
                    self.eligibility_traces[state][action] *= self.discount_lambda * self.e_lambda

        self.print_Q_table(state, action, reward, next_state)
        self.print_eligibility_traces(state, action, reward, next_state)

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

    # max 
    def get_action_list(self, state):
        # take action according to the q function table
        state_action = self.q_table[state]
        return state_action[:]

    # get q-table
    def get_q_table(self):
        return self.q_table[:]

    # vis: 0(left), 1(up), 2(right), 3(down)
    def print_Q_table(self, _curr_state, _action, _reward, _next_state):
        with open('log/q_table_value.txt', 'w') as fout:
            fout.write("State \t Left \t Up \t Right \t Down \n")
            for i in range(10):
                for j in range(10):
                    state = str((i, j))
                    fout.write("(%2d, %2d) \t %.2f \t %.2f \t %.2f \t %.2f \n"
                        %(i, j, self.q_table[state][0],self.q_table[state][1], 
                        self.q_table[state][2],self.q_table[state][3]))
                    if state == _curr_state:
                        fout.write("reward: %.2f, next state: %s\n"%(_reward, _next_state))
                        fout.write("----------------------------------------\n")

    # vis: 0(left), 1(up), 2(right), 3(down)
    def print_eligibility_traces(self, _curr_state, _action, _reward, _next_state):
        with open('log/eligibility_traces.txt', 'w') as fout:
            fout.write("State \t Left \t Up \t Right \t Down \n")
            for i in range(10):
                for j in range(10):
                    state = str((i, j))
                    fout.write("(%2d, %2d) \t %.2f \t %.2f \t %.2f \t %.2f \n"
                        %(i, j, self.eligibility_traces[state][0],self.eligibility_traces[state][1], 
                        self.eligibility_traces[state][2],self.eligibility_traces[state][3]))
                    if state == _curr_state:
                        fout.write("reward: %.2f, next state: %s\n"%(_reward, _next_state))
                        fout.write("----------------------------------------\n")

    def reset(self):
        self.q_table = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

    def reset_eligibility_traces(self):
        self.eligibility_traces = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

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
