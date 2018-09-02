import numpy
import rospy
import random
from collections import defaultdict

# code source: https://github.com/rlcode/reinforcement-learning/blob/master/1-grid-world/5-q-learning/q_learning_agent.py
class QLambdaLearningModel:
    def __init__(self, actions):
        # actions = [0, 1, 2, 3]
        self.actions = actions
        self.learning_alpha = 0.5
        self.discount_lambda = 0.9
        self.e_lambda = 0.9
        self.epsilon = 0.5
        self.anneal_decay = 0.02
        self.q_table = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])
        self.eligibility_traces = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])


    # update q function with sample <s, a, r, s'>
    def learn(self, state, action, reward, next_state):
        self.epsilon -= self.anneal_decay
        if self.epsilon < 0.05:
            self.epsilon = 0.05
        current_q = self.q_table[state][action]

        self.eligibility_traces[state][action] += 1

        # using Bellman Optimality Equation to update q function
        best_action = numpy.argmax(self.q_table[state])

        new_q = reward + self.discount_lambda * max(self.q_table[next_state])

        delta = new_q - current_q

        for i in range(10):
            for j in range(10):
                tmp_state = str((i, j))
                for tmp_action in range(4):
                    self.q_table[tmp_state][tmp_action] += self.learning_alpha * delta * self.eligibility_traces[tmp_state][tmp_action]
                    # rospy.loginfo("q table for state (%s) and action (%s): %f"%(str(tmp_state), str(tmp_action), self.q_table[tmp_state][tmp_action]))
                    if action == best_action:
                        self.eligibility_traces[tmp_state][tmp_action] *= self.discount_lambda * self.e_lambda
                    else:
                        self.eligibility_traces[tmp_state][tmp_action] = 0
                    # rospy.loginfo("eligibility traces for state (%s) and action (%s): %f"%(str(tmp_state), str(tmp_action), self.eligibility_traces[tmp_state][tmp_action]))

        # self.print_Q_table()
        # self.print_eligibility_traces()

    def complete_one_episode(self):
        self.reset_eligibility_traces()

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
    def get_action_max_more(self, state):
        # take action according to the q function table
        state_action = self.q_table[state]
        action = self.arg_max_more(state_action)
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
    def print_Q_table(self, _itr):
        with open('log/%s-q_table_value-%d.txt'%(rospy.get_param('username'), _itr), 'w') as fout:
            fout.write("State \t Left \t Up \t Right \t Down \n")
            for i in range(10):
                for j in range(10):
                    state = str((i, j))
                    fout.write("(%2d, %2d) \t %.6f \t %.6f \t %.6f \t %.6f \n"
                        %(i, j, self.q_table[state][0],self.q_table[state][1], 
                        self.q_table[state][2],self.q_table[state][3]))

    # vis: 0(left), 1(up), 2(right), 3(down)
    def print_eligibility_traces(self, _itr):
        with open('log/%s-eligibility_traces-%d.txt'%(rospy.get_param('username'), _itr), 'w') as fout:
            fout.write("State \t Left \t Up \t Right \t Down \n")
            for i in range(10):
                for j in range(10):
                    state = str((i, j))
                    fout.write("(%2d, %2d) \t %.6f \t %.6f \t %.6f \t %.6f \n"
                        %(i, j, self.eligibility_traces[state][0],self.eligibility_traces[state][1], 
                        self.eligibility_traces[state][2],self.eligibility_traces[state][3]))

    def reset(self):
        self.q_table = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

    def reset_eligibility_traces(self):
        self.eligibility_traces = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

    @staticmethod
    def arg_max_more(state_action):
        max_index_list = []
        max_value = state_action[0]
        for index, value in enumerate(state_action):
            if value > max_value:
                max_index_list = []
                max_value = value
                max_index_list.append(index)
            elif value == max_value:
                max_index_list.append(index)
        if len(max_index_list) == 4:
            return -1
        return random.choice(max_index_list)

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