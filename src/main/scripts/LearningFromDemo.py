#!/usr/bin/env python
import rospy
import math
import numpy
from scipy.stats import entropy

from main.srv import *

from QLearningModel import QLearningModel
from collections import defaultdict

from std_msgs.msg import String

class LearningFromDemo:
    def __init__(self):
        # four actions: 0(left), 1(up), 2(right), 3(down)
        self.model = QLearningModel([0, 1, 2, 3])
        rospy.Service('update_learning_demo', LearningDemo, self.cb_learning_demo)
        rospy.Service('update_learning', LearningDemo, self.cb_learning)
        rospy.Service('query_action', QueryAction, self.cb_queryAction)
        rospy.Service('query_action_confidence', QueryActionConfidence, self.cb_queryActionConfidence)
        rospy.Service('reset_demo', ResetDemoLearning, self.cb_reset)

        self.pub = rospy.Publisher('current_state', String, queue_size=1, latch=True)

        self.potential = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

        self.invtemp = 1

    def state_distance(self, _s, _s_demo):
        s = eval(_s)
        s_demo = eval(_s_demo)
        return abs(s[0] - s_demo[0]) + abs(s[1] - s_demo[1])

    def compute_potential(self, _s, _s_demo):
        diff = self.state_distance(_s, _s_demo)
        return -2*math.exp(-2 * diff * diff)

    def update_potential(self, _s_demo, _a_demo):
        for i in range(10):
            for j in range(10):
                state = str((i, j))
                pot = self.compute_potential(state, _s_demo)
                self.potential[state][_a_demo] += pot

    def get_next_state(self, _state, _action):
        # four actions: 0(left), 1(up), 2(right), 3(down)
        state = eval(_state)
        next_state_i = state[0]
        next_state_j = state[1]

        if _action == 0:
            next_state_j -= 1
        if _action == 1:
            next_state_i -= 1
        if _action == 2:
            next_state_j += 1
        if _action == 3:
            next_state_i += 1

        return str((next_state_i, next_state_j))

    def cb_learning(self, _req):
        s_demo = _req.state
        a_demo = _req.action
        r_demo = _req.reward
        ns_demo = _req.next_state

        # change reward
        na_demo = self.model.get_action_max(ns_demo)
        f_value = self.model.discount_lambda * self.potential[ns_demo][na_demo] - self.potential[s_demo][a_demo]
        new_reward =  r_demo + f_value

        self.model.learn(s_demo, a_demo, new_reward, ns_demo)

        return LearningDemoResponse(True)

    def cb_learning_demo(self, _req):
        s_demo = _req.state
        a_demo = _req.action
        r_demo = _req.reward
        ns_demo = _req.next_state

        msg_state = String(ns_demo)
        self.pub.publish(msg_state)

        self.update_potential(s_demo, a_demo)
        self.print_potential(s_demo)

        na_demo = self.model.get_action_max(ns_demo)
        f_value = self.model.discount_lambda * self.potential[ns_demo][na_demo] - self.potential[s_demo][a_demo]
        new_reward =  r_demo + f_value

        self.model.learn(s_demo, a_demo, new_reward, ns_demo)
        return LearningDemoResponse(True)

    def cb_queryAction_potential(self, _req):
        state_action = self.potential[_req.state]
        action = QLearningModel.arg_max(state_action)
        return action

    def cb_queryAction(self, _req):
        action = self.model.get_action(_req.state)
        return QueryActionResponse(action)

    def calculateConfidence(self, _action_list):
        temp_prob = numpy.exp(self.invtemp*_action_list) / numpy.sum(numpy.exp(self.invtemp*_action_list))
        return entropy(temp_prob)

    def cb_queryActionConfidence(self, _req):
        state = _req.state

        action_list = self.model.get_action_list(state)
        action = self.model.arg_max(action_list)

        confidence = self.calculateConfidence(action_list)
        next_state = self.get_next_state(state, action)

        result = QueryActionConfidenceResponse()
        result.action = action
        result.confidence = confidence
        result.next_state = next_state

        return result

    def cb_reset(self, _req):
        self.model.reset()
        return ResetDemoLearningResponse(True)

    def print_potential(self, _curr_state):
        with open('q_potential_value.txt', 'w') as fout:
            fout.write("State \t Left \t Up \t Right \t Down \n")
            for i in range(10):
                for j in range(10):
                    state = str((i, j))
                    fout.write("(%2d, %2d) \t %.2f \t %.2f \t %.2f \t %.2f \n"
                        %(i, j, self.potential[state][0],self.potential[state][1], 
                        self.potential[state][2],self.potential[state][3]))
                    if state == _curr_state:
                        fout.write("----------------------------------------\n")