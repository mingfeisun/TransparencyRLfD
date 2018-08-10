#!/usr/bin/env python
import rospy
import math
import numpy
from main.srv import *

from QLearningModel import QLearningModel

from collections import defaultdict

class LearningFromDemo:
    def __init__(self):
        # four actions: 0(left), 1(up), 2(right), 3(down)
        self.model = QLearningModel([0, 1, 2, 3])
        rospy.Service('update_learning', LearningDemo, self.cb_learning)
        rospy.Service('query_action', QueryAction, self.cb_queryAction)
        # rospy.Service('query_action', QueryAction, self.cb_queryAction_potential)
        rospy.Service('reset_demo', ResetDemoLearning, self.cb_reset)

        self.potential = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

    def state_distance(self, _s, _s_demo):
        s_i = int(_s/10)
        s_j = int(_s%10)
        s_demo_i = int(_s_demo/10)
        s_demo_j = int(_s_demo%10)

        return abs(s_demo_i - s_i) + abs(s_demo_j - s_j)

    def compute_potential(self, _s, _s_demo):
        diff = self.state_distance(_s, _s_demo)

        # for testing
        # if diff == 0: 
        #     return 1.0
        # else:
        #     return 0.0

        return math.exp(-2 * diff * diff)

    def update_potential(self, _s_demo, _a_demo):
        for each_s in range(99):
            pot = self.compute_potential(each_s, _s_demo)
            self.potential[each_s][_a_demo] += pot

    def cb_learning(self, _req):
        s_demo = _req.state
        a_demo = _req.action
        r_demo = _req.reward
        ns_demo = _req.next_state

        self.update_potential(s_demo, a_demo)

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

    def cb_reset(self, _req):
        self.model.reset()
        return ResetDemoLearningResponse(True)