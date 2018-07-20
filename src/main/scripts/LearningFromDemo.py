#!/usr/bin/env python
import rospy
from main.srv import *

from QLearningModel import QLearningModel

class LearningFromDemo:
    def __init__(self):
        # four actions: 0(left), 1(up), 2(right), 3(down)
        self.model = QLearningModel([0, 1, 2, 3])
        s1 = rospy.Service('update_learning', LearningDemo, self.cb_learning)
        s2 = rospy.Service('query_action', QueryAction, self.cb_queryAction)
        s3 = rospy.Service('reset_demo', ResetDemoLearning, self.cb_reset)

    def cb_learning(self, _req):
        self.model.learn(_req.state, _req.action, _req.reward, _req.next_state)
        return LearningDemoResponse(True)

    def cb_queryAction(self, _req):
        action = self.model.get_action(_req.state)
        return QueryActionResponse(action)

    def cb_reset(self, _req):
        self.model.reset()
        return ResetDemoLearningResponse(True)
