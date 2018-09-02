#!/usr/bin/env python
import rospy
import math
import numpy
from scipy.stats import entropy

from main.srv import *

from QLearningModel import QLearningModel
from QLambdaLearningModel import QLambdaLearningModel

from collections import defaultdict

from std_msgs.msg import String

class LearningFromDemo:
    from teleop_cup_server import REWARD_GOAL

    def __init__(self):
        # four actions: 0(left), 1(up), 2(right), 3(down)
        # self.model = QLearningModel([0, 1, 2, 3])
        self.model = QLambdaLearningModel([0, 1, 2, 3])

        rospy.Service('update_learning_demo', LearningDemo, self.cb_learning_demo)
        rospy.Service('update_learning', LearningDemo, self.cb_learning)

        rospy.Service('query_action', QueryAction, self.cb_queryAction)
        rospy.Service('query_action_confidence', QueryActionConfidence, self.cb_queryActionConfidence)

        rospy.Service('query_match_traces', QueryMatchTraces, self.cb_queryMatchTraces)
        rospy.Service('query_avg_confidence', QueryAvgConfidence, self.cb_queryAvgconfidence)

        rospy.Service('query_iterations', QueryIterations, self.cb_queryIteration)

        rospy.Service('reset_demo', ResetDemoLearning, self.cb_reset)

        self.pub = rospy.Publisher('current_state', String, queue_size=1, latch=True)

        self.potential = defaultdict(lambda: [0.0, 0.0, 0.0, 0.0])

        self.invtemp = 1

        self.match_traces = 0.0
        self.match_traces_lambda = 0.60

        self.avg_confidence = 0.0
        self.num_demo = 0

        self.num_itr = 1
        rospy.loginfo('iterations: %d'%self.num_itr)

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
                # if self.potential[state][_a_demo] > pot:
                #     self.potential[state][_a_demo] = pot

    def get_next_state(self, _state, _action):
        # four actions: 0(left), 1(up), 2(right), 3(down)
        state = eval(_state)
        next_state_i = state[0]
        next_state_j = state[1]

        if _action == 0:
            next_state_j -= 1
            return str((next_state_i, next_state_j))

        if _action == 1:
            next_state_i -= 1
            return str((next_state_i, next_state_j))

        if _action == 2:
            next_state_j += 1
            return str((next_state_i, next_state_j))

        if _action == 3:
            next_state_i += 1
            return str((next_state_i, next_state_j))

        return

    def cb_learning(self, _req):
        s_demo = _req.state
        a_demo = _req.action
        r_demo = _req.reward
        ns_demo = _req.next_state

        # change reward
        na_demo = self.model.get_action_max(ns_demo)
        f_value = self.model.discount_lambda * self.potential[ns_demo][na_demo] - self.potential[s_demo][a_demo]
        new_reward =  r_demo + f_value

        # rospy.loginfo("Learning from demonstration: %s, %s, %s, %s"%(str(s_demo), str(a_demo), str(new_reward), str(ns_demo)))

        self.model.learn(s_demo, a_demo, new_reward, ns_demo)

        ## complete one episode
        if r_demo == self.REWARD_GOAL:
            self.match_traces = 0 # reset match traces
            self.avg_confidence = 0 # reset confidence

            self.model.print_eligibility_traces(self.num_itr)
            self.model.print_Q_table(self.num_itr)
            self.print_potential()

            self.model.complete_one_episode()

            self.num_itr += 1

        return LearningDemoResponse(True)

    def cb_queryIteration(self, _req):
        return QueryIterationsResponse(self.num_itr)

    def update_match_traces(self, s_demo, a_demo):
        a_policy = self.model.get_action_max_more(s_demo)
        if a_policy == a_demo:
            self.match_traces = self.match_traces_lambda * (self.match_traces + 1)
        else:
            self.match_traces = self.match_traces_lambda * self.match_traces
        rospy.loginfo('Match traces: %f'%self.match_traces)

    def update_avg_confidence(self, s_demo, a_demo):
        action_list = self.model.get_action_list(s_demo)
        confidence = self.calculateConfidence(action_list)

        a_policy = self.model.get_action_max_more(s_demo)
        if a_policy != a_demo:
            return
        
        self.num_demo += 1

        if self.avg_confidence < confidence:
            # self.avg_confidence = confidence
            self.avg_confidence += 1.0/self.num_demo * (confidence - self.avg_confidence) # running average

        rospy.loginfo('Average confidence: %f'%self.avg_confidence)

    def cb_learning_demo(self, _req):
        s_demo = _req.state
        a_demo = _req.action
        r_demo = _req.reward
        ns_demo = _req.next_state

        self.update_match_traces(s_demo, a_demo)
        self.update_avg_confidence(s_demo, a_demo)

        msg_state = String(ns_demo)
        self.pub.publish(msg_state)

        self.update_potential(s_demo, a_demo)
        # self.print_potential(s_demo)


        na_demo = self.model.get_action_max(ns_demo)
        f_value = self.model.discount_lambda * self.potential[ns_demo][na_demo] - self.potential[s_demo][a_demo]
        new_reward =  r_demo + f_value

        self.model.learn(s_demo, a_demo, new_reward, ns_demo)

        ## complete one episode
        if r_demo == self.REWARD_GOAL:
            # rospy.loginfo('Reach goal')
            self.match_traces = 0 # reset match traces
            self.avg_confidence = 0 # reset confidence

            self.model.print_eligibility_traces(self.num_itr)
            self.model.print_Q_table(self.num_itr)
            self.print_potential()

            self.model.complete_one_episode()

            self.num_itr += 1
            rospy.loginfo('iterations: %d'%self.num_itr)

        return LearningDemoResponse(True)

    def cb_queryAction_potential(self, _req):
        state_action = self.potential[_req.state]
        action = QLearningModel.arg_max(state_action)
        return action

    def cb_queryAction(self, _req):
        action = self.model.get_action(_req.state)
        return QueryActionResponse(action)

    def cb_queryMatchTraces(self, _req):
        return QueryMatchTracesResponse(self.match_traces)

    def cb_queryAvgconfidence(self, _req):
        return QueryAvgConfidenceResponse(self.avg_confidence)

    def calculateConfidence(self, _action_list):
        self.invtemp = numpy.log10(self.num_itr) # log decay
        action_array = numpy.array(_action_list)
        temp_prob = numpy.exp(self.invtemp*action_array) / numpy.sum(numpy.exp(self.invtemp*action_array))
        return entropy(temp_prob)

    def cb_queryActionConfidence(self, _req):
        state = _req.state

        action_list = self.model.get_action_list(state)
        action = self.model.get_action_max(state)

        confidence = self.calculateConfidence(action_list)
        next_state = self.get_next_state(state, action)

        result = QueryActionConfidenceResponse()
        result.action = action
        result.confidence = confidence
        result.next_state = next_state

        return result

    def cb_reset(self, _req):
        self.num_itr = 0
        self.model.reset()
        return ResetDemoLearningResponse(True)

    def print_potential(self):
        with open('log/%s-q_potential_value-%d.txt'%(rospy.get_param('username'), self.num_itr), 'w') as fout:
            fout.write("State \t Left \t Up \t Right \t Down \n")
            for i in range(10):
                for j in range(10):
                    state = str((i, j))
                    fout.write("(%2d, %2d) \t %.2f \t %.2f \t %.2f \t %.2f \n"
                        %(i, j, self.potential[state][0],self.potential[state][1], 
                        self.potential[state][2],self.potential[state][3]))