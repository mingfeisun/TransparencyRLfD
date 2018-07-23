#!/usr/bin/env python
import rospy
import actionlib

from main.srv import *

from CupPoseControl import CupPoseControl
from QLearningModel import QLearningModel

from autonomous_learning import position2State
from autonomous_learning import action2Position
from autonomous_learning import action2Goal

from main.msg import CupMoveAction

class RobotMove:
    def __init__(self):
        self.cup_pos = CupPoseControl()

        self.client = actionlib.SimpleActionClient('teleop_cup', CupMoveAction)
        self.client.wait_for_server()

        rospy.wait_for_service('update_learning')
        self.update_learning = rospy.ServiceProxy('update_learning', LearningDemo)

        rospy.wait_for_service('query_action')
        self.query_action = rospy.ServiceProxy('query_action', QueryAction)

    def initCup(self):
        # set to init position
        self.cup_pos.setPoseDefault()

    def moveCup(self):
        # set cup pose to init position
        self.cup_pos.setPoseDefault()

        # 0: left, 1: up, 2: right, 3: down
        if rospy.has_param('table_params'):
            beg_pos = rospy.get_param('table_params/cup_pos_init')
            dst_pos = rospy.get_param('table_params/mat_pos')
        else:
            rospy.loginfo('Table not configured yet')
            sys.exit(1)

        curr_state = position2State(beg_pos)
        goal_state = position2State(dst_pos)

        while curr_state != goal_state:
            curr_action = self.query_action(curr_state).action
            curr_goal = action2Goal(curr_action)

            self.client.send_goal(curr_goal)
            self.client.wait_for_result()

            result = self.client.get_result()
            reward = result.reward

            next_pos = []
            next_pos.append(result.state_x)
            next_pos.append(result.state_y)
            next_state = position2State(next_pos)

            self.update_learning(curr_state, curr_action, reward, next_state)
            curr_state = next_state

        rospy.loginfo("Robot's turn over")
        rospy.loginfo("Starting human's turn")

        self.cup_pos.setPoseDefault()
