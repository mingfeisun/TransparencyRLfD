#!/usr/bin/env python
import sys
import rospy
import curses
import math
import actionlib

from main.srv import *
from main.msg import CupMoveAction, CupMoveGoal, CupMoveActionResult, CupMoveActionFeedback

from CupPoseControl import CupPoseControl

from LearningFromDemo import LearningFromDemo

FAKE_MODE = False

def action2Goal(_action):
    goal = CupMoveGoal()
    if _action == 0:
        goal.x = 0
        goal.y = -1
    if _action == 1:
        goal.x = -1
        goal.y = 0
    if _action == 2:
        goal.x = 0
        goal.y = 1
    if _action == 3:
        goal.x = 1
        goal.y = 0
    goal.time_factor = 100
    return goal

def action2Position(_action, _pos):
    pos = _pos[:]
    if _action == 0:
        pos[1] = pos[1] - 1
    if _action == 1:
        pos[0] = pos[0] - 1
    if _action == 2:
        pos[1] = pos[1] + 1
    if _action == 3:
        pos[0] = pos[0] + 1
    return pos

if __name__ == "__main__":
    rospy.init_node('teleop_cup', anonymous=True)
    if FAKE_MODE:
        client = actionlib.SimpleActionClient('teleop_cup_server_fake', CupMoveAction)
        client.wait_for_server()
    else:
        client = actionlib.SimpleActionClient('teleop_cup_server', CupMoveAction)
        client.wait_for_server()
        

    lfd = LearningFromDemo()

    rospy.wait_for_service('update_learning')
    client_learning = rospy.ServiceProxy('update_learning', LearningDemo)

    rospy.wait_for_service('query_action')
    client_action = rospy.ServiceProxy('query_action', QueryAction)

    rospy.wait_for_service('reset_demo')
    client_reset = rospy.ServiceProxy('reset_demo', ResetDemoLearning)

    client_reset(True)

    # 0: left, 1: up, 2: right, 3: down
    iteration_num = 54

    if rospy.has_param('table_params'):
        beg_pos = rospy.get_param('table_params/cup_pos_init')
        dst_pos = rospy.get_param('table_params/mat_pos')
    else:
        rospy.loginfo('Table not configured yet')
        sys.exit(1)

    iters = []
    counts = []

    cupPose = CupPoseControl()

    if FAKE_MODE:
        rospy.set_param('table_params/cup_pos', beg_pos)
    else:
        cupPose.setPoseDefault()

    for i in range(iteration_num):
        count_actions = 0
        curr_state = str((beg_pos[0], beg_pos[1]))
        goal_state = str((dst_pos[0], dst_pos[1]))

        # rospy.loginfo('Start status: %s'%str(curr_state))
        # rospy.loginfo('Goal status: %s'%str(goal_state))

        while curr_state != goal_state:
            # rospy.loginfo('Current state: %d'%(curr_state))

            curr_action = client_action(curr_state).action
            # rospy.loginfo('Current action: %d'%(curr_action))

            curr_goal = action2Goal(curr_action)

            client.send_goal(curr_goal)
            client.wait_for_result()

            result = client.get_result()
            reward = result.reward
            # rospy.loginfo('Reward received: %d'%(reward))

            next_state = str((result.state_x, result.state_y))
            client_learning(curr_state, curr_action, reward, next_state)

            curr_state = next_state
            # rospy.loginfo('Start status: %s'%str(curr_state))

            count_actions = count_actions + 1

        iters.append(i)
        counts.append(count_actions)
        rospy.loginfo('Reach goal! Actions taken: %d'%count_actions)

        cupPose.changeDefaultPoseToNext()

    rospy.loginfo(counts)

