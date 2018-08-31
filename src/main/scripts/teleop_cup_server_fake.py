#!/usr/bin/env python
import sys
import math
import rospy
import actionlib
from main.msg import CupMoveAction, CupMoveGoal, CupMoveResult, CupMoveFeedback

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

from CupPoseControl import CupPoseControl
from spawn_table_models import getXYZFromIJ

COLLISION    = 0
OUT_OF_TABLE = 1
REACH_GOAL   = 2
OK_TO_GO     = 3

REWARD_COLLISION = -1
REWARD_MOVE      = 0
REWARD_GOAL      = 10
REWARD_NOT_READY =  0

def calDistance(_pos1, _pos2):
    sum = 0.0
    for i in range(len(_pos1)):
        sum = sum + (_pos1[i] - _pos2[i]) * (_pos1[i] - _pos2[i])
    return math.sqrt(sum)

def checkAction(target_x, target_y):
    mat_pos = rospy.get_param('table_params/mat_pos')
    table_config = rospy.get_param('table_params/table_config')

    action_result = OK_TO_GO

    if target_x < 0 or target_y < 0:
        action_result = OUT_OF_TABLE
        return action_result

    if target_x >= 10 or target_y >= 10:
        action_result = OUT_OF_TABLE
        return action_result

    if target_x == mat_pos[0] and target_y == mat_pos[1]:
        action_result = REACH_GOAL
        return action_result

    if table_config[str(target_x)][str(target_y)] == "cubic":
        action_result = COLLISION

    return action_result

def cb_moveCup(_req):
    action_res = CupMoveResult()

    if rospy.has_param('table_params'):
        pre_index = rospy.get_param('table_params/cup_pos')
        grid_size = rospy.get_param('table_params/grid_size')
        table_size = rospy.get_param('table_params/table_size')
        margin_size = rospy.get_param('table_params/margin_size')
    else:
        action_res.complete_status = False
        action_res.reward = REWARD_NOT_READY
        action_res.state_x = 0
        action_res.state_y = 0
        action_res.distance_to_go = 0
        action_res.cup_pose = cupPoseCtrl.getPose()
        server.set_aborted(action_res, 'Table not ready yet')
        return

    pre_i = pre_index[0]
    pre_j = pre_index[1]

    move_i = _req.x
    move_j = _req.y

    target_i = pre_i + move_i
    target_j = pre_j + move_j

    check_result = checkAction(target_i, target_j)

    if check_result == COLLISION or check_result == OUT_OF_TABLE:
        action_res.complete_status = True
        action_res.reward = REWARD_COLLISION
        action_res.state_x = pre_i
        action_res.state_y = pre_j
        action_res.distance_to_go = 0 # need to define
        action_res.cup_pose = cupPoseCtrl.getPose()
        # rospy.loginfo('Collision detected')
        server.set_aborted(action_res, 'Collision detected')
        return

    x, y, z = getXYZFromIJ(target_i, target_j, grid_size, table_size, margin_size)

    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    # rospy.loginfo('OK to go')

    action_res.complete_status = True
    action_res.state_x = target_i
    action_res.state_y = target_j
    action_res.cup_pose = target_pose

    rospy.set_param('table_params/cup_pos', [target_i, target_j])

    if check_result == REACH_GOAL:
        action_res.reward = REWARD_GOAL
        action_res.distance_to_go = 0
        # rospy.loginfo('Mission completed')
        server.set_preempted(action_res, 'Mission completed')

    if check_result == OK_TO_GO:
        action_res.reward = REWARD_MOVE
        action_res.distance_to_go = 0 # need to define
        # rospy.loginfo('Moveing one step')
        server.set_succeeded(action_res, 'Moving one step')


rospy.init_node('teleop_cup_sever_fake', anonymous=True)
cupPoseCtrl = CupPoseControl()
server = actionlib.SimpleActionServer('teleop_cup_server_fake', CupMoveAction, cb_moveCup, False)
server.start()
rospy.loginfo('starting service teleop_cup_sever_fake: finished')
rospy.spin()
