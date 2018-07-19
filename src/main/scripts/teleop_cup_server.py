#!/usr/bin/env python
import sys
import math
import rospy
import actionlib
from main.msg import CupMoveAction, CupMoveGoal, CupMoveResult, CupMoveFeedback

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

from spawn_table_models import getXYZFromIJ

COLLISION    = 0
OUT_OF_TABLE = 1
REACH_GOAL   = 2
OK_TO_GO     = 3

REWARD_COLLISION = -2
REWARD_MOVE      = -1
REWARD_GOAL      =  1
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

def do_moveCup(_req):
    action_res = CupMoveResult()

    if rospy.has_param('table_params'):
        pre_index = rospy.get_param('table_params/cup_pos')
        grid_size = rospy.get_param('table_params/grid_size')
        table_size = rospy.get_param('table_params/table_size')
        margin_size = rospy.get_param('table_params/margin_size')
    else:
        action_res.complete_status = False
        action_res.reward = REWARD_NOT_READY
        action_res.distance_to_go = 0
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
        action_res.distance_to_go = 0 # need to define
        rospy.loginfo('Collision detected')
        server.set_aborted(action_res, 'Collision detected')
        return

    x, y, z = getXYZFromIJ(pre_i, pre_j, grid_size, table_size, margin_size)

    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    model_info = ModelState()
    model_info.model_name = 'cup'
    model_info.reference_frame = 'world'

    step_size = 50
    time_factor = 1

    delta_x = move_i * grid_size/step_size
    delta_y = move_j * grid_size/step_size

    rate = rospy.Rate(step_size/time_factor)

    rospy.loginfo('OK to go')
    for i in range(step_size):
        target_pose.position.x = target_pose.position.x + delta_x
        target_pose.position.y = target_pose.position.y + delta_y
        target_pose.position.z = 0.79

        model_info.pose = target_pose
        pub.publish(model_info)

        action_fb = CupMoveFeedback()
        action_fb.distance_moved = delta_x*(i+1) + delta_y*(i+1)
        server.publish_feedback(action_fb)
        rate.sleep()

    action_res.complete_status = True
    if check_result == REACH_GOAL:
        action_res.reward = REWARD_GOAL
        action_res.distance_to_go = 0
        rospy.loginfo('Mission completed')
        server.set_preempted(action_res, 'Mission completed')
    if check_result == OK_TO_GO:
        action_res.reward = REWARD_MOVE
        action_res.distance_to_go = 0 # need to define
        rospy.loginfo('Moveing one step')
        server.set_succeeded(action_res, 'Moving one step')

    rospy.set_param('table_params/cup_pos', [target_i, target_j])


rospy.init_node('teleop_cup', anonymous=True)
pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

server = actionlib.SimpleActionServer('teleop_cup', CupMoveAction, do_moveCup, False)
server.start()
rospy.loginfo('starting service teleop_cup: finished')
rospy.spin()
