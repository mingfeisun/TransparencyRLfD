#!/usr/bin/env python
import sys
import math
import rospy
import actionlib
from main.msg import CupMoveAction, CupMoveGoal, CupMoveResult, CupMoveFeedback

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

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

def checkAction(_position):
    cubic_size   = rospy.get_param('table_params/cubic_size')
    mat_size     = rospy.get_param('table_params/mat_size')
    cup_size     = rospy.get_param('table_params/cup_size')
    mat_pos      = rospy.get_param('table_params/mat_pos')
    margin_size  = rospy.get_param('table_params/margin_size')
    grid_size    = rospy.get_param('table_params/grid_size')
    table_size   = rospy.get_param('table_params/table_size')
    table_config = rospy.get_param('table_params/table_config')

    left_top_corner_x = -table_size/2 + margin_size
    left_top_corner_y = -table_size/2 + margin_size

    right_bottom_corner_x = table_size/2 - margin_size
    right_bottom_corner_y = table_size/2 - margin_size

    target_x = _position[0]
    target_y = _position[1]

    action_result = OK_TO_GO

    if target_x <= left_top_corner_x or target_x >= right_bottom_corner_x:
        action_result = OUT_OF_TABLE
        return action_result

    if target_y <= left_top_corner_y or target_y >= right_bottom_corner_y:
        action_result = OUT_OF_TABLE
        return action_result

    if abs(target_x - mat_pos[0]) < grid_size/4 and abs(target_y - mat_pos[1]) < grid_size/4:
        action_result = REACH_GOAL
        return action_result

    for i in table_config:
        for j in table_config[i]:
            if table_config[i][j] != "cubic":
                continue
            pos_x = grid_size*int(i) + grid_size/2 - table_size/2
            pos_y = grid_size*int(j) + grid_size/2 - table_size/2
            if abs(target_x - pos_x) <= cup_size+cubic_size/2 and abs(target_y - pos_y) <= cup_size+cubic_size/2:
                action_result = COLLISION
                return action_result

    return action_result

def do_moveCup(_req):
    action_res = CupMoveResult()

    step_size = 50
    time_factor = 1
    granuality = 1

    if rospy.has_param('table_params'):
        pre_position = rospy.get_param('table_params/cup_pos')
        grid_size = rospy.get_param('table_params/grid_size')
    else:
        action_res.complete_status = False
        action_res.reward = REWARD_NOT_READY
        action_res.distance_to_go = 0
        server.set_aborted(action_res, 'Table not ready yet')
        return

    # rospy.loginfo('x= %f, y= %f'%(_req.x, _req.y))
    delta_position = [_req.x*grid_size*granuality, _req.y*grid_size*granuality, _req.z*grid_size*granuality]

    rate = rospy.Rate(step_size/time_factor)

    delta_x = delta_position[0]/step_size
    delta_y = delta_position[1]/step_size

    target_pose = Pose()

    target_pose.position.x = pre_position[0]
    target_pose.position.y = pre_position[1]
    target_pose.position.z = pre_position[2]

    model_info = ModelState()
    model_info.model_name = 'cup'
    model_info.reference_frame = 'world'

    for i in range(step_size):
        next_pos = []
        next_pos.append(target_pose.position.x + delta_x)
        next_pos.append(target_pose.position.y + delta_y)
        next_pos.append(0.79)

        check_result = checkAction(next_pos)

        if check_result == REACH_GOAL:
            action_res.complete_status = True
            action_res.reward = REWARD_GOAL
            action_res.distance_to_go = 0
            rospy.loginfo('Mission completed')
            server.set_preempted(action_res, 'Mission completed')
            rospy.set_param('table_params/cup_pos', [target_pose.position.x, target_pose.position.y, target_pose.position.z])
            return

        if check_result == COLLISION or check_result == OUT_OF_TABLE:
            action_res.complete_status = True
            action_res.reward = REWARD_COLLISION
            action_res.distance_to_go = 0 # need to define
            rospy.loginfo('Collision detected')
            server.set_aborted(action_res, 'Collision detected')
            rospy.set_param('table_params/cup_pos', [target_pose.position.x, target_pose.position.y, target_pose.position.z])
            return

        if check_result == OK_TO_GO:
            rospy.loginfo('OK to go')
            target_pose.position.x = next_pos[0]
            target_pose.position.y = next_pos[1]
            target_pose.position.z = next_pos[2]

            model_info.pose = target_pose
            pub.publish(model_info)

            action_fb = CupMoveFeedback()
            action_fb.distance_moved = calDistance(next_pos, pre_position)
            server.publish_feedback(action_fb)

        rate.sleep()

    # set current position
    rospy.set_param('table_params/cup_pos', [target_pose.position.x, target_pose.position.y, target_pose.position.z])

    # request finished
    action_res.complete_status = True
    action_res.reward = REWARD_MOVE
    action_res.distance_to_go = 0 # need to define
    server.set_succeeded(action_res, 'Moving one step')

rospy.init_node('teleop_cup', anonymous=True)
pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

server = actionlib.SimpleActionServer('teleop_cup', CupMoveAction, do_moveCup, False)
server.start()
rospy.loginfo('starting service teleop_cup: finished')
rospy.spin()
