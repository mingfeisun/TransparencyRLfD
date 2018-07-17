#!/usr/bin/env python
import sys
import math
import rospy
import actionlib
from main.msg import CupMoveAction, CupMoveGoal, CupMoveResult, CupMoveFeedback

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

def calDistance(_pos1, _pos2):
    sum = 0.0
    for i in range(len(_pos1)):
        sum = sum + (_pos1[i] - _pos2[i]) * (_pos1[i] - _pos2[i])
    return math.sqrt(sum)

def do_moveCup(_req):
    action_res = CupMoveResult()

    step_size = 50
    time_factor = 1
    margin_distance = 0.05

    if rospy.has_param('table_params/cup_pos') and rospy.has_param('table_params/grid_size'):
        pre_position = rospy.get_param('table_params/cup_pos')
        grid_size = rospy.get_param('table_params/grid_size')
    else:
        action_res.res = False
        server.set_aborted(action_res, 'Cup does not exist')
        return

    # rospy.loginfo('x= %f, y= %f'%(_req.x, _req.y))
    delta_position = [_req.x*grid_size, _req.y*grid_size, _req.z*grid_size]

    rate = rospy.Rate(step_size/time_factor)

    delta_x = delta_position[0]/step_size
    delta_y = delta_position[1]/step_size

    this_pose = Pose()

    this_pose.position.x = pre_position[0]
    this_pose.position.y = pre_position[1]
    this_pose.position.z = pre_position[2]

    model_info = ModelState()
    model_info.model_name = 'cup'
    model_info.reference_frame = 'world'

    curr_pos = []
    for i in range(step_size):

        # request canceled
        if server.is_preempt_requested():
            this_pose.position.x = pre_position[0]
            this_pose.position.y = pre_position[1]
            this_pose.position.z = pre_position[2]
            model_info.pose = this_pose
            pub.publish(model_info)

            action_res.res = False
            server.set_preempted(action_res, 'Action canceled')
            return

        # action invalid
        if this_pose.position.x + delta_x >= table_size/2 - margin_distance or this_pose.position.x + delta_x <= -table_size/2 + margin_distance:
            curr_pos = [this_pose.position.x, this_pose.position.y, this_pose.position.z]
            break
        # action invalid
        if this_pose.position.y + delta_y >= table_size/2 - margin_distance or this_pose.position.y + delta_y <= -table_size/2 + margin_distance:
            curr_pos = [this_pose.position.x, this_pose.position.y, this_pose.position.z]
            break

        this_pose.position.y = this_pose.position.y + delta_y
        this_pose.position.x = this_pose.position.x + delta_x
        this_pose.position.z = 0.77

        curr_pos = []
        curr_pos.append(this_pose.position.x)
        curr_pos.append(this_pose.position.y)
        curr_pos.append(this_pose.position.z)

        model_info.pose = this_pose
        pub.publish(model_info)

        action_fb = CupMoveFeedback()
        action_fb.distance_left = calDistance(curr_pos, pre_position)
        server.publish_feedback(action_fb)

        rate.sleep()

    # set current position
    rospy.set_param('table_params/cup_pos', curr_pos)

    # request finished
    action_res.res = True
    server.set_succeeded(action_res, 'Cup moving successfully')

rospy.init_node('teleop_cup', anonymous=True)
pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)

table_size = 0.913

server = actionlib.SimpleActionServer('teleop_cup', CupMoveAction, do_moveCup, False)
server.start()
rospy.loginfo('starting service teleop_cup: finished')
rospy.spin()
