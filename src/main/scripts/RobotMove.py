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

import sys
import copy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from main.msg import CupMoveAction, CupMoveGoal, CupMoveResult, CupMoveFeedback

class RobotMove:
    def __init__(self):
        self.cup_pos_ctrl = CupPoseControl()

        self.client = actionlib.SimpleActionClient('teleop_cup', CupMoveAction)
        self.client.wait_for_server()

        rospy.wait_for_service('update_learning')
        self.update_learning = rospy.ServiceProxy('update_learning', LearningDemo)

        rospy.wait_for_service('query_action')
        self.query_action = rospy.ServiceProxy('query_action', QueryAction)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.group_left_gripper = moveit_commander.MoveGroupCommander("left_gripper")
        self.group_left_arm_torso = moveit_commander.MoveGroupCommander("left_arm_and_torso")

        self.group_right_arm = moveit_commander.MoveGroupCommander("right_arm")
        self.group_right_gripper = moveit_commander.MoveGroupCommander("right_gripper")
        self.group_right_arm_torso = moveit_commander.MoveGroupCommander("right_arm_and_torso")

        self.group_torso = moveit_commander.MoveGroupCommander("torso")
        self.group_base = moveit_commander.MoveGroupCommander("base")

        self.group_whole = moveit_commander.MoveGroupCommander("whole_body")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        self.current_pose_right_arm = self.group_right_arm.get_current_pose().pose


    def initCup(self):
        # set to init position
        self.cup_pos_ctrl.setPoseDefault()

    def moveArmTo(self, _pose_goal):
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 1.0
        # pose_goal.position.x = 0.4
        # pose_goal.position.y = 0.1
        # pose_goal.position.z = 0.4
        self.group_right_arm.set_pose_target(_pose_goal)
        self.group_right_arm.go(wait=True)
        self.group_right_arm.stop()
        self.group_right_arm.clear_pose_targets()
        self.current_pose_right_arm = self.group_right_arm.get_current_pose().pose

    def test_moveArmTo(self, _pose_goal):
        pose_goal = _pose_goal
        pose_goal.position.x += 0.45
        # pose_goal.position.y += 0.45
        pose_goal.position.z = 0.95
        self.group_right_arm_torso.set_pose_target(pose_goal)
        self.group_right_arm_torso.go(wait=True)
        # self.group_right_arm.stop()
        # self.group_right_arm.clear_pose_targets()

    # for testing
    def test_moveArmRandom(self):
        pose_goal = self.group_right_arm.get_current_pose().pose
        pose_goal.position.x -= random.uniform(-0.5, 0.5)
        pose_goal.position.y -= random.uniform(-0.5, 0.5)
        self.group_right_arm.set_pose_target(pose_goal)
        self.group_right_arm.go(wait=True)
        self.group_right_arm.stop()
        self.group_right_arm.clear_pose_targets()

    def cb_action_request(self, _feedback):
        target_pose = _feedback.cup_pose
        self.moveArmTo(target_pose)

    def moveCup(self):
        # set cup pose to init position
        self.cup_pos_ctrl.setPoseDefault()

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

            self.client.send_goal(curr_goal, feedback_cb=self.cb_action_request)
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

        self.cup_pos_ctrl.setPoseDefault()
