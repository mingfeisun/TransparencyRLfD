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
from geometry_msgs.msg import Pose
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from control_msgs.msg import *
from trajectory_msgs.msg import *

from main.msg import CupMoveAction, CupMoveGoal, CupMoveResult, CupMoveFeedback

class RobotMoveURRobot:
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

        self.group_end = moveit_commander.MoveGroupCommander("endeffector")
        self.group_man = moveit_commander.MoveGroupCommander("manipulator")

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        self.current_pose = self.group_man.get_current_pose().pose


        self.joint_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.joint_client.wait_for_server()

        self.orien_x =  0.500274157622
        self.orien_y =  0.501706342807
        self.orien_z = -0.499590134138
        self.orien_w =  0.498423726035

        self.robot_pose = Pose()
        self.robot_pose.position.x = -0.65
        self.robot_pose.position.y = 0
        self.robot_pose.position.z = 0.8

    def initCup(self):
        # set to init position
        self.cup_pos_ctrl.setPoseDefault()

    def initRobotPose(self):
        JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # [144.593816163399, 5.754934304529601, 7.194142435155028, 10.61821127013265, 4.675844406769917, 7.934736338099062]
        Q1 = [0.009653124896662924, -0.6835756311532828, 1.0619281313412259, -0.3737989105267019, 0.009994925707914604, -0.001918946439335656]
        Q2 = [0.009653124896662924, -0.6835756311532828, 1.170799852990027, -1.9876127002995183, 4.681749171284383, 1.8825401280344316]
        # Q3 = [0.009653124896662924, -0.6835756311532828, 1.170799852990027, -1.9876127002995183, 4.681749171284383, 1.8825401280344316]
        # Q2 = [1.5,0,-1.57,0,0,0]
        # Q3 = [1.5,-0.2,-1.57,0,0,0]

        g = FollowJointTrajectoryGoal()

        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES

        g.trajectory.points = [
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(3.0)), 
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(6.0))
            # JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(6.0))
        ]

        self.joint_client.send_goal(g)
        try:
            self.joint_client.wait_for_result()
        except KeyboardInterrupt:
            self.joint_client.cancel_goal()

    def moveArmToCupTop(self):
        cup_pose = self.cup_pos_ctrl.getPose()
        target_x, target_y, target_z = self.cupPoseToRobotPose(cup_pose)
        self.moveArmTo(target_x, target_y, target_z)

    def moveArmToPose(self, _pose_goal):
        self.group_man.set_pose_target(_pose_goal)
        self.group_man.go(wait=True)
        self.group_man.stop()
        self.group_man.clear_pose_targets()
        self.current_pose = self.group_man.get_current_pose().pose

    def moveArmTo(self, _x, _y, _z):
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.x = self.orien_x
        pose_goal.orientation.y = self.orien_y
        pose_goal.orientation.z = self.orien_z
        pose_goal.orientation.w = self.orien_w

        pose_goal.position.x = _x
        pose_goal.position.y = _y
        pose_goal.position.z = _z

        self.group_man.set_pose_target(pose_goal)
        self.group_man.go(wait=True)
        self.group_man.stop()
        self.group_man.clear_pose_targets()
        self.current_pose = self.group_man.get_current_pose().pose

    def test_moveArmTo(self, _pose_goal):
        pose_goal = _pose_goal
        pose_goal.position.x += 0.45
        # pose_goal.position.y += 0.45
        pose_goal.position.z = 1.05
        self.group_man.set_pose_target(pose_goal)
        self.group_man.go(wait=True)
        # self.group_right_arm.stop()
        # self.group_right_arm.clear_pose_targets()

    # for testing
    def test_moveArmRandom(self):
        pose_goal = self.group_man.get_current_pose().pose
        print pose_goal

        x_diff = 0.1
        y_diff = 0.1
        if pose_goal.position.x > 0.9:
            x_diff = -0.1
        if pose_goal.position.x < 0.1:
            x_diff = 0.1
        if pose_goal.position.y > 0.5:
            y_diff = -0.1
        if pose_goal.position.y < -0.5:
            x_diff = 0.1
        pose_goal.position.x += x_diff
        pose_goal.position.y += y_diff
        pose_goal.position.z = 0.2
        pose_goal.orientation.x = self.orien_x
        pose_goal.orientation.y = self.orien_y
        pose_goal.orientation.z = self.orien_z
        pose_goal.orientation.w = self.orien_w

        self.group_man.set_pose_target(pose_goal)
        self.group_man.go(wait=True)
        self.group_man.stop()
        self.group_man.clear_pose_targets()

    def cb_action_request(self, _feedback):
        target_pose = _feedback.cup_pose
        self.moveArmToPose(target_pose)

    def moveCup(self):
        # set cup pose to init position
        self.cup_pos_ctrl.setPoseDefault()
        # self.initRobotPose()
        self.moveArmToCupTop()

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

    def cupPoseToRobotPose(self, _cup_pose):
        delta_x = _cup_pose.position.x - self.robot_pose.position.x
        delta_y = _cup_pose.position.y - self.robot_pose.position.y
        delta_z = _cup_pose.position.z - self.robot_pose.position.z + 0.15

        return delta_x, delta_y, delta_z

    def gesturing(self):
        pass

    def pausing(self):
        pass

if __name__ == "__main__":
    # for testing
    rospy.init_node('robot_move_ur', anonymous=True)
    test = RobotMoveURRobot()
    test.initRobotPose()
    test.moveArmToCupTop()
    # freq = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     test.test_moveArmRandom()
    #     freq.sleep()